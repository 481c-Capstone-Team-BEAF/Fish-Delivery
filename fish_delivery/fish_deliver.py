"""Hand the caught fish to the child, driven directly by the game master.
Same pattern as fish_grab: a persistent HelloNode commanded over topics, with
the actual motion run on a worker thread so the executor stays free.
    /deliver_fish/start    std_msgs/Empty   begin a delivery
    /deliver_fish/release  std_msgs/Empty   child has the fish; stop waiting and
                                            retract now (the voice-command hook
                                            for later -- replaces the timer)
    /deliver_fish/stop      std_msgs/Empty   abort: retract and reset immediately
    /deliver_done           std_msgs/Bool    published when done (always True on a
                                            normal/early finish, False on abort)
The delivery motion is deliberately dumb and slow for safety near a child:
    1. tilt head down to find ArUco marker ID 90 on the floor
    2. lift to mid-height, angle the gripper down a bit
    3. extend the arm toward the child VERY SLOWLY, using marker depth
       to stop at a safe distance
    4. wait (default 30 s, or until /deliver_fish/release) for the child to grab
    5. retract, restore the starting pose
    6. publish /deliver_done so the game master returns to the start state
"""
import threading
import time
import traceback
import os
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty, Bool
import hello_helpers.hello_misc as hm
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

START_TOPIC   = "/deliver_fish/start"
RELEASE_TOPIC = "/deliver_fish/release"
STOP_TOPIC    = "/deliver_fish/stop"
DONE_TOPIC    = "/deliver_done"

COLOR_TOPIC = "/camera/color/image_raw"
INFO_TOPIC  = "/camera/color/camera_info"

ARUCO_DICT = aruco.DICT_5X5_1000
MARKER_ID  = 90


class FishDeliver(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        # Starts the node + its own MultiThreadedExecutor spin thread and sets
        # up the trajectory client / joint_states sub.
        hm.HelloNode.main(self, 'fish_deliverer', 'fish_deliverer',
                          wait_for_first_pointcloud=False)

        # marker for child delivery
        self.marker_id       = 90
        self.marker_size_m   = 0.20    # 200mm printed size

        # Tunable pose / timing (setup-dependent; tweak per session at launch).
        # Hardcoded to avoid conflicts with HelloNode's pre-declared parameters.
        self.lift_height     = 0.7     # m, arm lift for the hand-off
        self.wrist_pitch     = -0.3    # rad, negative = tip down a bit
        self.wrist_yaw       = 0.0     # rad, point straight across mat
        self.extend_target   = 0.45    # m, absolute wrist_extension at full reach
        self.extend_step     = 0.02    # m per step (smaller = smoother)
        self.extend_duration = 20.0    # s, total time for the reach
        self.wait_sec        = 30.0    # s, hold for the child to grab
        self.search_timeout  = 15.0    # s, how long to scan before giving up

        # Head sweep for marker search. TBD: tune to your setup.
        self.head_tilt_search = -0.5   # TBD: rad, negative = look down at floor
        self.head_pan_min     = -3.0  # TBD: rad, sweep start (one side)
        self.head_pan_max     =  -0.4   # TBD: rad, sweep end (other side)
        self.head_pan_steps   = 7      # TBD: number of poses across the sweep
        self.settle_per_pose  = 0.8    # TBD: s, dwell per pose before detecting
        self.safe_distance_m  = 0.5    # m, min distance to human during reach

        self._busy = False
        self._cancel = False
        self._wait_event = threading.Event()
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._dist_coeffs = None
        self._latest_frame = None
        self._frame_lock = threading.Lock()
        self._aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
        self._aruco_params = aruco.DetectorParameters()
        self._detector = aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
        self._optical_frame = None
        self._tf_broadcaster = TransformBroadcaster(self)

        self._debug_log_path = os.path.expanduser("~/fish_deliver_debug.log")
        self._debug_log = open(self._debug_log_path, "a")
        self.get_logger().info(f"Debug scan -> {self._debug_log_path}")

        cb = ReentrantCallbackGroup()

        self.create_subscription(
            Empty, START_TOPIC, self._on_start, 1, callback_group=cb)
        self.create_subscription(
            Empty, RELEASE_TOPIC, self._on_release, 1, callback_group=cb)
        self.create_subscription(
            Empty, STOP_TOPIC, self._on_stop, 1, callback_group=cb)
        self.create_subscription(
            Image, COLOR_TOPIC, self._on_image, 1, callback_group=cb)
        self.create_subscription(
            CameraInfo, INFO_TOPIC, self._on_camera_info, 1, callback_group=cb)
        self.done_pub = self.create_publisher(Bool, DONE_TOPIC, 10)

        self.get_logger().info(
            f"fish_deliverer up; publish Empty on {START_TOPIC} to deliver, "
            f"result on {DONE_TOPIC}.")

    def _on_camera_info(self, msg):
        if self._camera_matrix is not None:
            return  # already have it
        self._camera_matrix = np.array(msg.k).reshape((3, 3))
        self._dist_coeffs = np.array(msg.d)
        self._optical_frame = msg.header.frame_id
        self.get_logger().info("Camera info received; ready to detect markers.")

    def _on_image(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            return
        with self._frame_lock:
            self._latest_frame = frame

    def _on_start(self, _msg):
        if self._busy:
            self.get_logger().warn("Delivery already in progress; ignoring start.")
            return
        threading.Thread(target=self._run_deliver, daemon=True).start()

    def _on_release(self, _msg):
        if self._busy:
            self.get_logger().info("Release received; ending hold early.")
            self._wait_event.set()

    def _on_stop(self, _msg):
        if self._busy:
            self.get_logger().info("Stop requested; aborting after current move.")
            self._cancel = True
            self._wait_event.set()

    def _publish_done(self, ok):
        msg = Bool()
        msg.data = ok
        self.done_pub.publish(msg)

    def _get_joint(self, name):
        js = self.joint_state
        return js.position[js.name.index(name)]

    def _run_deliver(self):
        self._busy = True
        self._cancel = False
        self._wait_event.clear()
        try:
            if self.joint_state is None:
                self.get_logger().error("No joint states yet; cannot deliver.")
                self._publish_done(False)
                return

            # Optional: move the head to a preset search pose for better marker detection.
            if self._camera_matrix is None:
                self.get_logger().error("No camera info yet; cannot deliver.")
                self._publish_done(False)
                return

            marker_pos = self._find_marker()
            if marker_pos is None or self._cancel:
                self.get_logger().error("Marker not found; cannot deliver.")
                self._publish_done(False)
                return
            self.get_logger().info(f"Marker detected at {marker_pos}; starting delivery.")

            # Pass marker_pos into _deliver so arm uses actual depth to child
            self._publish_done(self._deliver(marker_pos))
        except Exception:
            self.get_logger().error(
                "Delivery failed:\n" + traceback.format_exc())
            self._publish_done(False)
        finally:
            self._busy = False

    def _find_marker(self):
        """
        Sweep the head across a pan range at a fixed downward tilt, checking each
        pose for Marker ID self.marker_id. Return its translation vector in the
        camera frame (x, y, z meters), or None if not found / cancelled.
        Re-sweeps until search_timeout elapses.
        """
        pans = np.linspace(self.head_pan_min, self.head_pan_max,
                           self.head_pan_steps)
        deadline = time.time() + self.search_timeout

        while time.time() < deadline and not self._cancel:
            for pan in pans:
                if self._cancel or time.time() >= deadline:
                    break

                self.move_to_pose({
                    'joint_head_pan':  float(pan),
                    'joint_head_tilt': self.head_tilt_search,
                }, blocking=True)
                time.sleep(self.settle_per_pose)

                with self._frame_lock:
                    frame = self._latest_frame.copy() \
                        if self._latest_frame is not None else None
                if frame is None:
                    self.get_logger().warn(f"pan={pan:.2f}: no camera frame")
                    continue

                self._debug_scan(pan, frame)
                tvec = self._detect_marker(frame)
                if tvec is not None:
                    self.get_logger().info(
                        f"Found marker {self.marker_id} at "
                        f"pan={pan:.2f}, tvec={tvec}.")
                    return tvec

        self.get_logger().warn(
            f"Marker {self.marker_id} not found after sweeping for "
            f"{self.search_timeout:.0f}s.")
        return None

    def _debug_scan(self, pan, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates = [
            ("5X5_50",   aruco.DICT_5X5_50),
            ("5X5_100",  aruco.DICT_5X5_100),
            ("5X5_250",  aruco.DICT_5X5_250),
            ("5X5_1000", aruco.DICT_5X5_1000),
            ("4X4_1000", aruco.DICT_4X4_1000),
            ("6X6_250",  aruco.DICT_6X6_250),
            ("ORIGINAL", aruco.DICT_ARUCO_ORIGINAL),
        ]
        found = []
        for name, d in candidates:
            det = aruco.ArucoDetector(
                aruco.getPredefinedDictionary(d), aruco.DetectorParameters())
            _, ids, _ = det.detectMarkers(gray)
            if ids is not None:
                found.append(f"{name}:{sorted(int(i) for i in ids.flatten())}")
        self._dbg(f"pan={pan:.2f} dicts: " + ("  ".join(found) if found else "NONE"))

    def _dbg(self, msg):
        self._debug_log.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
        self._debug_log.flush()

    def _detect_marker(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)
        if ids is None:
            return None
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id != self.marker_id:
                continue
            half = self.marker_size_m / 2.0
            obj_pts = np.array([
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ], dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, corners[i][0],
                self._camera_matrix, self._dist_coeffs)
            if ok:
                return tvec.flatten()
        return None

    def _deliver(self, marker_pos):
        # original position to revert to
        start_lift  = self._get_joint('joint_lift')
        start_ext   = self._get_joint('wrist_extension')
        start_pitch = self._get_joint('joint_wrist_pitch')
        start_yaw   = self._get_joint('joint_wrist_yaw')

        # Lift to default position
        self.move_to_pose({
            'joint_lift':        self.lift_height,
            'joint_wrist_pitch': self.wrist_pitch,
            'joint_wrist_yaw':   self.wrist_yaw,
        }, blocking=True)
        if self._cancel:
            return self._retract_and_reset(start_lift, start_ext,
                                           start_pitch, start_yaw, ok=False)

        # Compute extension from marker depth, leaving safe clearance.
        # marker_pos[2] is camera-frame z (distance from camera to marker).
        # ext_scale_factor is conservative — tune on real robot if arm
        # consistently over- or under-shoots.
        marker_depth  = float(marker_pos[2])
        current_ext   = self._get_joint('wrist_extension')
        ext_scale     = 0.8  # conservative: use 80% of depth as extension
        desired_ext   = current_ext + (marker_depth - self.safe_distance_m) * ext_scale
        extend_to     = min(desired_ext, self.extend_target)
        extend_to     = max(extend_to, current_ext)  # never retract in this step

        self.get_logger().info(
            f"Marker depth={marker_depth:.3f}m → extending to "
            f"{extend_to:.3f}m (safe clearance={self.safe_distance_m:.2f}m)")

        # Slow reach: step the wrist toward the absolute extend_target with a
        # pause between each step for slow motion
        ext_from = current_ext
        span = extend_to - ext_from
        n_steps = max(1, round(abs(span) / self.extend_step))
        pause_per_step = self.extend_duration / n_steps
        for i in range(1, n_steps + 1):
            if self._cancel:
                return self._retract_and_reset(start_lift, start_ext,
                                               start_pitch, start_yaw, ok=False)
            target = ext_from + span * (i / n_steps)
            self.move_to_pose({'wrist_extension': target}, blocking=True)
            if self._wait_event.wait(timeout=pause_per_step):
                break  # released or stopped mid-reach

        self.get_logger().info(
            f"Holding up to {self.wait_sec:.0f}s for the child to grab.")
        self._wait_event.wait(timeout=self.wait_sec)

        ok = not self._cancel
        return self._retract_and_reset(start_lift, start_ext,
                                       start_pitch, start_yaw, ok=ok)

    def _retract_and_reset(self, lift, ext, pitch, yaw, ok):
        # Retract first (away from the child), then restore the rest.
        self.move_to_pose({'wrist_extension': ext}, blocking=True)
        self.move_to_pose({
            'joint_lift':        lift,
            'joint_wrist_pitch': pitch,
            'joint_wrist_yaw':   yaw,
        }, blocking=True)
        self.get_logger().info("Delivery complete; pose reset." if ok
                               else "Delivery aborted; pose reset.")
        return ok


def main():
    node = FishDeliver()
    try:
        # HelloNode.main already spins the node on a daemon thread; just keep
        # the process alive.
        node.new_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()