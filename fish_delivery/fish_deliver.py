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
    1. lift to mid-height, angle the gripper down a bit
    2. extend the arm ~0.4 m across the mat VERY SLOWLY (timed reach)
    3. wait (default 30 s, or until /deliver_fish/release) for the child to grab
    4. retract, restore the starting pose
    5. publish /deliver_done so the game master returns to the start state

Later the 30 s wait becomes a voice command: just publish on
/deliver_fish/release instead of relying on the timeout.
"""
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty, Bool
import hello_helpers.hello_misc as hm

START_TOPIC = "/deliver_fish/start"
RELEASE_TOPIC = "/deliver_fish/release"
STOP_TOPIC = "/deliver_fish/stop"
DONE_TOPIC = "/deliver_done"


class FishDeliver(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        # Starts the node + its own MultiThreadedExecutor spin thread and sets
        # up the trajectory client / joint_states sub.
        hm.HelloNode.main(self, 'fish_deliverer', 'fish_deliverer',
                          wait_for_first_pointcloud=False)

        # Tunable pose / timing (setup-dependent; tweak per session at launch).
        self.lift_height = self.declare_parameter(
            'lift_height', 0.7).value          # m, arm lift for the hand-off
        self.wrist_pitch = self.declare_parameter(
            'wrist_pitch', -0.3).value         # rad, negative = tip down a bit
        self.wrist_yaw = self.declare_parameter(
            'wrist_yaw', 0.0).value            # rad, point straight across mat
        self.extend_target = self.declare_parameter(
            'extend_target', 0.45).value       # m, absolute wrist_extension at full reach
        self.extend_step = self.declare_parameter(
            'extend_step', 0.02).value         # m per step (smaller = smoother)
        self.extend_duration = self.declare_parameter(
            'extend_duration', 20.0).value     # s, total time for the reach
        self.wait_sec = self.declare_parameter(
            'wait_sec', 30.0).value            # s, hold for the child to grab

        self._busy = False
        self._cancel = False
        self._wait_event = threading.Event()
        cb = ReentrantCallbackGroup()

        self.create_subscription(
            Empty, START_TOPIC, self._on_start, 1, callback_group=cb)
        self.create_subscription(
            Empty, RELEASE_TOPIC, self._on_release, 1, callback_group=cb)
        self.create_subscription(
            Empty, STOP_TOPIC, self._on_stop, 1, callback_group=cb)
        self.done_pub = self.create_publisher(Bool, DONE_TOPIC, 10)

        self.get_logger().info(
            f"fish_deliverer up; publish Empty on {START_TOPIC} to deliver, "
            f"result on {DONE_TOPIC}.")

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
            self._publish_done(self._deliver())
        finally:
            self._busy = False

    def _deliver(self):
        # original position to revert to
        start_lift = self._get_joint('joint_lift')
        start_ext = self._get_joint('wrist_extension')
        start_pitch = self._get_joint('joint_wrist_pitch')
        start_yaw = self._get_joint('joint_wrist_yaw')

        # Lift to default position
        self.move_to_pose({
            'joint_lift': self.lift_height,
            'joint_wrist_pitch': self.wrist_pitch,
            'joint_wrist_yaw': self.wrist_yaw,
        }, blocking=True)
        if self._cancel:
            return self._retract_and_reset(start_lift, start_ext,
                                           start_pitch, start_yaw, ok=False)

        # Slow reach: step the wrist toward the absolute extend_target with a
        # pause between each step for slow motion
        ext_from = self._get_joint('wrist_extension')
        span = self.extend_target - ext_from
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
            'joint_lift': lift,
            'joint_wrist_pitch': pitch,
            'joint_wrist_yaw': yaw,
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
