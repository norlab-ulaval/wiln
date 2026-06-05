#!/usr/bin/env python3
"""
wiln_safety_monitor -- Passive safety watchdog for the WILN teach-and-repeat system.

Port of mtt_repeat_supervisor.py redesigned around topics only (zero services).

Responsibilities:
  - Monitor odometry heartbeat (ICP odom staleness).
  - Monitor replay/follower state topics for FAULTED conditions.
  - Optional e-stop integration (platform-specific topic).
  - Optional ICP score watchdog.
  - Publish "cancel" on /wiln/command when a fault is detected.
  - Publish aggregated safety status on /wiln/safety/state.

This node is intentionally PASSIVE -- it never initiates replay or recording.
It only publishes "cancel" when something goes wrong.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

# Import the generated WilnState message from wiln package
try:
    from wiln.msg import WilnState
except ImportError:
    # Fallback: will fail at runtime if msg not generated yet
    WilnState = None


# ---------------------------------------------------------------------------
# QoS profiles
# ---------------------------------------------------------------------------
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

TRANSIENT_LOCAL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class WilnSafetyMonitor(Node):
    """Passive watchdog: monitors state topics and publishes cancel on fault."""

    # WilnState constants
    IDLE      = 0
    RECORDING = 1
    PLAYING   = 2
    LOADING   = 3
    SAVING    = 4
    FAULTED   = 5

    def __init__(self):
        super().__init__('wiln_safety_monitor')

        # ----- Parameters -----
        self.declare_parameter('odom_topic',            '/mapping/icp_odom')
        self.declare_parameter('odom_timeout_s',        3.0)
        self.declare_parameter('check_rate_hz',         5.0)
        self.declare_parameter('estop_topic',           '')   # empty = disabled
        self.declare_parameter('icp_score_topic',       '')   # empty = disabled
        self.declare_parameter('icp_score_threshold',   0.1)  # above = bad
        self.declare_parameter('icp_score_window_s',    2.0)

        odom_topic         = self.get_parameter('odom_topic').value
        self._odom_timeout = self.get_parameter('odom_timeout_s').value
        check_hz           = self.get_parameter('check_rate_hz').value
        estop_topic        = self.get_parameter('estop_topic').value
        icp_score_topic    = self.get_parameter('icp_score_topic').value
        self._icp_thresh   = self.get_parameter('icp_score_threshold').value
        self._icp_window   = self.get_parameter('icp_score_window_s').value

        # ----- Internal state -----
        self._last_odom_time        = None
        self._replay_state          = self.IDLE
        self._follower_state        = self.IDLE
        self._estop_active          = False
        self._icp_score             = 0.0
        self._icp_bad_since         = None   # time when score went bad
        self._active                = False  # True when replay is running
        self._fault_published       = False  # avoid spamming cancel

        # ----- Publishers -----
        self._cmd_pub = self.create_publisher(String, '/wiln/command', RELIABLE_QOS)
        self._safety_state_pub = self.create_publisher(
            String, '/wiln/safety/state', TRANSIENT_LOCAL_QOS)
        self._ready_pub = self.create_publisher(
            Bool, '/wiln/safety/ready', TRANSIENT_LOCAL_QOS)

        # ----- Subscriptions -----
        self.create_subscription(Odometry, odom_topic, self._on_odom, BEST_EFFORT_QOS)

        if WilnState is not None:
            self.create_subscription(
                WilnState, '/wiln/replay/state',
                self._on_replay_state, TRANSIENT_LOCAL_QOS)
            self.create_subscription(
                WilnState, '/wiln/follower/state',
                self._on_follower_state, TRANSIENT_LOCAL_QOS)
        else:
            self.get_logger().warn('WilnState message not found — state monitoring disabled')

        if estop_topic:
            self.create_subscription(Bool, estop_topic, self._on_estop, BEST_EFFORT_QOS)
            self.get_logger().info(f'E-stop topic: {estop_topic}')

        if icp_score_topic:
            from std_msgs.msg import Float64
            self.create_subscription(
                Float64, icp_score_topic, self._on_icp_score, BEST_EFFORT_QOS)
            self.get_logger().info(f'ICP score topic: {icp_score_topic}')

        # ----- Watchdog timer -----
        self._timer = self.create_timer(1.0 / check_hz, self._watchdog_tick)

        self.get_logger().info('wiln_safety_monitor started')

    # -----------------------------------------------------------------------
    # Subscription callbacks
    # -----------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        self._last_odom_time = self.get_clock().now()

    def _on_replay_state(self, msg):
        prev = self._replay_state
        self._replay_state = msg.state
        if msg.state == self.PLAYING and prev != self.PLAYING:
            self._active = True
            self._fault_published = False
            self.get_logger().info('Replay PLAYING — safety watchdog active')
        elif msg.state == self.IDLE and prev == self.PLAYING:
            self._active = False
            self.get_logger().info('Replay IDLE — safety watchdog passive')
        elif msg.state == self.FAULTED:
            self.get_logger().error(f'Replay FAULTED: {msg.detail}')

    def _on_follower_state(self, msg):
        self._follower_state = msg.state
        if msg.state == self.FAULTED:
            self.get_logger().error(f'Follower FAULTED: {msg.detail}')

    def _on_estop(self, msg: Bool):
        self._estop_active = msg.data
        if msg.data and self._active:
            self.get_logger().error('E-STOP active during replay — cancelling')
            self._publish_cancel('e_stop_active')

    def _on_icp_score(self, msg):
        self._icp_score = msg.data
        now = self.get_clock().now()
        if msg.data > self._icp_thresh:
            if self._icp_bad_since is None:
                self._icp_bad_since = now
        else:
            self._icp_bad_since = None

    # -----------------------------------------------------------------------
    # Watchdog timer
    # -----------------------------------------------------------------------
    def _watchdog_tick(self):
        faults = []
        now = self.get_clock().now()

        # Only run full checks while replay is active
        if self._active:
            # 1. Odom timeout
            if self._last_odom_time is not None:
                age_s = (now - self._last_odom_time).nanoseconds * 1e-9
                if age_s > self._odom_timeout:
                    faults.append(f'odom_stale_{age_s:.1f}s')
            else:
                faults.append('odom_never_received')

            # 2. E-stop
            if self._estop_active:
                faults.append('estop')

            # 3. ICP score watchdog
            if self._icp_bad_since is not None:
                bad_s = (now - self._icp_bad_since).nanoseconds * 1e-9
                if bad_s > self._icp_window:
                    faults.append(f'icp_score_bad_{self._icp_score:.3f}_{bad_s:.1f}s')

            # 4. Follower faulted
            if self._follower_state == self.FAULTED:
                faults.append('follower_faulted')

        if faults:
            reason = '|'.join(faults)
            self._publish_cancel(reason)
            ready = False
        else:
            ready = True

        # Publish ready status
        self._publish_ready(ready)
        self._publish_safety_state(faults)

    # -----------------------------------------------------------------------
    # Publish helpers
    # -----------------------------------------------------------------------
    def _publish_cancel(self, reason: str):
        if self._fault_published:
            return
        self.get_logger().error(f'Safety fault [{reason}] — publishing cancel')
        msg = String()
        msg.data = 'cancel'
        self._cmd_pub.publish(msg)
        self._fault_published = True

    def _publish_ready(self, ready: bool):
        msg = Bool()
        msg.data = ready
        self._ready_pub.publish(msg)

    def _publish_safety_state(self, faults: list):
        msg = String()
        if faults:
            msg.data = 'FAULT:' + '|'.join(faults)
        else:
            msg.data = 'OK'
        self._safety_state_pub.publish(msg)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = WilnSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
