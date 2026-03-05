#!/usr/bin/env python3
import rclpy, math, time, h5py
from rclpy.node import Node
from rclpy.action import ActionClient
from pathlib import Path
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from fleetsafe_msgs.msg import Episode, SafetyStatus

WPT = [(0,0,0),(5,0,0),(5,5,1.57),(0,5,3.14),(0,10,1.57),(10,10,0),(10,0,-1.57)]

class BaselineNavNode(Node):
    def __init__(self):
        super().__init__("baseline_nav")
        self.declare_parameter("data_dir", "/data/fleet_safe/baseline")
        self.declare_parameter("num_episodes", 50)
        self.declare_parameter("robot_id", "fastbot_0")
        self.D = Path(self.get_parameter("data_dir").value)
        self.N = self.get_parameter("num_episodes").value
        self.ID = self.get_parameter("robot_id").value
        self.D.mkdir(parents=True, exist_ok=True)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._odom = None
        self._viol = 0
        self._ep = 0
        self._wi = 0
        self.create_subscription(Odometry,"odom",lambda m:setattr(self,"_odom",m),10)
        self.create_subscription(SafetyStatus,"safety/status",self._scb,10)
        self._pub = self.create_publisher(Episode,"data/episode",10)
        self._h5 = h5py.File(str(self.D/f"baseline_{self.ID}.h5"),"a")
        self.get_logger().info(f"Ready -> {self.D}")
        self._t = self.create_timer(2.0, self._wait)

    def _scb(self, m):
        if not m.is_safe: self._viol += 1

    def _wait(self):
        if not self._nav.wait_for_server(timeout_sec=1.0): return
        self._t.cancel()
        self.get_logger().info("Nav2 up - starting")
        self._start = time.time()
        self._next()

    def _next(self):
        if self._ep >= self.N:
            self.get_logger().info("Done")
            self._h5.close()
            return
        x,y,yaw = WPT[self._wi % len(WPT)]
        self._wi += 1; self._viol = 0; self._start = time.time()
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw/2)
        g.pose.pose.orientation.w = math.cos(yaw/2)
        fh = self._nav.send_goal_async(g)
        fh.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected")
            self._next(); return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        elapsed = time.time() - self._start
        ep = Episode()
        ep.header.stamp = self.get_clock().now().to_msg()
        ep.robot_id = self.ID
        ep.episode_id = self._ep
        ep.time_to_goal = float(elapsed)
        ep.goal_reached = True
        ep.num_violations = self._viol
        self._pub.publish(ep)
        grp = self._h5.require_group(f"ep_{self._ep}")
        grp.attrs["time"] = elapsed
        grp.attrs["violations"] = self._viol
        self.get_logger().info(f"Ep {self._ep} done t={elapsed:.1f}s viol={self._viol}")
        self._ep += 1
        self._next()

def main():
    rclpy.init()
    node = BaselineNavNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
