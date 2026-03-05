from pathlib import Path
import os, math

R = Path.home() / "fleet_safe_vla_ws"

def w(path, content):
    p = R / path
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content)
    print("wrote:", path)

# Baseline Nav Node
BASELINE = """#!/usr/bin/env python3
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
\"\"\"

w("src/fastbot_bringup/fastbot_bringup/baseline_nav_node.py", BASELINE)

# STL Monitor
STL = \"\"\"#!/usr/bin/env python3
import rclpy, numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from collections import deque
import rtamt
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from fleetsafe_msgs.msg import SafetyStatus, ConstraintViolation

class STLMonitorNode(Node):
    def __init__(self):
        super().__init__(\"stl_monitor\")
        self.declare_parameter(\"robot_id\",\"fastbot_0\")
        self.declare_parameter(\"min_dist\",0.3)
        self.declare_parameter(\"max_vel\",1.5)
        self.RID = self.get_parameter(\"robot_id\").value
        self.MD  = self.get_parameter(\"min_dist\").value
        self.MV  = self.get_parameter(\"max_vel\").value
        self._db = deque(maxlen=100)
        self._vb = deque(maxlen=100)
        self._t  = 0.0
        self._mc = self._mk(\"dist\", f\"always (dist > {self.MD})\")
        self._ms = self._mk(\"vel\",  f\"always (vel  < {self.MV})\")
        qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,\"scan\",self._scan,qos)
        self.create_subscription(Odometry,\"odom\",self._odom,10)
        self._ps = self.create_publisher(SafetyStatus,\"safety/status\",10)
        self._pv = self.create_publisher(ConstraintViolation,\"safety/violation\",10)
        self.create_timer(0.1, self._eval)
        self.get_logger().info(f\"STL Monitor [{self.RID}] ready\")

    def _mk(self, var, spec):
        s = rtamt.StlDiscreteTimeSpecification()
        s.declare_var(var,\"float\"); s.spec = spec; s.parse(); return s

    def _scan(self, m):
        r = np.array(m.ranges)
        v = r[np.isfinite(r)&(r>m.range_min)]
        self._db.append((self._t, float(np.min(v)) if len(v)>0 else float(m.range_max)))

    def _odom(self, m):
        vx=m.twist.twist.linear.x; vy=m.twist.twist.linear.y
        self._vb.append((self._t, float(np.hypot(vx,vy))))

    def _eval(self):
        self._t += 0.1
        if len(self._db)<2 or len(self._vb)<2: return
        td,d = self._db[-1]; tv,v = self._vb[-1]
        rc = self._mc.update(td,[(\"dist\",d)])
        rs = self._ms.update(tv,[(\"vel\",v)])
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.RID
        msg.collision_robustness = float(rc) if rc is not None else 0.0
        msg.speed_robustness     = float(rs) if rs is not None else 0.0
        msg.is_safe = ((rc is None or rc>0) and (rs is None or rs>0))
        self._ps.publish(msg)
        if rc is not None and rc<=0: self._vio(\"COLLISION\",rc,d)
        if rs is not None and rs<=0: self._vio(\"SPEED\",rs,v)

    def _vio(self,kind,rob,val):
        m=ConstraintViolation()
        m.header.stamp=self.get_clock().now().to_msg()
        m.robot_id=self.RID; m.constraint_name=kind
        m.robustness=float(rob); m.signal_value=float(val)
        self._pv.publish(m)
        self.get_logger().warn(f\"VIOLATION {kind} rob={rob:.3f}\")

def main():
    rclpy.init(); node=STLMonitorNode(); rclpy.spin(node)

if __name__==\"__main__\": main()
\"\"\"

w(\"src/fleetsafe_monitor/fleetsafe_monitor/stl_monitor_node.py\", STL)
w(\"src/fleetsafe_monitor/fleetsafe_monitor/__init__.py\", \"\")

w(\"src/fleetsafe_monitor/package.xml\",
\"\"\"<?xml version=\\\"1.0\\\"?>
<package format=\\\"3\\\">
  <name>fleetsafe_monitor</name><version>0.1.0</version>
  <description>STL Safety Monitor</description>
  <maintainer email=\\\"frankleroyvan@gmail.com\\\">Frank</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclpy</depend><depend>sensor_msgs</depend>
  <depend>nav_msgs</depend><depend>fleetsafe_msgs</depend>
</package>\"\"\")

w(\"src/fleetsafe_monitor/CMakeLists.txt\",
\"\"\"cmake_minimum_required(VERSION 3.8)
project(fleetsafe_monitor)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(\${PROJECT_NAME})
install(PROGRAMS fleetsafe_monitor/stl_monitor_node.py DESTINATION lib/\${PROJECT_NAME})
ament_package()\"\"\")

# Mark files executable
os.chmod(str(R/\"src/fastbot_bringup/fastbot_bringup/baseline_nav_node.py\"), 0o755)
os.chmod(str(R/\"src/fleetsafe_monitor/fleetsafe_monitor/stl_monitor_node.py\"), 0o755)
print(\"Done\")
