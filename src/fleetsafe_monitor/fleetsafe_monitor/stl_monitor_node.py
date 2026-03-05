#!/usr/bin/env python3
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
        super().__init__("stl_monitor")
        self.declare_parameter("robot_id","fastbot_0")
        self.declare_parameter("min_dist",0.3)
        self.declare_parameter("max_vel",1.5)
        self.RID = self.get_parameter("robot_id").value
        self.MD  = self.get_parameter("min_dist").value
        self.MV  = self.get_parameter("max_vel").value
        self._db = deque(maxlen=100)
        self._vb = deque(maxlen=100)
        self._t  = 0.0
        self._mc = self._mk("dist", f"always (dist > {self.MD})")
        self._ms = self._mk("vel",  f"always (vel  < {self.MV})")
        qos = QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,"scan",self._scan,qos)
        self.create_subscription(Odometry,"odom",self._odom,10)
        self._ps = self.create_publisher(SafetyStatus,"safety/status",10)
        self._pv = self.create_publisher(ConstraintViolation,"safety/violation",10)
        self.create_timer(0.1, self._eval)
        self.get_logger().info(f"STL Monitor [{self.RID}] ready")

    def _mk(self, var, spec):
        s = rtamt.StlDiscreteTimeSpecification()
        s.declare_var(var,"float"); s.spec = spec; s.parse(); return s

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
        rc = self._mc.update(td,[("dist",d)])
        rs = self._ms.update(tv,[("vel",v)])
        msg = SafetyStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robot_id = self.RID
        msg.collision_robustness = float(rc) if rc is not None else 0.0
        msg.speed_robustness     = float(rs) if rs is not None else 0.0
        msg.is_safe = ((rc is None or rc>0) and (rs is None or rs>0))
        self._ps.publish(msg)
        if rc is not None and rc<=0: self._vio("COLLISION",rc,d)
        if rs is not None and rs<=0: self._vio("SPEED",rs,v)

    def _vio(self,kind,rob,val):
        m=ConstraintViolation()
        m.header.stamp=self.get_clock().now().to_msg()
        m.robot_id=self.RID; m.constraint_name=kind
        m.robustness=float(rob); m.signal_value=float(val)
        self._pv.publish(m)
        self.get_logger().warn(f"VIOLATION {kind} rob={rob:.3f}")

def main():
    rclpy.init(); node=STLMonitorNode(); rclpy.spin(node)

if __name__=="__main__": main()
