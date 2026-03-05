from pathlib import Path
import os

R = Path.home() / "fleet_safe_vla_ws"

def w(path, content):
    p = R / path
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(content)
    print(f"wrote: {path}")

def pkg_xml(name, desc, deps=None):
    xml = '<?xml version="1.0"?>
<package format="3">
'
    xml += f'  <name>{name}</name>
  <version>0.1.0</version>
'
    xml += f'  <description>{desc}</description>
'
    xml += '  <maintainer email="frank@example.com">Frank</maintainer>
'
    xml += '  <license>Apache-2.0</license>
'
    xml += '  <buildtool_depend>ament_cmake</buildtool_depend>
'
    if deps:
        for d in deps: xml += f'  <depend>{d}</depend>
'
    xml += '  <export><build_type>ament_cmake</build_type></export>
</package>'
    return xml

# 1. fleetsafe_msgs
w("src/fleetsafe_msgs/package.xml", pkg_xml("fleetsafe_msgs", "Msgs", ["std_msgs", "geometry_msgs", "rosidl_default_generators", "rosidl_default_runtime"]))
w("src/fleetsafe_msgs/CMakeLists.txt", """cmake_minimum_required(VERSION 3.8)
project(fleetsafe_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  "msg/ConstraintViolation.msg"
  "msg/Episode.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()""")
w("src/fleetsafe_msgs/msg/SafetyStatus.msg", "std_msgs/Header header
string robot_id
float32 collision_robustness
float32 speed_robustness
bool is_safe")
w("src/fleetsafe_msgs/msg/ConstraintViolation.msg", "std_msgs/Header header
string robot_id
string constraint_name
float32 robustness
float32 signal_value")
w("src/fleetsafe_msgs/msg/Episode.msg", "std_msgs/Header header
string robot_id
uint32 episode_id
float32 total_reward
float32 total_cost
float32 path_length
float32 time_to_goal
bool goal_reached
uint32 num_violations")

# 2. fastbot_description
w("src/fastbot_description/package.xml", pkg_xml("fastbot_description", "URDF", ["robot_state_publisher", "xacro"]))
w("src/fastbot_description/CMakeLists.txt", """cmake_minimum_required(VERSION 3.8)
project(fastbot_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch DESTINATION share/${PROJECT_NAME})
ament_package()""")
w("src/fastbot_description/urdf/fastbot.urdf.xacro", """<?xml version="1.0"?>
<robot name="fastbot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="base_link"><visual><geometry><cylinder radius="0.175" length="0.1"/></geometry></visual></link>
</robot>""")

# 3. fastbot_nav2
w("src/fastbot_nav2/package.xml", pkg_xml("fastbot_nav2", "Nav2", ["nav2_bringup"]))
w("src/fastbot_nav2/CMakeLists.txt", "cmake_minimum_required(VERSION 3.8)
project(fastbot_nav2)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config maps launch DESTINATION share/${PROJECT_NAME})
ament_package()")

# 4. fastbot_bringup
w("src/fastbot_bringup/package.xml", pkg_xml("fastbot_bringup", "Bringup", ["rclpy", "nav2_msgs", "nav_msgs", "fleetsafe_msgs"]))
w("src/fastbot_bringup/CMakeLists.txt", """cmake_minimum_required(VERSION 3.8)
project(fastbot_bringup)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS fastbot_bringup/baseline_nav_node.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()""")
w("src/fastbot_bringup/fastbot_bringup/__init__.py", "")
w("src/fastbot_bringup/fastbot_bringup/baseline_nav_node.py", "#!/usr/bin/env python3
import rclpy
def main(): pass")

# 5. fleetsafe_monitor
w("src/fleetsafe_monitor/package.xml", pkg_xml("fleetsafe_monitor", "Monitor", ["rclpy", "sensor_msgs", "nav_msgs", "fleetsafe_msgs"]))
w("src/fleetsafe_monitor/CMakeLists.txt", """cmake_minimum_required(VERSION 3.8)
project(fleetsafe_monitor)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS fleetsafe_monitor/stl_monitor_node.py DESTINATION lib/${PROJECT_NAME})
ament_package()""")
w("src/fleetsafe_monitor/fleetsafe_monitor/__init__.py", "")
w("src/fleetsafe_monitor/fleetsafe_monitor/stl_monitor_node.py", "#!/usr/bin/env python3
import rclpy
def main(): pass")

print("Setup Complete")
