import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/ws/tutorial/ros2_ws/install/examples_rclpy_guard_conditions'
