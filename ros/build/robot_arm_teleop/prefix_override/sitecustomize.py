import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sebas/robot-arm-teleop/ros/install/robot_arm_teleop'
