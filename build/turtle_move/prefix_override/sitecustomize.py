import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nathanielrobotics/turtlebot3_ws/install/turtle_move'
