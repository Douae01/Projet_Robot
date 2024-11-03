import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/douae/ENIB/SEN/Projet_Robot/pyqt_ihm_ros2_ws/install/py_ihm'
