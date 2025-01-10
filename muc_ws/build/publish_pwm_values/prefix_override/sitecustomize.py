import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/muc/MUC-Hardware/muc_ws/install/publish_pwm_values'
