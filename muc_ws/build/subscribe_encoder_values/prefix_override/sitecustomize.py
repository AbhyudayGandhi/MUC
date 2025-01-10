import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anirudh/MUC-Hardware/muc_ws/install/subscribe_encoder_values'
