import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gnanasai/MUC-Hardware-/install/subscribe_encoder_values'
