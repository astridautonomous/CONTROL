import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/irmak/diag_ws/src/diag_pkg/install/diag_pkg'
