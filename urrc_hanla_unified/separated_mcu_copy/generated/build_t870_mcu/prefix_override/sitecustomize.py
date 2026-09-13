import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/werwerwer/urrc_hanla/urrc_hanla_unified/install/t870_mcu'
