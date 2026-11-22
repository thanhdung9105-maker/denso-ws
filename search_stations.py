import re

p = r'c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)\System\PEData_Goc_V20.plf.bak'
with open(p, 'rb') as f:
    d = f.read()

# Search for S7-1200 or CPU or PC station / WinCC Runtime
cpus = set(re.findall(rb'CPU [0-9A-Z/ ]+', d))
print('CPUs:', cpus)

pc_stations = set(re.findall(rb'(?:PC station|WinCC|SIMATIC|KTP[0-9]+|TP[0-9]+)[A-Za-z0-9_ ]*', d))
print('PC / HMI stations:', list(pc_stations)[:20])
