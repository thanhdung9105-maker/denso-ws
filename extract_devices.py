import re

p = r'c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)\System\PEData_Goc_V20.plf.bak'
with open(p, 'rb') as f:
    d = f.read()

# Find device MLFB numbers (e.g., 6ES7...)
mlfb = set(re.findall(rb'6[A-Z0-9]{2}[0-9]-[0-9A-Z]{4,7}-[0-9A-Z]{3,4}', d))
print('MLFB Devices found:', mlfb)

# Find block names or network comments
blocks = set(re.findall(rb'(?:Main|OB[0-9]+|FB[0-9]+|FC[0-9]+|DB[0-9]+|[A-Za-z0-9_]+_DB)', d))
print('Blocks sample:', list(blocks)[:20])

# Find HMI screen names
hmi_screens = set(re.findall(rb'(?:Screen|HMI|SCADA|ManHinh|GiamSat)[A-Za-z0-9_]*', d))
print('HMI screens sample:', hmi_screens)
