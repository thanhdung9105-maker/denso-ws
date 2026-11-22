import os, re

p17 = r'c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3_V17\System\PEData.plf'
p20 = r'c:\Users\thanh\OneDrive\Máy tính\BTL_PLC_SCADA_Nhom3 (1)\System\PEData_Goc_V20.plf.bak'

with open(p17, 'rb') as f:
    d17 = f.read()
with open(p20, 'rb') as f:
    d20 = f.read()

# Extract ASCII strings >= 4 chars
def get_strings(data):
    return set(re.findall(rb'[A-Za-z0-9_]{4,}', data))

s17 = get_strings(d17)
s20 = get_strings(d20)

new_in_20 = s20 - s17
print('Strings in V20 but not in V17 (count):', len(new_in_20))

# Filter for interesting keywords
interesting = [s.decode('ascii', errors='ignore') for s in new_in_20 if any(k in s.lower() for k in [b'scada', b'hmi', b'tag', b'screen', b'db', b'fb', b'fc', b'motor', b'sensor', b'box', b'conveyor'])]
print('Interesting new items in V20 sample (50):', interesting[:50])
