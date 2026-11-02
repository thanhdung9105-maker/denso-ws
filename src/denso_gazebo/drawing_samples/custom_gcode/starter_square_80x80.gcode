; --- Hinh Vuong Chuan 80x80mm ---
; Tam ve: X=-380.00, Y=0.00 | Cao do Z_draw=180.00, Z_lift=205.00
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-420.00 Y-40.00 ; Di chuyen den goc bat dau
G01 Z180.00 F1000 ; Ha but tiep xuc ban ve
G01 X-340.00 Y-40.00 F1200 ; Canh 1
G01 X-340.00 Y40.00 F1200 ; Canh 2
G01 X-420.00 Y40.00 F1200 ; Canh 3
G01 X-420.00 Y-40.00 F1200 ; Canh 4 khep kin
G00 Z205.00 F2500 ; Nhac but
G00 X-380.00 Y0.00 ; Ve tam san sang
M30
