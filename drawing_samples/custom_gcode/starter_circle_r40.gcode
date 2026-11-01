; --- Vong Tron Ban Kinh R=40mm ---
; Tam ve: X=-380.00, Y=0.00 | Cao do Z_draw=180.00, Z_lift=205.00
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500
G00 X-340.00 Y0.00 ; Diem khoi dau tren duong tron
G01 Z180.00 F1000
G02 X-340.00 Y0.00 I-40.00 J0.00 F1200 ; Cung tron 360 do kim dong ho
G00 Z205.00 F2500
G00 X-380.00 Y0.00
M30
