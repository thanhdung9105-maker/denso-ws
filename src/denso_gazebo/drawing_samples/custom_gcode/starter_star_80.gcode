; --- Ngoi Sao 5 Canh 80mm Can Doi ---
; Tam ve can doi: X=-380.00, Y=0.00 | Cao do: Z_draw=180.00, Z_lift=205.00
; Duong kinh ngoai D=80mm (R=40mm)
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-340.00 Y0.00 ; Dinh 1 (Top)
G01 Z180.00 F1000 ; Ha but tiep xuc mat giay
G01 X-403.51 Y-38.04 F1500 ; Dinh 3 (Bottom-Left)
G01 X-367.64 Y23.51 F1500 ; Dinh 5 (Top-Right)
G01 X-367.64 Y-23.51 F1500 ; Dinh 2 (Top-Left)
G01 X-403.51 Y38.04 F1500 ; Dinh 4 (Bottom-Right)
G01 X-340.00 Y0.00 F1500 ; Ve lai Dinh 1 khep kin
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-380.00 Y0.00 ; Ve tam san sang
M30
