; --- Hinh Tam Giac Deu Canh 80mm ---
; Tam ve can doi: X=-380.00, Y=0.00 | Cao do: Z_draw=180.00, Z_lift=205.00
; 3 dinh: Dinh tren (-333.81, 0.00), Day trai (-403.09, -40.00), Day phai (-403.09, 40.00)
; Do dai 3 canh chinh xac: 80.00mm, 80.00mm, 80.00mm
G21 ; mm
G90 ; Toa do tuyet doi
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-333.81 Y0.00 ; Di chuyen den dinh tren
G01 Z180.00 F1000 ; Ha but tiep xuc mat giay
G01 X-403.09 Y-40.00 F1500 ; Canh 1 (80.00mm)
G01 X-403.09 Y40.00 F1500 ; Canh 2 (80.00mm)
G01 X-333.81 Y0.00 F1500 ; Canh 3 khep kin (80.00mm)
G00 Z205.00 F2500 ; Nhac but an toan
G00 X-380.00 Y0.00 ; Ve tam san sang
M30
