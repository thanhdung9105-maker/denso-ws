import os
import pptx
from pptx import Presentation

# Load temporary presentation
pptx_path = 'c:/Users/thanh/denso_ws/temp_presentation.pptx'
prs = Presentation(pptx_path)

print(f"Loaded presentation: {len(prs.slides)} slides")

def replace_image_in_slide(slide, shape_name, new_image_path):
    for shape in slide.shapes:
        if shape.name == shape_name:
            rId = shape._element.blip_rId
            part = slide.part.related_part(rId)
            with open(new_image_path, 'rb') as f:
                part._blob = f.read()
            print(f"  [OK] Replaced image {shape_name} with {new_image_path}")
            return True
    print(f"  [WARN] Shape {shape_name} not found in slide")
    return False

# ==============================================================================
# SLIDE 5: MỤC TIÊU ĐỀ TÀI & CẤU HÌNH
# ==============================================================================
slide5 = prs.slides[4]
for sh in slide5.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "Hiệu chuẩn, bù nhiệt ẩm" in p.text:
                p.text = "Hiệu chuẩn 400.0 ppm, bù nhiệt ẩm Hanwei, Oversampling 20 mẫu và điều tốc quạt PWM 4 cấp"
            elif "Xây dựng mạch đo CO₂ (MQ-135)" in p.text:
                p.text = "Xây dựng mạch đo CO₂ (MQ-135), nhiệt độ – độ ẩm (DHT11), hiển thị LCD I²C và xuất 6 kênh tín hiệu dao động liên tục trên Arduino Serial Plotter"
print("Updated Slide 5 text")

# ==============================================================================
# SLIDE 7: CẢM BIẾN MQ-135
# ==============================================================================
slide7 = prs.slides[6]
replace_image_in_slide(slide7, 'eq_ppm.png', 'c:/Users/thanh/denso_ws/new_eq_ppm.png')

# ==============================================================================
# SLIDE 8: BÙ NHIỆT ẨM VÀ LỌC TÍN HIỆU
# ==============================================================================
slide8 = prs.slides[7]
for sh in slide8.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "Lọc tín hiệu số" in p.text:
                p.text = "Xử lý tín hiệu số & Serial Plotter"
replace_image_in_slide(slide8, 'eq_ema.png', 'c:/Users/thanh/denso_ws/new_eq_ema.png')

# ==============================================================================
# SLIDE 9: ĐẶC TUYẾN VÀ ĐỘ PHÂN GIẢI
# ==============================================================================
slide9 = prs.slides[8]
for sh in slide9.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "kEnvAdapt = 1,74" in p.text:
                p.text = "Mốc chuẩn 400,0 ppm · kEnvAdapt = 0,57 – 1,74 · 31,4 °C · 62,4 %RH"
    elif sh.has_table:
        table = sh.table
        # Table has rows: [Header, row1, row2, row3, row4]
        # Header: CO₂ (ppm) | Vout (V) | Δppm / LSB
        new_data = [
            ("400", "1,266", "5,7"),
            ("650", "1,439", "8,6"),
            ("950", "1,584", "11,9"),
            ("1300", "1,709", "15,6")
        ]
        for r_idx, row_vals in enumerate(new_data, start=1):
            if r_idx < len(table.rows):
                for c_idx, val in enumerate(row_vals):
                    table.rows[r_idx].cells[c_idx].text = val
        print("  [OK] Updated Slide 9 Table to new thresholds (400, 650, 950, 1300)")

replace_image_in_slide(slide9, 'dac_tuyen_b.png', 'c:/Users/thanh/denso_ws/new_dac_tuyen_b.png')

# ==============================================================================
# SLIDE 10: ĐIỀU KHIỂN QUẠT BẰNG PWM
# ==============================================================================
slide10 = prs.slides[9]
for sh in slide10.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "D9 (Timer1) phát PWM" in p.text:
                p.text = "D9 (Timer1) phát PWM ≈ 490 Hz, giá trị 0–255 (Full 100% 5V khi nguy cấp)"
            elif "Soft-ramp" in p.text:
                p.text = "Soft-ramp +14 / −6 mỗi 100 ms, tăng tốc êm, chống sụt áp nguồn USB"
            elif "PWM mục tiêu theo CO₂" in p.text:
                p.text = "PWM mục tiêu theo 4 cấp CO₂ (0 / 90–145 / 150–210 / 255), vùng trễ Schmitt 25 ppm"
replace_image_in_slide(slide10, 'dieu_khien_quat.png', 'c:/Users/thanh/denso_ws/new_dieu_khien_quat.png')

# ==============================================================================
# SLIDE 14: LƯU ĐỒ VÀ THÔNG SỐ CHƯƠNG TRÌNH
# ==============================================================================
slide14 = prs.slides[13]
for sh in slide14.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "setup() → loop() ≈ 350 ms" in p.text:
                p.text = "setup() → loop() ≈ 100 ms (~10 mẫu/giây)"
    elif sh.has_table:
        table = sh.table
        new_params = [
            ("Chu kỳ vòng lặp", "≈ 100 ms (~10 Hz)"),
            ("Lấy mẫu ADC", "Oversampling 20 mẫu / 400 µs"),
            ("Đọc DHT11", "2 s (kèm nhịp tim DHT_Live)"),
            ("R0", "61,50 kΩ (Calib 6 lần ngoài trời)"),
            ("kEnvAdapt", "0,57 – 1,74 (tự thích nghi phòng)"),
            ("Trễ Schmitt", "25 ppm")
        ]
        for r_idx, (param, val) in enumerate(new_params, start=1):
            if r_idx < len(table.rows):
                table.rows[r_idx].cells[0].text = param
                table.rows[r_idx].cells[1].text = val
        print("  [OK] Updated Slide 14 Table")

replace_image_in_slide(slide14, 'luu_do.png', 'c:/Users/thanh/denso_ws/new_luu_do.png')

# ==============================================================================
# SLIDE 18: ĐÁNH GIÁ HỆ THỐNG
# ==============================================================================
slide18 = prs.slides[17]
for sh in slide18.shapes:
    if sh.has_text_frame:
        full_txt = " ".join([p.text for p in sh.text_frame.paragraphs])
        if "Ưu điểm" in full_txt:
            sh.text_frame.clear()
            p0 = sh.text_frame.paragraphs[0]
            p0.text = "Ưu điểm"
            p0.font.bold = True
            bullets = [
                "Đo tỷ lệ, không phụ thuộc dao động nguồn USB",
                "Bù nhiệt ẩm Hanwei, Oversampling 20 mẫu chống đơ số liệu",
                "Hỗ trợ xuất đồng thời 6 kênh đồ thị dao động liên tục trên Arduino Serial Plotter",
                "Quạt khởi động mềm (+14/−6 mỗi 100 ms), trễ Schmitt 25 ppm chống rung giật",
                "LCD 1602 tự động phục hồi kết nối I²C khi chập chờn"
            ]
            for b in bullets:
                p = sh.text_frame.add_paragraph()
                p.text = b
        elif "Giới hạn" in full_txt:
            sh.text_frame.clear()
            p0 = sh.text_frame.paragraphs[0]
            p0.text = "Giới hạn"
            p0.font.bold = True
            bullets = [
                "MQ-135 cho CO₂ tương đương, độ nhạy chéo với khí khử khác",
                "Sai số DHT11 ±2 °C → CO₂ lệch ≈ 4 % (đã bù offset)",
                "Cảm biến bán dẫn cần thời gian sấy ban đầu để đạt độ ổn định tối đa"
            ]
            for b in bullets:
                p = sh.text_frame.add_paragraph()
                p.text = b
        elif "Sơ đồ đã vẽ khớp" in full_txt:
            for p in sh.text_frame.paragraphs:
                p.text = "Sơ đồ đã vẽ khớp 100% với code INO · phản xạ tức thì theo hơi thở · giám sát trực quan trên Serial Plotter"
print("Updated Slide 18 text")

# ==============================================================================
# SLIDE 19: KỊCH BẢN KIỂM THỬ
# ==============================================================================
slide19 = prs.slides[18]
for sh in slide19.shapes:
    if sh.has_table:
        table = sh.table
        for row in table.rows:
            if "TC-10" in row.cells[0].text:
                row.cells[1].text = "Chuỗi ngưỡng (650/950/1300), trễ 25 ppm, soft-ramp +14/−6"
print("Updated Slide 19 table")

# ==============================================================================
# SLIDE 20: NGÂN SÁCH SAI SỐ
# ==============================================================================
slide20 = prs.slides[19]
for sh in slide20.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "700 / 1000 / 1400" in p.text:
                p.text = "U ≈ ±41 / ±58 / ±79 ppm tại 650 / 950 / 1300 ppm (≈ ±6 %) · đóng góp lớn nhất: DHT11"
print("Updated Slide 20 text")

# ==============================================================================
# SLIDE 21: MỐC HIỆU CHUẨN VÀ TIÊU CHÍ SAI SỐ
# ==============================================================================
slide21 = prs.slides[20]
for sh in slide21.shapes:
    if sh.has_text_frame:
        for p in sh.text_frame.paragraphs:
            if "code đang dùng a = 415" in p.text:
                p.text = "Mốc hiệu chuẩn: 6 lần ngoài trời sáng sớm, đã cập nhật chuẩn xác mốc a = 400.0 ppm trong code INO. Thuật toán kEnvAdapt đo 2 s ban đầu giúp tự động neo mốc phòng về chuẩn sạch 51,30 kΩ."
print("Updated Slide 21 text")

# ==============================================================================
# SLIDE 25: KẾT QUẢ ĐẠT ĐƯỢC
# ==============================================================================
slide25 = prs.slides[24]
for sh in slide25.shapes:
    if sh.has_text_frame and "Xây dựng hệ thống đo CO₂" in " ".join([p.text for p in sh.text_frame.paragraphs]):
        sh.text_frame.clear()
        bullets = [
            "Xây dựng hoàn chỉnh hệ thống giám sát CO₂, nhiệt – ẩm trên Arduino Uno R3.",
            "Hiệu chuẩn 6 lần ngoài trời đạt R0 = 61,50 kΩ, mốc chuẩn 400.0 ppm, bù nhiệt ẩm Hanwei.",
            "Thuật toán Oversampling 20 mẫu ADC, chu kỳ 100 ms: triệt tiêu hiện tượng đơ cảm biến, số nhảy tức thì và 6 đường tín hiệu dao động liên tục trên Serial Plotter.",
            "Điều khiển quạt thông gió 4 cấp PWM (0 / 90–145 / 150–210 / 255) với trễ Schmitt 25 ppm và Soft-ramp (+14/−6).",
            "Sơ đồ nguyên lý và sơ đồ lắp dây khớp 100% với mã nguồn Arduino INO."
        ]
        p0 = sh.text_frame.paragraphs[0]
        p0.text = bullets[0]
        for b in bullets[1:]:
            p = sh.text_frame.add_paragraph()
            p.text = b
print("Updated Slide 25 text")

# ==============================================================================
# SAVE UPDATED PRESENTATION
# ==============================================================================
output_paths = [
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/OneDrive/Máy tính/Thuyet_trinh_He_thong_giam_sat_CO2_Moi.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
]

for out_p in output_paths:
    try:
        prs.save(out_p)
        print(f"Successfully saved: {out_p}")
    except Exception as e:
        print(f"Failed to save {out_p}: {e}")

