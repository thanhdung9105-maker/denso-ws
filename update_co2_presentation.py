import os
import pptx
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN

TEMPLATE_PATH = r'c:\Users\thanh\denso_ws\template_co2.pptx'
OUTPUT_DESKTOP_PATH = r'C:\Users\thanh\Desktop\Bao_Cao_He_Thong_Do_CO2_Chuyen_Sau.pptx'

prs = pptx.Presentation(TEMPLATE_PATH)
print(f"Loaded template with {len(prs.slides)} slides.")

def format_paragraph(p, text, font_name="Times New Roman", font_size_pt=13, bold=False, italic=False, color_rgb=None, align=None):
    p.text = text
    p.font.name = font_name
    p.font.size = Pt(font_size_pt)
    p.font.bold = bold
    p.font.italic = italic
    if color_rgb:
        p.font.color.rgb = color_rgb
    if align is not None:
        p.alignment = align

# -----------------------------------------------------------------------------
# CẬP NHẬT SLIDE 6: CHI TIẾT ĐỀ TÀI & 4 TRỤ CỘT CÔNG NGHỆ
# -----------------------------------------------------------------------------
slide6 = prs.slides[5] # index 5
for shape in slide6.shapes:
    if shape.has_text_frame and "Hiệu chuẩn 400.0 ppm" in shape.text_frame.text:
        tf = shape.text_frame
        tf.clear()
        p1 = tf.paragraphs[0]
        format_paragraph(p1, "• Đo lường & Xử lý:", font_size_pt=13, bold=True, color_rgb=RGBColor(30, 64, 175))
        p2 = tf.add_paragraph()
        format_paragraph(p2, "Xây dựng mạch đo CO₂ (MQ-135), nhiệt - ẩm (DHT11), hiển thị LCD 1602 I²C và xuất 6 kênh tín hiệu dao động liên tục trên Arduino Serial Plotter.", font_size_pt=13)
        p3 = tf.add_paragraph()
        format_paragraph(p3, "• Hiệu chuẩn & Bù trôi:", font_size_pt=13, bold=True, color_rgb=RGBColor(5, 150, 105))
        p4 = tf.add_paragraph()
        format_paragraph(p4, "Thực nghiệm 6 lần ngoài trời mốc chuẩn 400.0 ppm (R0 = 61.50 kΩ), bù nhiệt ẩm Hanwei và neo thích ứng phòng kEnvAdapt.", font_size_pt=13)
        p5 = tf.add_paragraph()
        format_paragraph(p5, "• Tín hiệu & Chấp hành:", font_size_pt=13, bold=True, color_rgb=RGBColor(217, 119, 6))
        p6 = tf.add_paragraph()
        format_paragraph(p6, "Oversampling 20 mẫu ADC (12-bit) chống đơ số, điều tốc quạt PWM 4 cấp với Soft-ramp (+14/-6) và trễ Schmitt 25 ppm.", font_size_pt=13)
        p7 = tf.add_paragraph()
        format_paragraph(p7, "• Đánh giá giới hạn vật lý:", font_size_pt=13, bold=True, color_rgb=RGBColor(220, 38, 38))
        p8 = tf.add_paragraph()
        format_paragraph(p8, "Phân tích hiện tượng bão hòa bề mặt màng SnO₂ ở dải nồng độ cao và sai số khi tiếp xúc hơi thở đậm đặc.", font_size_pt=13)

# -----------------------------------------------------------------------------
# CẬP NHẬT SLIDE 14: CÁC PHƯƠNG PHÁP LỌC NHIỄU TOÀN DIỆN (3 TẦNG LỌC)
# -----------------------------------------------------------------------------
slide14 = prs.slides[13] # index 13
for shape in slide14.shapes:
    if shape.has_text_frame and "CÁC PHƯƠNG PHÁP LỌC NHIỄU" in shape.text_frame.text:
        continue # giữ tiêu đề
    if shape.has_text_frame and ("Oversampling 20 mẫu" in shape.text_frame.text or "1. Cảm biến MQ-135" in shape.text_frame.text):
        tf = shape.text_frame
        tf.clear()
        
        items_s14 = [
            ("1. Cảm biến MQ-135 (Tín hiệu Analog chân A0)", True, RGBColor(30, 64, 175)),
            ("• Kỹ thuật đo tỷ lệ (Ratiometric): Cầu phân áp RL/(Rs + RL) và ADC dùng chung Vc → Vc tự triệt tiêu trong công thức tính Rs, loại bỏ hoàn toàn ảnh hưởng khi nguồn USB máy tính bị sụt áp.", False, None),
            ("• Oversampling 20 mẫu ADC: Đọc 20 mẫu liên tục cách nhau 400 µs (tổng 8 ms nằm trong chu kỳ bán sóng 50 Hz). Nâng độ phân giải hiệu dụng lên ~12-bit, khử nhiễu trắng và triệt tiêu 100% hiện tượng 'đơ số'.", False, None),
            ("• Bù nhiệt ẩm phi tuyến Hanwei: Cập nhật hàm đa thức bậc 2 liên tục từ DHT11 để bù trừ trôi nhiệt ẩm của màng oxit kim loại.", False, None),
            ("2. Cảm biến DHT11 (Tín hiệu Digital chân D2)", True, RGBColor(5, 150, 105)),
            ("• Khống chế chu kỳ đọc 2 giây: Đọc non-blocking bằng millis() tránh nghẽn CPU và tránh cảm biến tự nóng sai số.", False, None),
            ("• Bộ lọc dữ liệu dị biệt & Watchdog: Bỏ qua xung lỗi (!isnan, dải 0-70°C, 5-100%), nếu mất gói tin thì giữ giá trị hợp lệ gần nhất.", False, None),
            ("3. Khâu điều khiển quạt PWM (Tín hiệu Chấp hành chân D9)", True, RGBColor(217, 119, 6)),
            ("• Vùng trễ Schmitt Trigger 25 ppm: Chống dao động đóng ngắt quạt liên tục sát ngưỡng 650, 950 và 1300 ppm.", False, None),
            ("• Khởi động mềm Soft-ramp (+14 / -6): Khống chế độ dốc PWM mỗi 100 ms, triệt tiêu dòng xung kích động cơ và chống sụt áp hệ thống.", False, None)
        ]
        
        p0 = tf.paragraphs[0]
        format_paragraph(p0, items_s14[0][0], font_size_pt=13, bold=True, color_rgb=items_s14[0][2])
        for txt, is_b, col in items_s14[1:]:
            p = tf.add_paragraph()
            format_paragraph(p, txt, font_size_pt=12.5 if not is_b else 13, bold=is_b, color_rgb=col)

# -----------------------------------------------------------------------------
# CẬP NHẬT SLIDE 20: ĐÁNH GIÁ HỆ THỐNG & GIỚI HẠN VẬT LÝ DẢI NỒNG ĐỘ CAO
# -----------------------------------------------------------------------------
slide20 = prs.slides[19] # index 19
for shape in slide20.shapes:
    if shape.has_text_frame:
        if "Ưu điểm" in shape.text_frame.text:
            tf = shape.text_frame
            tf.clear()
            p0 = tf.paragraphs[0]
            format_paragraph(p0, "ƯU ĐIỂM VÀ ĐIỂM MẠNH HỆ THỐNG", font_size_pt=14, bold=True, color_rgb=RGBColor(5, 150, 105))
            
            pros = [
                "• Giám sát thời gian thực nhạy bén: Nhận biết chính xác các dải nồng độ CO₂, nhiệt độ, độ ẩm phòng dân dụng (400 - 1500 ppm).",
                "• Thuật toán lọc tín hiệu ưu việt: Oversampling 20 mẫu giúp số liệu hiển thị liên tục, khử đơ cảm biến, 6 đường đồ thị Serial Plotter uốn lượn sống động.",
                "• Điều tốc quạt 4 cấp PWM mượt mà: Soft-ramp và trễ Schmitt 25 ppm giúp quạt vận hành êm ái, không sụt áp nguồn và kéo dài tuổi thọ động cơ.",
                "• Chi phí chế tạo tối ưu: Giá thành thấp gấp 10 lần so với module NDIR công nghiệp, phù hợp ứng dụng rộng rãi trong nhà ở và lớp học."
            ]
            for pr in pros:
                p = tf.add_paragraph()
                format_paragraph(p, pr, font_size_pt=12.5)

        elif "Giới hạn" in shape.text_frame.text:
            tf = shape.text_frame
            tf.clear()
            p0 = tf.paragraphs[0]
            format_paragraph(p0, "GIỚI HẠN THIẾT BỊ: VÌ SAO KHÔNG ĐO CHÍNH XÁC Ở NỒNG ĐỘ LỚN?", font_size_pt=13.5, bold=True, color_rgb=RGBColor(220, 38, 38))
            
            limits = [
                "• Bão hòa bề mặt hấp phụ (Langmuir Saturation): Số lượng tâm hấp phụ oxy hoạt tính trên màng SnO₂ là hữu hạn. Khi nồng độ CO₂ vượt 2000 - 3000 ppm, các vị trí liên kết đều bị chiếm đóng; Rs tiệm cận đáy nên điện áp ra gần như không đổi dù nồng độ CO₂ tăng gấp đôi.",
                "• Độ nhạy dốc suy giảm nghiêm trọng: Đạo hàm đặc tuyến d(PPM)/d(Rs) ~ -2.769*Rs^(-3.769). Ở nồng độ cao, đường cong gần như nằm ngang; một sai số nhỏ 1 bước ADC (4.89 mV) bị khuếch đại thành sai lệch hàng trăm ppm.",
                "• Phản ứng chéo (Cross-Sensitivity): Cảm biến nhạy mạnh với hơi ẩm bão hòa và hợp chất hữu cơ (VOCs) trong hơi thở người, gây méo mó giá trị định lượng tuyệt đối.",
                "• Định hướng công nghệ: MQ-135 đóng vai trò hoàn hảo như một 'cảm biến chỉ thị xu hướng và phân cấp cảnh báo'. Để đo chính xác dải cao (> 3000 ppm), cần nâng cấp cảm biến quang học NDIR (SenseAir S8)."
            ]
            for lm in limits:
                p = tf.add_paragraph()
                format_paragraph(p, lm, font_size_pt=12.5)

# -----------------------------------------------------------------------------
# CẬP NHẬT SLIDE 22: ĐÁNH GIÁ SAI SỐ VÀ ĐỘ CHÍNH XÁC
# -----------------------------------------------------------------------------
slide22 = prs.slides[21] # index 21
for shape in slide22.shapes:
    if shape.has_text_frame and "NHẬN XÉT VÀ KẾT QUẢ THỰC NGHIỆM" in shape.text_frame.text:
        tf = shape.text_frame
        tf.clear()
        p0 = tf.paragraphs[0]
        format_paragraph(p0, "NHẬN XÉT VÀ KẾT QUẢ THỰC NGHIỆM ĐÁNH GIÁ SAI SỐ:", font_size_pt=13.5, bold=True, color_rgb=RGBColor(30, 64, 175))
        
        eval_items = [
            "• Phản xạ thực tế với hơi thở người: Khi thổi hơi thở trực tiếp, nồng độ nhảy tức thì từ nền lên 1800 - 2000+ ppm, quạt lập tức kích hoạt 100% công suất (255 PWM) và đưa ra cảnh báo mức NGUY hiểm đúng yêu cầu bảo vệ.",
            "• Khử triệt để hiện tượng đơ cảm biến: Thuật toán Oversampling 20 mẫu ADC (chu kỳ 400 µs) giúp vi xử lý bám sát biến thiên micro-volt, triệt tiêu hoàn toàn hiện tượng đứng số liệu.",
            "• Độ không đảm bảo đo mở rộng U ≈ ±6.1% (k = 2, P = 95%): Tại điểm kiểm tra 1000 ppm trong dải phòng dân dụng (400 - 1300 ppm), hệ thống đáp ứng trọn vẹn tiêu chuẩn IAQ về thiết bị cảnh báo chất lượng không khí.",
            "• Khuyến nghị nồng độ lớn: Do đặc tính bão hòa màng SnO₂, các số liệu hiển thị trên 3000 ppm mang ý nghĩa chỉ thị ngưỡng cảnh báo nguy cấp, không dùng làm đại lượng đo lường chuẩn."
        ]
        for it in eval_items:
            p = tf.add_paragraph()
            format_paragraph(p, it, font_size_pt=13)

# -----------------------------------------------------------------------------
# CẬP NHẬT SLIDE 23: MỐC HIỆU CHUẨN VÀ TIÊU CHÍ SAI SỐ
# -----------------------------------------------------------------------------
slide23 = prs.slides[22] # index 22
for shape in slide23.shapes:
    if shape.has_text_frame and "Mốc hiệu chuẩn:" in shape.text_frame.text:
        tf = shape.text_frame
        tf.clear()
        p0 = tf.paragraphs[0]
        format_paragraph(p0, "QUY TRÌNH HIỆU CHUẨN VÀ BÙ TRỪ MỐC CHUẨN:", font_size_pt=13.5, bold=True, color_rgb=RGBColor(5, 150, 105))
        
        cal_items = [
            "• Hiệu chuẩn 6 lần ngoài trời: Đo thực nghiệm tại vườn cây thoáng đãng sáng sớm tiệm cận mốc chuẩn khí quyển 400.0 ppm, xác lập Rs_clean = 51.30 kΩ tại 31.4°C, 62.4% RH.",
            "• Tính toán R0_base = 61.50 kΩ: Sử dụng hệ số bù nhiệt ẩm Hanwei cor_cal = 0.8341 để nạp cố định mốc R0 chuẩn xác vào mã nguồn.",
            "• Thuật toán tự thích ứng phòng (kEnvAdapt): Trong 2 giây đầu lúc khởi động, vi xử lý đo 20 mẫu phòng ban đầu để tính kEnvAdapt = Rs_clean / Rs_room, tự động neo số liệu phòng về mốc vườn chuẩn.",
            "• Độ lệch nền khí quyển toàn cầu: Nồng độ CO₂ nền toàn cầu thực tế (NOAA 2025) đạt ~425.6 ppm → mốc chuẩn hóa 400.0 ppm tạo sai lệch có kiểm soát khoảng -6% (an toàn cho phòng kín)."
        ]
        for ci in cal_items:
            p = tf.add_paragraph()
            format_paragraph(p, ci, font_size_pt=13)

# -----------------------------------------------------------------------------
# ĐỒNG BỘ TOÀN BỘ PHÔNG CHỮ SANG TIMES NEW ROMAN VÀ CỠ CHỮ CHUẨN
# -----------------------------------------------------------------------------
print("Normalizing all slides to Times New Roman and optimizing font sizes...")
for s_idx, slide in enumerate(prs.slides):
    for shape in slide.shapes:
        if shape.has_text_frame:
            for p in shape.text_frame.paragraphs:
                p.font.name = "Times New Roman"
                # Cập nhật các đoạn văn bản thân bài (không phải tiêu đề quá lớn hoặc quá bé) về quanh 13 pt
                if p.font.size is not None:
                    sz = p.font.size.pt
                    if 9.0 <= sz <= 14.0:
                        p.font.size = Pt(13)
                else:
                    p.font.size = Pt(13)
        elif shape.has_table:
            for row in shape.table.rows:
                for cell in row.cells:
                    for p in cell.text_frame.paragraphs:
                        p.font.name = "Times New Roman"
                        p.font.size = Pt(11) # Bảng biểu tối ưu 11 pt để không vỡ dòng

# Lưu file kết quả
prs.save(OUTPUT_DESKTOP_PATH)
print(f"SUCCESS: Successfully generated presentation at: {OUTPUT_DESKTOP_PATH}")
