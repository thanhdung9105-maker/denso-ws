import pptx
from pptx import Presentation
from pptx.util import Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE

# Read working presentation
pptx_path = 'c:/Users/thanh/denso_ws/temp_work.pptx'
prs = Presentation(pptx_path)

# Slide 8 is at index 7. We want to insert the new slide at index 8 (Slide 9)
s8 = prs.slides[7]
new_slide = prs.slides.add_slide(s8.slide_layout)

# Move new slide to index 8
sldId = prs.slides._sldIdLst[-1]
prs.slides._sldIdLst.insert(8, sldId)

# 1. Decorative blue bars on left edge
rect16 = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, 244475, 840828)
rect16.fill.solid()
rect16.fill.fore_color.rgb = RGBColor(0, 112, 192)
rect16.line.fill.background()

rect10 = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 430886, 244475, 861885)
rect10.fill.solid()
rect10.fill.fore_color.rgb = RGBColor(0, 112, 192)
rect10.line.fill.background()

# 2. Slide Number Placeholder
slide_num_box = new_slide.shapes.add_textbox(11655911, 6444466, 536089, 413534)
tf_num = slide_num_box.text_frame
tf_num.word_wrap = True
p_num = tf_num.paragraphs[0]
p_num.text = "9"
p_num.font.name = "Calibri"
p_num.font.size = Pt(14)
p_num.font.color.rgb = RGBColor(120, 144, 156)

# 3. Slide Title
title_box = new_slide.shapes.add_textbox(402336, 260000, 11521440, 750000)
tf_title = title_box.text_frame
tf_title.word_wrap = True
p_title = tf_title.paragraphs[0]
p_title.text = "CÁC PHƯƠNG PHÁP LỌC NHIỄU TOÀN DIỆN CỦA HỆ THỐNG"
p_title.font.name = "Calibri"
p_title.font.size = Pt(26)
p_title.font.bold = True
p_title.font.color.rgb = RGBColor(0, 112, 192)

# Helper function to add vertical column cards
def add_col_card(slide, left, top, width, height, title, subtitle, items, bg_color, border_color, title_color):
    card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    card.fill.solid()
    card.fill.fore_color.rgb = bg_color
    card.line.color.rgb = border_color
    card.line.width = Pt(1.4)
    
    tf = card.text_frame
    tf.word_wrap = True
    tf.margin_left = Pt(10)
    tf.margin_right = Pt(10)
    tf.margin_top = Pt(10)
    tf.margin_bottom = Pt(10)
    
    p0 = tf.paragraphs[0]
    p0.text = title
    p0.font.name = "Calibri"
    p0.font.size = Pt(13.5)
    p0.font.bold = True
    p0.font.color.rgb = title_color
    p0.space_after = Pt(2)
    
    p_sub = tf.add_paragraph()
    p_sub.text = subtitle
    p_sub.font.name = "Calibri"
    p_sub.font.size = Pt(10.5)
    p_sub.font.italic = True
    p_sub.font.color.rgb = RGBColor(84, 110, 122)
    p_sub.space_after = Pt(8)
    
    for bold_txt, norm_txt in items:
        p = tf.add_paragraph()
        p.space_after = Pt(6)
        
        r1 = p.add_run()
        r1.text = "• " + bold_txt
        r1.font.name = "Calibri"
        r1.font.size = Pt(10.5)
        r1.font.bold = True
        r1.font.color.rgb = title_color
        
        r2 = p.add_run()
        r2.text = norm_txt
        r2.font.name = "Calibri"
        r2.font.size = Pt(10.2)
        r2.font.bold = False
        r2.font.color.rgb = RGBColor(38, 50, 56)

# Column 1: Lọc nhiễu Analog (MQ-135 - A0)
c1_items = [
    ("Oversampling 20 mẫu ADC:", " Đọc 20 mẫu liên tục cách nhau 400 µs (tổng 8 ms) rồi lấy trung bình. Triệt tiêu nhiễu trắng, nhiễu 50 Hz, tăng độ phân giải lên ~12-bit và khử đơ số."),
    ("Đo tỷ lệ (Ratiometric):", " Cầu phân áp RL/(Rs + RL) và ADC dùng chung Vc → Vc tự triệt tiêu trong công thức tính Rs, không phụ thuộc nguồn USB trồi sụt."),
    ("Bù nhiệt ẩm Hanwei:", " Dùng đa thức Hanwei khử độ lệch trôi phi tuyến của màng oxit kim loại khi nhiệt độ và độ ẩm phòng thay đổi.")
]
add_col_card(new_slide, 650000, 1120000, 3500000, 4980000,
             "1. Cảm biến MQ-135", "Tín hiệu Analog (Chân A0)", c1_items,
             bg_color=RGBColor(240, 244, 248), border_color=RGBColor(144, 202, 249), title_color=RGBColor(13, 71, 161))

# Column 2: Lọc nhiễu Digital (DHT11 - D2)
c2_items = [
    ("Lọc dữ liệu dị biệt (Sanity Check):", " Thuật toán kiểm tra điều kiện vật lý (!isnan và dải 0–70 °C, 5–100%). Lập tức loại bỏ khung truyền lỗi bit, nhiễu xung điện."),
    ("Khống chế chu kỳ 2 giây (Rate Limiting):", " Giới hạn tần số đọc đúng 2 s/lần. Tránh cảm biến bị tự nóng lên (self-heating) làm sai lệch nhiệt độ."),
    ("Bộ đếm lỗi & Giữ trạng thái (Watchdog):", " Khi mất gói tin tạm thời, giữ nguyên giá trị hợp lệ gần nhất, chỉ cảnh báo lỗi khi mất tín hiệu liên tiếp ≥ 4 giây.")
]
add_col_card(new_slide, 4350000, 1120000, 3500000, 4980000,
             "2. Cảm biến DHT11", "Tín hiệu Digital (Chân D2)", c2_items,
             bg_color=RGBColor(232, 245, 233), border_color=RGBColor(165, 214, 167), title_color=RGBColor(27, 94, 32))

# Column 3: Lọc nhiễu Chấp hành (Quạt PWM - D9)
c3_items = [
    ("Vùng trễ Schmitt Trigger 25 ppm:", " Tạo khoảng trễ 25 ppm tại các biên 650, 950, 1300 ppm. Triệt tiêu hoàn toàn hiện tượng quạt bật/tắt liên tục (hunting/chatter) khi số liệu mấp mé."),
    ("Khởi động mềm Soft-ramp (+14/−6):", " Khống chế độ dốc biến thiên PWM mỗi 100 ms. Triệt tiêu dòng xung kích (inrush current), chống sụt áp nguồn USB của Arduino."),
    ("Bảo vệ động cơ quạt:", " Quạt tăng tốc êm ái, kéo dài tuổi thọ động cơ DC và không phát ra tiếng ồn giật cục.")
]
add_col_card(new_slide, 8050000, 1120000, 3500000, 4980000,
             "3. Khâu quạt PWM", "Tín hiệu Chấp hành (Chân D9)", c3_items,
             bg_color=RGBColor(255, 248, 225), border_color=RGBColor(255, 213, 79), title_color=RGBColor(191, 54, 12))

# Bottom banner
banner = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 650000, 6240000, 10900000, 420000)
banner.fill.solid()
banner.fill.fore_color.rgb = RGBColor(13, 71, 161)
banner.line.fill.background()
tf_b = banner.text_frame
tf_b.word_wrap = True
p_b = tf_b.paragraphs[0]
p_b.text = "★ ĐÁNH GIÁ: Kiến trúc lọc 3 tầng bảo đảm hệ thống đo nhạy tức thì theo hơi thở, dữ liệu ổn định và phần cứng an toàn tuyệt đối."
p_b.font.name = "Calibri"
p_b.font.size = Pt(11)
p_b.font.bold = True
p_b.font.color.rgb = RGBColor(255, 255, 255)
p_b.alignment = PP_ALIGN.CENTER

# Update all slide numbers from 1 to 29
for idx, s in enumerate(prs.slides):
    s_actual_num = idx + 1
    for sh in s.shapes:
        if sh.name == "Slide Number Placeholder 1" and sh.has_text_frame:
            sh.text_frame.text = str(s_actual_num)
            for p in sh.text_frame.paragraphs:
                p.font.name = "Calibri"
                p.font.size = Pt(14)
                p.font.color.rgb = RGBColor(120, 144, 156)

# Save to Desktop targets
targets = [
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_LocNhieu.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_LocNhieu.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_LocNhieu.pptx'
]

for t in targets:
    try:
        prs.save(t)
        print(f"[OK] Saved {t}")
    except Exception as e:
        print(f"[ERR] Failed {t}: {e}")

# Also try updating Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx if unlocked
try:
    prs.save('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
    print("[OK] Saved Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx")
except Exception as e:
    print(f"[INFO] Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx is locked: {e}")

print(f"Done! Total slides now: {len(prs.slides)}")
