import pptx
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

pptx_path = 'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx'
prs = Presentation(pptx_path)

# Slide 23 is at index 22
s23 = prs.slides[22]

# Add new slide using s23's layout
new_slide = prs.slides.add_slide(s23.slide_layout)

# Move new slide to index 23 (right after slide 23)
sldId = prs.slides._sldIdLst[-1]
prs.slides._sldIdLst.insert(23, sldId)

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
p_num.text = "24"
p_num.font.name = "Calibri"
p_num.font.size = Pt(14)
p_num.font.color.rgb = RGBColor(120, 144, 156)

# 3. Slide Title
title_box = new_slide.shapes.add_textbox(402336, 260000, 11521440, 750000)
tf_title = title_box.text_frame
tf_title.word_wrap = True
p_title = tf_title.paragraphs[0]
p_title.text = "TÍNH TOÁN NỒNG ĐỘ CO₂ KHI THỔI VÀO HỘP KÍN (200 × 120 × 80 mm)"
p_title.font.name = "Calibri"
p_title.font.size = Pt(26)
p_title.font.bold = True
p_title.font.color.rgb = RGBColor(0, 112, 192)

# Helper function to add styled cards
def add_card(slide, left, top, width, height, title, items, bg_color=RGBColor(245, 247, 250), border_color=RGBColor(207, 216, 220), title_color=RGBColor(13, 71, 161)):
    card = slide.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, left, top, width, height)
    card.fill.solid()
    card.fill.fore_color.rgb = bg_color
    card.line.color.rgb = border_color
    card.line.width = Pt(1.2)
    
    tf = card.text_frame
    tf.word_wrap = True
    tf.margin_left = Pt(12)
    tf.margin_right = Pt(12)
    tf.margin_top = Pt(10)
    tf.margin_bottom = Pt(10)
    
    p0 = tf.paragraphs[0]
    p0.text = title
    p0.font.name = "Calibri"
    p0.font.size = Pt(13.5)
    p0.font.bold = True
    p0.font.color.rgb = title_color
    p0.space_after = Pt(6)
    
    for bold_txt, norm_txt in items:
        p = tf.add_paragraph()
        p.space_after = Pt(4)
        
        r1 = p.add_run()
        r1.text = "• " + bold_txt
        r1.font.name = "Calibri"
        r1.font.size = Pt(10.8)
        r1.font.bold = True
        r1.font.color.rgb = title_color
        
        r2 = p.add_run()
        r2.text = norm_txt
        r2.font.name = "Calibri"
        r2.font.size = Pt(10.5)
        r2.font.bold = False
        r2.font.color.rgb = RGBColor(38, 50, 56)

# Card 1: Thông số đầu vào & Lượng khí thở (Top Left)
c1_items = [
    ("Thể tích lòng hộp:", " 20 × 12 × 8 cm = 1.920 cm³ = 1,92 L (trừ linh kiện còn V_khí ≈ 1,87 L = 1.870 ml)."),
    ("Lượng CO₂ nền ban đầu:", " Tại mốc chuẩn 400 ppm: V_nền = 1.870 × 0,0004 = 0,75 ml CO₂."),
    ("Khí thở 300 ml (chứa 4,5% CO₂):", " Lượng khí đưa vào: V_thở = 300 × 0,045 = 13,50 ml CO₂ nguyên chất.")
]
add_card(new_slide, 700000, 1100000, 5100000, 2400000,
         "1. Thông số ban đầu & Lượng khí thở đưa vào", c1_items,
         bg_color=RGBColor(240, 244, 248), border_color=RGBColor(144, 202, 249), title_color=RGBColor(13, 71, 161))

# Card 2: Công thức & Kết quả tính toán nồng độ (Bottom Left)
c2_items = [
    ("Tổng CO₂ sau khi thổi:", " V_tổng = V_nền + V_thở = 0,75 + 13,50 = 14,25 ml CO₂."),
    ("Nồng độ hòa trộn lý thuyết:", " C = 14,25 ml / 1.870 ml ≈ 0,00762 (0,762% thể tích)."),
    ("Nồng độ hiển thị theo ppm:", " C = 0,00762 × 10⁶ = 7.620 ppm (Dải thực tế: 6.500 – 8.000 ppm)."),
    ("Giới hạn trần Arduino INO:", " Code cài trần bảo vệ 9.999 ppm nên hệ thống hiển thị an toàn, không bị tràn số.")
]
add_card(new_slide, 700000, 3650000, 5100000, 2450000,
         "2. Công thức xác định nồng độ CO₂ trong hộp kín", c2_items,
         bg_color=RGBColor(255, 248, 225), border_color=RGBColor(255, 213, 79), title_color=RGBColor(191, 54, 12))

# Card 3: So sánh đối chứng Thực nghiệm (Top Right)
c3_items = [
    ("Thổi ngoài không gian tự do (cách 5–10 cm):", " Luồng khí hòa trộn với phòng → Cảm biến bắt được nồng độ cục bộ: 1.800 – 2.000 ppm (Đúng với kết quả thực nghiệm vừa đo!)."),
    ("Thổi vào hộp kín 1,87 L:", " Toàn bộ lượng CO₂ bị giữ kín trong thể tích nhỏ → Nồng độ tích tụ vượt trần: 6.500 – 8.000 ppm (Lớn gấp ~4 lần ngoài trời).")
]
add_card(new_slide, 6100000, 1100000, 5400000, 2400000,
         "3. So sánh: Thổi tự do ngoài trời vs Thổi trong hộp kín", c3_items,
         bg_color=RGBColor(232, 245, 233), border_color=RGBColor(165, 214, 167), title_color=RGBColor(27, 94, 32))

# Card 4: Đáp ứng phần cứng & Điều khiển quạt (Bottom Right)
c4_items = [
    ("Cảnh báo mức NGUY:", " LCD 1602 hiển thị [NGUY], còi/cảnh báo sẵn sàng kích hoạt."),
    ("Quạt tự động 100% PWM 255:", " Tăng tốc Soft-ramp (+14 mỗi 100 ms) lên cực đại 5V để xả khí."),
    ("Quy luật giảm nồng độ:", " Mở lỗ gió, quạt hút khí tươi vào → CO₂ giảm dần theo hàm mũ C(t) = C_nền + (C₀ − C_nền)·e^(−t/τ) với τ ≈ 30 – 60 s.")
]
add_card(new_slide, 6100000, 3650000, 5400000, 2450000,
         "4. Đáp ứng điều khiển tự động của hệ thống", c4_items,
         bg_color=RGBColor(237, 231, 246), border_color=RGBColor(179, 157, 219), title_color=RGBColor(69, 39, 160))

# Bottom banner
banner = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 700000, 6220000, 10800000, 420000)
banner.fill.solid()
banner.fill.fore_color.rgb = RGBColor(13, 71, 161)
banner.line.fill.background()
tf_b = banner.text_frame
tf_b.word_wrap = True
p_b = tf_b.paragraphs[0]
p_b.text = "★ KẾT LUẬN: Khẳng định tính đúng đắn giữa mô hình toán lý thuyết và phản xạ thực tế 1.800 – 2.000 ppm của hệ thống Arduino."
p_b.font.name = "Calibri"
p_b.font.size = Pt(11)
p_b.font.bold = True
p_b.font.color.rgb = RGBColor(255, 255, 255)
p_b.alignment = PP_ALIGN.CENTER

# Update all slide numbers from slide 24 to 28
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
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
]

for t in targets:
    try:
        prs.save(t)
        print(f"[OK] Saved: {t}")
    except Exception as e:
        print(f"[ERR] Failed {t}: {e}")

print(f"Done! Total slides now: {len(prs.slides)}")
