import pptx
from pptx import Presentation
from pptx.util import Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE

pptx_path = 'c:/Users/thanh/denso_ws/temp_work.pptx'
prs = Presentation(pptx_path)

# Slide 24 is at index 23
s24 = prs.slides[23]

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
    p0.font.size = Pt(13)
    p0.font.bold = True
    p0.font.color.rgb = title_color
    p0.space_after = Pt(6)
    
    for bold_txt, norm_txt in items:
        p = tf.add_paragraph()
        p.space_after = Pt(4)
        
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

# Clear old content cards on s24
shapes_to_remove = []
for sh in s24.shapes:
    if sh.name not in ["Slide Number Placeholder 1", "Rectangle 16", "Rectangle 10"]:
        if sh.has_text_frame and "TÍNH TOÁN NỒNG ĐỘ" in sh.text_frame.text:
            sh.text_frame.text = "TÍNH TOÁN NỒNG ĐỘ CO₂ KHI THỔI 300 ML HƠI THỞ VÀO HỘP KÍN (200×120×80 mm)"
            sh.text_frame.paragraphs[0].font.name = "Calibri"
            sh.text_frame.paragraphs[0].font.size = Pt(25)
            sh.text_frame.paragraphs[0].font.bold = True
            sh.text_frame.paragraphs[0].font.color.rgb = RGBColor(0, 112, 192)
        else:
            shapes_to_remove.append(sh)

for sh in shapes_to_remove:
    sp = sh._element
    sp.getparent().remove(sp)

# 1. Card 1: Thông số 300 ml hơi thở
c1_items = [
    ("Thể tích lòng hộp:", " 20 × 12 × 8 cm = 1.920 cm³ = 1,92 L (trừ linh kiện còn V_khí ≈ 1,87 L = 1.870 ml)."),
    ("Lượng khí thở (300 ml):", " 1 hơi thở người V_thở = 300 ml (chứa 4,0% – 5,0% CO₂, trung bình 4,5%)."),
    ("Lượng CO₂ nguyên chất từ 300 ml:", " V_CO2_thở = 300 ml × 4,5% = 13,50 ml CO₂ nguyên chất (≈ 24,3 mg)."),
    ("Khí cũ bị đẩy ra ngoài:", " 300 ml khí thở đẩy 300 ml khí cũ ra ngoài, lượng CO₂ nền 400 ppm còn lại trong 1.570 ml là ≈ 0,63 ml.")
]
add_card(s24, 700000, 1100000, 5100000, 2450000,
         "1. Thông số ban đầu & Lượng hơi thở 300 ml đưa vào", c1_items,
         bg_color=RGBColor(240, 244, 248), border_color=RGBColor(144, 202, 249), title_color=RGBColor(13, 71, 161))

# 2. Card 2: Công thức tính nồng độ đạt được
c2_items = [
    ("Tổng lượng CO₂ trong hộp:", " V_tổng = 0,63 ml (nền cũ) + 13,50 ml (hơi thở) = 14,13 ml CO₂."),
    ("Nồng độ hòa trộn lý thuyết:", " C = 14,13 ml / 1.870 ml ≈ 0,00756 (tức 0,756% thể tích khí)."),
    ("Quy đổi sang đơn vị ppm:", " C = 0,00756 × 10⁶ = 7.560 ppm (Dải thực tế theo cơ địa: 6.500 – 8.000 ppm)."),
    ("Hiển thị thực tế trên Arduino:", " Vượt xa ngưỡng nguy hiểm (≥ 1300 ppm), hệ thống bắt được đỉnh tăng vọt tức thì.")
]
add_card(s24, 700000, 3700000, 5100000, 2450000,
         "2. Công thức xác định nồng độ CO₂ cực đại", c2_items,
         bg_color=RGBColor(255, 248, 225), border_color=RGBColor(255, 213, 79), title_color=RGBColor(191, 54, 12))

# 3. Card 3: So sánh đối chứng thực nghiệm
c3_items = [
    ("Thổi tự do ngoài trời (cách 5–10 cm):", " Luồng khí phân tán nhanh với khí phòng → Nồng độ cục bộ bắt được: 1.800 – 2.000 ppm (Đúng thực nghiệm bạn vừa đo!)."),
    ("Thổi 300 ml vào hộp kín 1,87 L:", " Nồng độ tích tụ đậm đặc đạt 6.500 – 8.000 ppm (gấp ~4 lần ngoài trời) do khí bị giam kín.")
]
add_card(s24, 6100000, 1100000, 5400000, 2450000,
         "3. So sánh: Thổi tự do ngoài trời vs Thổi 300 ml vào hộp", c3_items,
         bg_color=RGBColor(232, 245, 233), border_color=RGBColor(165, 214, 167), title_color=RGBColor(27, 94, 32))

# 4. Card 4: Đáp ứng điều khiển quạt
c4_items = [
    ("Kích hoạt báo động NGUY:", " LCD lập tức báo [NGUY], nhận diện không khí vượt ngưỡng an toàn."),
    ("Quạt tự động 100% công suất (255 PWM):", " Soft-ramp tăng tốc nhanh lên 5V tối đa để hút khí ô nhiễm ra ngoài."),
    ("Quá trình suy giảm khi thông gió:", " Mở lỗ khí vào, quạt thổi liên tục → Nồng độ giảm theo hàm mũ C(t) = C_nền + (C₀ − C_nền)·e^(−t/τ) về lại ngưỡng an toàn.")
]
add_card(s24, 6100000, 3700000, 5400000, 2450000,
         "4. Phản ứng điều khiển tự động của hệ thống", c4_items,
         bg_color=RGBColor(237, 231, 246), border_color=RGBColor(179, 157, 219), title_color=RGBColor(69, 39, 160))

# Bottom banner
banner = s24.shapes.add_shape(MSO_SHAPE.RECTANGLE, 700000, 6260000, 10800000, 420000)
banner.fill.solid()
banner.fill.fore_color.rgb = RGBColor(13, 71, 161)
banner.line.fill.background()
tf_b = banner.text_frame
tf_b.word_wrap = True
p_b = tf_b.paragraphs[0]
p_b.text = "★ KẾT LUẬN: 300 ml hơi thở đưa 13,5 ml CO₂ vào hộp 1,87 L khiến nồng độ vọt lên 6.500 – 8.000 ppm, hệ thống kích hoạt quạt Full 100% (255 PWM)."
p_b.font.name = "Calibri"
p_b.font.size = Pt(11)
p_b.font.bold = True
p_b.font.color.rgb = RGBColor(255, 255, 255)
p_b.alignment = PP_ALIGN.CENTER

# Save to Desktop targets
targets = [
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_300ml.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_300ml.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_300ml.pptx'
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
