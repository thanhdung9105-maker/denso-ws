import os
import pptx
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN
from pptx.enum.shapes import MSO_SHAPE

# Khởi tạo presentation 16:9
prs = pptx.Presentation()
prs.slide_width = Inches(13.333)
prs.slide_height = Inches(7.5)
blank_layout = prs.slide_layouts[6]

# Bảng màu chuyên nghiệp
COLOR_BG_DARK = RGBColor(15, 23, 42)      # Deep Navy
COLOR_BG_LIGHT = RGBColor(248, 250, 252)  # Light Slate
COLOR_TEXT_MAIN = RGBColor(15, 23, 42)
COLOR_TEXT_MUTED = RGBColor(71, 85, 105)
COLOR_CARD_BG = RGBColor(255, 255, 255)
COLOR_CARD_BORDER = RGBColor(226, 232, 240)

# Màu 4 phương pháp
COLOR_ANNEAL = RGBColor(16, 185, 129)    # Xanh lá (Ủ)
COLOR_NORM = RGBColor(37, 99, 235)       # Xanh dương (Thường hóa)
COLOR_QUENCH = RGBColor(220, 38, 38)     # Đỏ (Tôi)
COLOR_TEMPER = RGBColor(217, 119, 6)     # Vàng cam (Ram)

ASSETS_DIR = r'c:\Users\thanh\denso_ws\heat_treatment_assets'
DESKTOP_PATH = r'C:\Users\thanh\Desktop\Bao_Cao_4_Phuong_Phap_Nhiet_Luyen_Thep.pptx'

def add_header(slide, title_text, subtitle_text, slide_num_str):
    # Banner trên
    header_box = slide.shapes.add_textbox(Inches(0.8), Inches(0.4), Inches(11.7), Inches(0.9))
    tf = header_box.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_top = tf.margin_right = tf.margin_bottom = 0
    
    p = tf.paragraphs[0]
    p.text = title_text.upper()
    p.font.name = 'Arial'
    p.font.size = Pt(20)
    p.font.bold = True
    p.font.color.rgb = COLOR_TEXT_MAIN
    
    p2 = tf.add_paragraph()
    p2.text = subtitle_text
    p2.font.name = 'Arial'
    p2.font.size = Pt(11)
    p2.font.color.rgb = COLOR_TEXT_MUTED

    # Số trang ở góc phải
    num_box = slide.shapes.add_textbox(Inches(11.5), Inches(0.4), Inches(1.0), Inches(0.5))
    tf_num = num_box.text_frame
    p_num = tf_num.paragraphs[0]
    p_num.alignment = PP_ALIGN.RIGHT
    p_num.text = slide_num_str
    p_num.font.name = 'Arial'
    p_num.font.size = Pt(13)
    p_num.font.bold = True
    p_num.font.color.rgb = COLOR_NORM

# ==============================================================================
# SLIDE 1: TRANG BÌA (TITLE SLIDE)
# ==============================================================================
s1 = prs.slides.add_slide(blank_layout)

# Nền tối sang trọng
bg_shape = s1.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, Inches(13.333), Inches(7.5))
bg_shape.fill.solid()
bg_shape.fill.fore_color.rgb = COLOR_BG_DARK
bg_shape.line.color.rgb = COLOR_BG_DARK

# Card tiêu đề
t_box = s1.shapes.add_textbox(Inches(1.2), Inches(1.5), Inches(10.9), Inches(3.5))
tf = t_box.text_frame
tf.word_wrap = True

p_sub0 = tf.paragraphs[0]
p_sub0.text = "BÁO CÁO CHUYÊN ĐỀ MÔN HỌC: VẬT LIỆU HỌC VÀ XỬ LÝ (CHƯƠNG 2 - 6)"
p_sub0.font.name = 'Arial'
p_sub0.font.size = Pt(13)
p_sub0.font.bold = True
p_sub0.font.color.rgb = RGBColor(56, 189, 248) # Cyan

p_title = tf.add_paragraph()
p_title.text = "4 PHƯƠNG PHÁP NHIỆT LUYỆN THÉP\nỦ – THƯỜNG HÓA – TÔI – RAM"
p_title.font.name = 'Arial'
p_title.font.size = Pt(32)
p_title.font.bold = True
p_title.font.color.rgb = RGBColor(255, 255, 255)

p_desc = tf.add_paragraph()
p_desc.text = "Cơ sở lý thuyết Giản đồ Fe–Fe₃C, Chuyển biến pha khi nguội & Ứng dụng chế tạo chi tiết cơ khí thực tế"
p_desc.font.name = 'Arial'
p_desc.font.size = Pt(14)
p_desc.font.color.rgb = RGBColor(203, 213, 225)

# Khối thẻ tóm tắt 4 phương pháp
methods = [
    ("Ủ (Annealing)", "Làm mềm, khử ứng suất, tăng dẻo", COLOR_ANNEAL),
    ("Thường Hóa (Normalizing)", "Đồng đều tổ chức, làm mịn hạt phôi", COLOR_NORM),
    ("Tôi (Quenching)", "Tạo Mactenxit, độ cứng cực đại HRC 60-65", COLOR_QUENCH),
    ("Ram (Tempering)", "Khử ứng suất sau tôi, điều chỉnh độ dai/đàn hồi", COLOR_TEMPER)
]

for idx, (m_title, m_desc, m_col) in enumerate(methods):
    cx = Inches(1.2 + idx * 2.8)
    cy = Inches(5.3)
    cw = Inches(2.6)
    ch = Inches(1.4)
    card = s1.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = RGBColor(30, 41, 59)
    card.line.color.rgb = m_col
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.15)
    cp1 = ctf.paragraphs[0]
    cp1.text = m_title
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(11)
    cp1.font.bold = True
    cp1.font.color.rgb = m_col
    
    cp2 = ctf.add_paragraph()
    cp2.text = m_desc
    cp2.font.name = 'Arial'
    cp2.font.size = Pt(9.5)
    cp2.font.color.rgb = RGBColor(226, 232, 240)

# ==============================================================================
# SLIDE 2: CƠ SỞ GIẢN ĐỒ PHA SẮT - CACBON (Fe - Fe3C)
# ==============================================================================
s2 = prs.slides.add_slide(blank_layout)
add_header(s2, "Cơ Sở Lý Thuyết: Giản Đồ Pha Sắt – Cacbon (Fe – Fe₃C)", 
           "Xác định nhiệt độ nung tới hạn cho 4 phương pháp nhiệt luyện thép (Chương 4 & 6)", "02 / 10")

# Card bên trái: Tóm tắt lý thuyết các đường tới hạn
card_l = s2.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.5), Inches(5.2), Inches(5.4))
card_l.fill.solid()
card_l.fill.fore_color.rgb = COLOR_CARD_BG
card_l.line.color.rgb = COLOR_CARD_BORDER

tf_l = card_l.text_frame
tf_l.word_wrap = True
tf_l.margin_left = tf_l.margin_right = Inches(0.2)
tf_l.margin_top = Inches(0.2)

p = tf_l.paragraphs[0]
p.text = "1. Các Đường Tới Hạn Cốt Lõi Trên Giản Đồ:"
p.font.name = 'Arial'
p.font.size = Pt(12)
p.font.bold = True
p.font.color.rgb = COLOR_NORM

items_l2 = [
    ("• Đường A₁ (727°C, đường P-S-K):", "Đường chuyển biến cùng tích Peclit (Ferrite + Fe₃C) ↔ Austenite (γ). Điểm cùng tích S = 0.77% C."),
    ("• Đường A₃ (đường G-S, 912°C → 727°C):", "Đường kết thúc hòa tan Ferrite vào Austenite khi nung thép trước cùng tích (< 0.77% C)."),
    ("• Đường Acm (đường S-E, 727°C → 1147°C):", "Đường hòa tan hoàn toàn Xementit II vào Austenite khi nung thép sau cùng tích (> 0.77% C)."),
    ("2. Vùng Nhiệt Độ Nung Của 4 Phương Pháp:", ""),
    ("• Thường hóa:", "Nung trên A₃ hoặc Acm + (30 ÷ 50°C), tạo 100% Austenite đồng nhất."),
    ("• Tôi thép:", "Thép trước cùng tích nung A₃ + (30 ÷ 50°C); Thép sau cùng tích nung A₁ + (30 ÷ 50°C) để giữ hạt Fe₃C cứng."),
    ("• Ủ hoàn toàn:", "Nung A₃ + (20 ÷ 40°C), làm nguội rất chậm cùng lò."),
    ("• Ram thép:", "Nung nóng lại thép đã tôi dưới A₁ (150°C ÷ 650°C).")
]

for title_item, desc_item in items_l2:
    pi = tf_l.add_paragraph()
    pi.text = title_item + " " + desc_item
    pi.font.name = 'Arial'
    pi.font.size = Pt(9.5)
    if title_item.startswith("•"):
        pi.font.color.rgb = COLOR_TEXT_MAIN
    else:
        pi.font.bold = True
        pi.font.color.rgb = COLOR_NORM

# Ảnh bên phải: Giản đồ Fe-C có đánh dấu vùng nhiệt luyện
img_path_fec = os.path.join(ASSETS_DIR, 'fe_c_heat_treatment.png')
if os.path.exists(img_path_fec):
    s2.shapes.add_picture(img_path_fec, Inches(6.2), Inches(1.5), Inches(6.3), Inches(5.4))

# ==============================================================================
# SLIDE 3: CHUYỂN BIẾN PHA VÀ TỐC ĐỘ LÀM NGUỘI
# ==============================================================================
s3 = prs.slides.add_slide(blank_layout)
add_header(s3, "Chuyển Biến Pha & Tốc Độ Làm Nguội (Chương 5 & 6)", 
           "Mối liên hệ giữa tốc độ làm nguội, khuếch tán nguyên tử và tổ chức tế vi cuối cùng", "03 / 10")

# Ảnh bên phải: Đường cong làm nguội
img_path_cooling = os.path.join(ASSETS_DIR, 'heat_treatment_cooling_curves.png')
if os.path.exists(img_path_cooling):
    s3.shapes.add_picture(img_path_cooling, Inches(5.9), Inches(1.5), Inches(6.6), Inches(5.4))

# Khối giải thích bên trái
card_s3 = s3.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(1.5), Inches(4.9), Inches(5.4))
card_s3.fill.solid()
card_s3.fill.fore_color.rgb = COLOR_CARD_BG
card_s3.line.color.rgb = COLOR_CARD_BORDER

tf3 = card_s3.text_frame
tf3.word_wrap = True
tf3.margin_left = tf3.margin_right = Inches(0.2)
tf3.margin_top = Inches(0.2)

p3 = tf3.paragraphs[0]
p3.text = "Cơ Chế Chuyển Biến Từ Pha Mẹ Austenite (γ):"
p3.font.name = 'Arial'
p3.font.size = Pt(12)
p3.font.bold = True
p3.font.color.rgb = COLOR_NORM

s3_points = [
    ("1. Chuyển biến có khuếch tán (Nguội chậm):", "Khi làm nguội chậm trong lò (Ủ) hoặc không khí (Thường hóa), các nguyên tử Carbon có đủ thời gian khuếch tán để tạo Peclit (tấm xen kẽ Ferrite + Xementit). Hạt càng mịn khi nguội càng nhanh."),
    ("2. Chuyển biến không khuếch tán (Tôi thép):", "Khi nguội nhanh vượt tốc độ tới hạn (V > Vth) trong nước/dầu, Carbon bị bẫy lại trong mạng tinh thể sắt tạo mạng chính phương tâm khối (BCT) méo lệch mạng cực lớn → Pha MACTENXIT (M) có độ cứng cực cao HRC 60-65 nhưng rất giòn."),
    ("3. Quá trình Ram (Nung lại sau Tôi):", "Cung cấp nhiệt lượng để Carbon khuếch tán giải tỏa ứng suất dư, biến tính Mactenxit tôi thành Mactenxit ram, Troostit ram hoặc Sorbit ram tùy theo nhiệt độ.")
]

for title_pt, desc_pt in s3_points:
    p_t = tf3.add_paragraph()
    p_t.text = title_pt
    p_t.font.name = 'Arial'
    p_t.font.size = Pt(10)
    p_t.font.bold = True
    p_t.font.color.rgb = COLOR_TEXT_MAIN
    
    p_d = tf3.add_paragraph()
    p_d.text = desc_pt
    p_d.font.name = 'Arial'
    p_d.font.size = Pt(9.2)
    p_d.font.color.rgb = COLOR_TEXT_MUTED

# ==============================================================================
# SLIDE 4: PHƯƠNG PHÁP 1: Ủ THÉP (ANNEALING)
# ==============================================================================
s4 = prs.slides.add_slide(blank_layout)
add_header(s4, "Phương Pháp 1: Ủ Thép (Annealing)", 
           "Làm mềm thép, giảm độ cứng, tăng độ dẻo và triệt tiêu ứng suất dư bên trong chi tiết", "04 / 10")

# 3 cột thẻ
cards_data_s4 = [
    ("1. NGUYÊN LÝ & NHIỆT ĐỘ NUNG", COLOR_ANNEAL, [
        ("• Bản chất:", "Nung nóng thép lên nhiệt độ thích hợp, giữ nhiệt đủ lâu, sau đó làm nguội RẤT CHẬM CÙNG LÒ (tốc độ 20 ÷ 50°C/h)."),
        ("• Ủ hoàn toàn (thép trước cùng tích):", "Nung tới Ac₃ + (20 ÷ 40°C), chuyển biến thành 100% Austenite rồi làm nguội chậm tạo Ferrite + Peclit thô."),
        ("• Ủ cầu hóa (thép sau cùng tích):", "Nung dao động quanh Ac₁ (740 ÷ 770°C) để biến Xementit dạng tấm thành dạng hạt cầu tròn."),
        ("• Ủ khử ứng suất:", "Nung dưới Ac₁ (500 ÷ 650°C), không làm thay đổi pha, giải tỏa ứng suất dư do đúc, hàn, gia công biến dạng.")
    ]),
    ("2. TỔ CHỨC TẾ VI & CƠ TÍNH", COLOR_NORM, [
        ("• Tổ chức tế vi thu được:", "Trạng thái cân bằng theo giản đồ Fe–Fe₃C: Ferrite + Peclit dạng tấm thô hoặc Peclit hạt cầu."),
        ("• Cơ tính đặc trưng:", "Độ cứng thấp nhất (thường ≤ 160 ÷ 180 HBW), độ dẻo và độ dai cao nhất."),
        ("• Khử hoàn toàn biến cứng:", "Triệt tiêu hiện tượng hóa bền biến dạng do cán nguội, kéo nguội trước đó."),
        ("• Triệt tiêu ứng suất dư:", "Ngăn ngừa chi tiết bị cong vênh, biến dạng hay nứt vỡ trong các công đoạn gia công tiếp theo.")
    ]),
    ("3. ỨNG DỤNG & SẢN PHẨM CỤ THỂ", COLOR_QUENCH, [
        ("• Dây thép dập nguội bu lông, vít:", "Ủ cầu hóa dây thép cuộn (thép carbon trung bình) trước khi đưa vào máy dập nguội tạo đầu mũ bu lông để tránh bị nứt toác mép."),
        ("• Phôi đúc thân máy, vỏ hộp số:", "Ủ khử ứng suất thân gang đúc/thép đúc kích thước lớn để giữ ổn định kích thước khi gia công phay, doa lỗ ổ trục chính xác."),
        ("• Kết cấu hàn bồn áp lực, đường ống:", "Ủ khử ứng suất cục bộ vùng ảnh hưởng nhiệt (HAZ) quanh mối hàn để tránh nứt giòn dưới áp suất cao.")
    ])
]

for idx, (c_title, c_color, c_items) in enumerate(cards_data_s4):
    cx = Inches(0.8 + idx * 3.95)
    cy = Inches(1.5)
    cw = Inches(3.8)
    ch = Inches(5.4)
    
    card = s4.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = COLOR_CARD_BG
    card.line.color.rgb = c_color
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.18)
    ctf.margin_top = Inches(0.2)
    
    cp1 = ctf.paragraphs[0]
    cp1.text = c_title
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(11)
    cp1.font.bold = True
    cp1.font.color.rgb = c_color
    
    for item in c_items:
        if isinstance(item, (tuple, list)):
            txt = " ".join(str(x) for x in item).strip()
        else:
            txt = str(item)
        pi = ctf.add_paragraph()
        pi.text = txt
        pi.font.name = 'Arial'
        pi.font.size = Pt(9.2)
        pi.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 5: PHƯƠNG PHÁP 2: THƯỜNG HÓA THÉP (NORMALIZING)
# ==============================================================================
s5 = prs.slides.add_slide(blank_layout)
add_header(s5, "Phương Pháp 2: Thường Hóa Thép (Normalizing)", 
           "Làm mịn hạt tinh thể, đồng đều hóa tổ chức phôi và cải thiện tính gia công cắt gọt", "05 / 10")

cards_data_s5 = [
    ("1. NGUYÊN LÝ & NHIỆT ĐỘ NUNG", COLOR_NORM, [
        ("• Bản chất:", "Nung thép lên trạng thái hoàn toàn Austenite, giữ nhiệt, sau đó LÀM NGUỘI TRONG KHÔNG KHÍ TĨNH."),
        ("• Nhiệt độ nung:", "Nung tới Ac₃ hoặc Acm + (30 ÷ 50°C)."),
        ("• Thép trước cùng tích:", "Nung trên Ac₃ để xóa bỏ hoàn toàn tổ chức hạt Ferrite thô to của phôi đúc/rèn."),
        ("• Thép sau cùng tích:", "Nung trên Acm để hòa tan hoàn toàn lưới Xementit II giòn bao quanh ranh giới hạt Austenite, khi nguội trong không khí lưới này không kịp hình thành lại.")
    ]),
    ("2. TỔ CHỨC TẾ VI & CƠ TÍNH", COLOR_ANNEAL, [
        ("• Tổ chức tế vi thu được:", "Peclit phân tán mịn (hoặc Sorbit thường hóa) với kích thước hạt tinh thể nhỏ và đồng đều."),
        ("• Cơ tính so với Ủ:", "Do tốc độ nguội trong không khí nhanh hơn trong lò, tổ chức mịn hơn → độ bền (σb) và độ cứng cao hơn ủ một chút (170 ÷ 210 HBW)."),
        ("• Tính gia công cắt gọt tuyệt vời:", "Thép sau thường hóa không bị quá mềm dẻo như ủ (tránh hiện tượng bết dao, dính phoi, bề mặt gia công bị xước xơ).")
    ]),
    ("3. ỨNG DỤNG & SẢN PHẨM CỤ THỂ", COLOR_TEMPER, [
        ("• Phôi rèn trục xe tải, trục truyền động:", "Thường hóa phôi thép 40Cr, C45 sau rèn dập ở 860°C để làm mịn hạt, triệt tiêu thiên tích trước khi đưa vào tiện thô."),
        ("• Phôi bánh răng hộp số trước cắt răng:", "Thường hóa phôi rèn bánh răng để đạt độ cứng đồng đều 180-200 HBW, giúp dao phay lăn răng cắt ngọt, phoi thoát dễ dàng và tuổi thọ dao cao."),
        ("• Dầm thép chữ I, thép kết cấu cán nóng:", "Thường hóa loại bỏ các dải dị hướng cán, đảm bảo độ dai va đập đồng đều theo mọi phương.")
    ])
]

for idx, (c_title, c_color, c_items) in enumerate(cards_data_s5):
    cx = Inches(0.8 + idx * 3.95)
    cy = Inches(1.5)
    cw = Inches(3.8)
    ch = Inches(5.4)
    
    card = s5.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = COLOR_CARD_BG
    card.line.color.rgb = c_color
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.18)
    ctf.margin_top = Inches(0.2)
    
    cp1 = ctf.paragraphs[0]
    cp1.text = c_title
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(11)
    cp1.font.bold = True
    cp1.font.color.rgb = c_color
    
    for item in c_items:
        if isinstance(item, (tuple, list)):
            txt = " ".join(str(x) for x in item).strip()
        else:
            txt = str(item)
        pi = ctf.add_paragraph()
        pi.text = txt
        pi.font.name = 'Arial'
        pi.font.size = Pt(9.2)
        pi.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 6: PHƯƠNG PHÁP 3: TÔI THÉP (QUENCHING)
# ==============================================================================
s6 = prs.slides.add_slide(blank_layout)
add_header(s6, "Phương Pháp 3: Tôi Thép (Quenching)", 
           "Làm nguội nhanh tạo pha Mactenxit nhằm đạt độ cứng và khả năng chống mài mòn cực đại", "06 / 10")

cards_data_s6 = [
    ("1. CHỌN NHIỆT ĐỘ & MÔI TRƯỜNG TÔI", COLOR_QUENCH, [
        ("• Thép trước cùng tích (C45, 40Cr):", "Nung Ac₃ + (30 ÷ 50°C) → Tôi hoàn toàn để chuyển 100% thành Austenite, tránh sót hạt Ferrite mềm gây đốm mềm trên chi tiết."),
        ("• Thép sau cùng tích (thép dụng cụ):", "Nung Ac₁ + (30 ÷ 50°C) → Tôi không hoàn toàn để giữ lại các hạt Xementit II siêu cứng phân tán trên nền Mactenxit, tránh thô hạt và nứt."),
        ("• Môi trường tôi:", "Nước (thép carbon đơn giản), Dầu khoáng (thép hợp kim để tránh nứt nẻ cong vênh), Dung dịch muối, Polyme."),
        ("• 4 cách tôi thực tế:", "Tôi 1 môi trường, tôi 2 môi trường (nước qua dầu), tôi phân cấp, tôi đẳng nhiệt (tạo Bainit dẻo dai).")
    ]),
    ("2. TỔ CHỨC TẾ VI & CƠ TÍNH", COLOR_NORM, [
        ("• Tổ chức tế vi:", "MACTENXIT (M) dạng hình kim hoặc dạng thanh + một lượng nhỏ Austenite dư."),
        ("• Độ cứng đạt đỉnh:", "HRC 55 ÷ 65 (tùy % C). Thép có độ chịu mài mòn và giới hạn bền cực cao."),
        ("• Nhược điểm chí tử:", "Độ giòn rất cao, độ dai va đập kém, ứng suất dư bên trong rất lớn."),
        ("• NGUYÊN TẮC BẮT BUỘC:", "Chi tiết sau khi tôi KHÔNG BAO GIỜ được đưa vào sử dụng ngay mà PHẢI TIẾN HÀNH RAM NGAY để tránh tự nứt vỡ!")
    ]),
    ("3. ỨNG DỤNG & SẢN PHẨM CỤ THỂ", COLOR_TEMPER, [
        ("• Dụng cụ cắt gọt kim loại:", "Dao tiện, dao phay, mũi khoan, lưỡi cưa sắt (thép hợp kim dụng cụ 9XC, W18Cr4V) cần tôi để đạt độ cứng HRC 62-65 cắt được thép khác."),
        ("• Vòng bi, con lăn bạc đạn:", "Ổ lăn làm từ thép GCr15 / 100Cr6 được tôi dầu đạt HRC 60-64 chống mài mòn và chịu áp lực tiếp xúc cực lớn."),
        ("• Bề mặt răng bánh răng & Ngõng trục:", "Tôi cảm ứng bề mặt (tần số cao) để tạo lớp vỏ mactenxit cứng chống mài mòn, trong khi lõi vẫn giữ nguyên độ dẻo dai.")
    ])
]

for idx, (c_title, c_color, c_items) in enumerate(cards_data_s6):
    cx = Inches(0.8 + idx * 3.95)
    cy = Inches(1.5)
    cw = Inches(3.8)
    ch = Inches(5.4)
    
    card = s6.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = COLOR_CARD_BG
    card.line.color.rgb = c_color
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.18)
    ctf.margin_top = Inches(0.2)
    
    cp1 = ctf.paragraphs[0]
    cp1.text = c_title
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(11)
    cp1.font.bold = True
    cp1.font.color.rgb = c_color
    
    for item in c_items:
        if isinstance(item, (tuple, list)):
            txt = " ".join(str(x) for x in item).strip()
        else:
            txt = str(item)
        pi = ctf.add_paragraph()
        pi.text = txt
        pi.font.name = 'Arial'
        pi.font.size = Pt(9.2)
        pi.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 7: PHƯƠNG PHÁP 4: RAM THÉP (TEMPERING)
# ==============================================================================
s7 = prs.slides.add_slide(blank_layout)
add_header(s7, "Phương Pháp 4: Ram Thép (Tempering)", 
           "Nung nóng thép đã tôi dưới Ac₁ nhằm khử ứng suất và điều chỉnh độ cứng, độ dai theo yêu cầu", "07 / 10")

cards_data_s7 = [
    ("1. RAM THẤP (150 ÷ 250°C)", COLOR_QUENCH, [
        ("• Tổ chức tạo thành:", "Mactenxit ram (cacbit ε phân tán mịn tách ra từ mạng BCT)."),
        ("• Cơ tính đạt được:", "Độ cứng vẫn rất cao HRC 58 ÷ 62, độ bền mòn tốt, giảm bớt ứng suất dư và giảm giòn."),
        ("• Sản phẩm ứng dụng:", ""),
        ("  - Dụng cụ cắt gọt (dao phay, mũi khoan)."),
        ("  - Dụng cụ đo chính xác (thước kẹp, panme)."),
        ("  - Khuôn dập nguội, khuôn miết."),
        ("  - Ổ bi, con lăn bạc đạn, chốt piston."),
        ("  - Bề mặt bánh răng sau thấm cacbon + tôi.")
    ]),
    ("2. RAM TRUNG BÌNH (350 ÷ 450°C)", COLOR_TEMPER, [
        ("• Tổ chức tạo thành:", "Troostite ram (hỗn hợp Ferrite và hạt Xementit rất mịn)."),
        ("• Cơ tính đạt được:", "Đạt GIỚI HẠN ĐÀN HỒI CAO NHẤT (σđh max), độ bền mỏi cao, độ cứng vừa phải HRC 40 ÷ 48."),
        ("• Sản phẩm ứng dụng:", ""),
        ("  - Nhíp lá xe tải, xe khách (thép 65Mn, C70)."),
        ("  - Lò xo cuộn giảm xóc xe máy, ô tô."),
        ("  - Thanh xoắn cân bằng gầm xe hơi."),
        ("  - Vòng đệm vênh chống lỏng bu lông."),
        ("  - Các chi tiết đàn hồi chịu tải va đập lặp.")
    ]),
    ("3. RAM CAO (500 ÷ 650°C)", COLOR_NORM, [
        ("• Tổ chức tạo thành:", "Sorbite ram (Ferrite và Xementit hạt tròn kết tụ)."),
        ("• Cơ tính đạt được:", "Phối hợp tối ưu giữa ĐỘ BỀN CAO (σb ≥ 900 MPa) và ĐỘ DAI VA ĐẬP CAO (ak lớn). HRC 25 ÷ 35 (241-285 HBW)."),
        ("• 'NHIỆT LUYỆN CẢI THIỆN' (Hóa tốt):", "Tôi + Ram cao là quy trình kinh điển cho chi tiết chịu tải động nặng:"),
        ("  - Trục truyền động các-đăng xe tải (thép 40Cr)."),
        ("  - Trục khuỷu, thanh truyền/tay biên động cơ."),
        ("  - Bu lông cường độ cao cấp bền 8.8, 10.9.")
    ])
]

for idx, (c_title, c_color, c_items) in enumerate(cards_data_s7):
    cx = Inches(0.8 + idx * 3.95)
    cy = Inches(1.5)
    cw = Inches(3.8)
    ch = Inches(5.4)
    
    card = s7.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = COLOR_CARD_BG
    card.line.color.rgb = c_color
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.18)
    ctf.margin_top = Inches(0.2)
    
    cp1 = ctf.paragraphs[0]
    cp1.text = c_title
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(11)
    cp1.font.bold = True
    cp1.font.color.rgb = c_color
    
    for item in c_items:
        if isinstance(item, (tuple, list)):
            txt = " ".join(str(x) for x in item).strip()
        else:
            txt = str(item)
        pi = ctf.add_paragraph()
        pi.text = txt
        pi.font.name = 'Arial'
        pi.font.size = Pt(9.2)
        pi.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 8: BẢNG SO SÁNH TỔNG HỢP 4 PHƯƠNG PHÁP NHIỆT LUYỆN
# ==============================================================================
s8 = prs.slides.add_slide(blank_layout)
add_header(s8, "Bảng So Sánh Tổng Hợp 4 Phương Pháp Nhiệt Luyện", 
           "Đối chiếu toàn diện các chỉ tiêu công nghệ, tổ chức tế vi, cơ tính và mục đích sử dụng", "08 / 10")

# Bảng so sánh 5 cột x 5 hàng
tbl_shape = s8.shapes.add_table(5, 5, Inches(0.8), Inches(1.5), Inches(11.7), Inches(5.3))
tbl = tbl_shape.table

# Đặt độ rộng cột
tbl.columns[0].width = Inches(2.1)
tbl.columns[1].width = Inches(2.2)
tbl.columns[2].width = Inches(2.2)
tbl.columns[3].width = Inches(2.4)
tbl.columns[4].width = Inches(2.8)

headers = ["Phương Pháp", "Nhiệt Độ Nung", "Tốc Độ Làm Nguội", "Tổ Chức Tế Vi", "Cơ Tính & Ứng Dụng Điển Hình"]
for col_idx, h_text in enumerate(headers):
    cell = tbl.cell(0, col_idx)
    cell.fill.solid()
    cell.fill.fore_color.rgb = COLOR_BG_DARK
    tf_c = cell.text_frame
    tf_c.word_wrap = True
    p = tf_c.paragraphs[0]
    p.text = h_text
    p.font.name = 'Arial'
    p.font.size = Pt(10)
    p.font.bold = True
    p.font.color.rgb = RGBColor(255, 255, 255)
    p.alignment = PP_ALIGN.CENTER

table_data = [
    ("1. Ủ (Annealing)", 
     "Ac₃ + (20÷40°C) [Ủ hoàn toàn]\nAc₁ + (20÷40°C) [Ủ cầu hóa]\n500÷650°C [Ủ khử ứng suất]", 
     "Rất chậm cùng lò\n(20 ÷ 50°C/h)", 
     "Ferrite + Peclit thô\nhoặc Peclit hạt cầu\n(Pha cân bằng)", 
     "Độ cứng thấp nhất (≤160 HBW), độ dẻo cao.\nKhử ứng suất đúc, hàn; làm mềm thép kéo dây dập nguội bu lông."),
    
    ("2. Thường Hóa\n(Normalizing)", 
     "Ac₃ hoặc Acm + (30÷50°C)\n(Austenit hóa hoàn toàn)", 
     "Vừa phải trong\nkhông khí tĩnh", 
     "Peclit phân tán mịn\nhoặc Sorbit\n(Hạt tinh thể mịn)", 
     "Cơ tính cao hơn ủ (180-210 HBW).\nLàm mịn hạt phôi rèn, cải thiện cắt gọt cho phôi bánh răng, trục xe."),
    
    ("3. Tôi Thép\n(Quenching)", 
     "Ac₃ + (30÷50°C) [Thép <0.77%C]\nAc₁ + (30÷50°C) [Thép >0.77%C]", 
     "Rất nhanh trong\nNước / Dầu (V > Vth)", 
     "Mactenxit (M)\n+ Austenite dư\n(Mạng BCT biến dạng)", 
     "Độ cứng cực đại (HRC 55-65), chống mòn cao, giòn lớn, ứng suất dư rất cao.\nDao tiện, dao phay, mũi khoan, ổ bi bạc đạn."),
    
    ("4. Ram Thép\n(Tempering)", 
     "Dưới Ac₁ (150 ÷ 650°C):\n- Thấp: 150-250°C\n- TB: 350-450°C\n- Cao: 500-650°C", 
     "Trong không khí\nhoặc dầu làm nguội", 
     "- Mactenxit ram\n- Troostite ram\n- Sorbite ram", 
     "Điều chỉnh theo yêu cầu:\n- Ram thấp: Giữ cứng HRC 60 (dao, khuôn dập)\n- Ram TB: Đàn hồi cao (nhíp xe, lò xo)\n- Ram cao: Bền & dai tối ưu (trục khuỷu, trục 40Cr)")
]

for row_idx, row_vals in enumerate(table_data):
    for col_idx, val in enumerate(row_vals):
        cell = tbl.cell(row_idx + 1, col_idx)
        cell.fill.solid()
        if row_idx % 2 == 0:
            cell.fill.fore_color.rgb = RGBColor(255, 255, 255)
        else:
            cell.fill.fore_color.rgb = RGBColor(241, 245, 249)
        tf_c = cell.text_frame
        tf_c.word_wrap = True
        p = tf_c.paragraphs[0]
        p.text = val
        p.font.name = 'Arial'
        p.font.size = Pt(8.8)
        if col_idx == 0:
            p.font.bold = True
            p.font.color.rgb = COLOR_TEXT_MAIN
        else:
            p.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 9: ỨNG DỤNG THỰC TẾ TRONG CHẾ TẠO CHI TIẾT CƠ KHÍ (CASE STUDY 40Cr)
# ==============================================================================
s9 = prs.slides.add_slide(blank_layout)
add_header(s9, "Ứng Dụng Thực Tế Trong Chế Tạo Chi Tiết Cơ Khí (Chương 6)", 
           "Quy trình nhiệt luyện phối hợp gia công trục truyền động 40Cr và bánh răng hộp số", "09 / 10")

# Sơ đồ flow chart ở trên
img_path_proc = os.path.join(ASSETS_DIR, 'case_study_process.png')
if os.path.exists(img_path_proc):
    s9.shapes.add_picture(img_path_proc, Inches(0.8), Inches(1.4), Inches(11.7), Inches(2.5))

# 2 Card phân tích cụ thể chi tiết ở dưới
# Card trái: Trục truyền động các-đăng 40Cr
card_shaft = s9.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.8), Inches(4.1), Inches(5.7), Inches(2.9))
card_shaft.fill.solid()
card_shaft.fill.fore_color.rgb = COLOR_CARD_BG
card_shaft.line.color.rgb = COLOR_NORM
card_shaft.line.width = Pt(1.5)

tf_sh = card_shaft.text_frame
tf_sh.word_wrap = True
tf_sh.margin_left = tf_sh.margin_right = Inches(0.2)
tf_sh.margin_top = Inches(0.15)

psh1 = tf_sh.paragraphs[0]
psh1.text = "CASE STUDY 1: TRỤC TRUYỀN ĐỘNG CÁC-ĐĂNG (THÉP 40Cr)"
psh1.font.name = 'Arial'
psh1.font.size = Pt(10.5)
psh1.font.bold = True
psh1.font.color.rgb = COLOR_NORM

shaft_details = [
    ("• Đặc điểm tải:", "Truyền mô-men xoắn lớn, chịu uốn quay và va đập mỏi liên tục."),
    ("• Bước 1 - Thường hóa (860°C):", "Nguội không khí làm mịn hạt phôi rèn dập, sau đó gia công tiện thô."),
    ("• Bước 2 - Tôi dầu (850°C x 90 phút):", "Austenit hóa hoàn toàn, làm nguội nhanh trong dầu khoáng."),
    ("• Bước 3 - Ram cao (560°C x 150 phút):", "Tạo tổ chức SORBITE RAM. Đạt độ cứng 241-285 HBW, độ bền kéo σb ≥ 900 MPa, độ dai va đập cực tốt chống gãy trục."),
    ("• Bước 4 - Tôi cảm ứng bề mặt:", "Gia nhiệt cao tần ngõng trục lắp then hoa để đạt HRC 50-55 chống mòn.")
]
for th, tb in shaft_details:
    pi = tf_sh.add_paragraph()
    pi.text = th + " " + tb
    pi.font.name = 'Arial'
    pi.font.size = Pt(8.5)
    pi.font.color.rgb = COLOR_TEXT_MAIN

# Card phải: Bánh răng hộp số & Nhíp đàn hồi
card_gear = s9.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, Inches(6.8), Inches(4.1), Inches(5.7), Inches(2.9))
card_gear.fill.solid()
card_gear.fill.fore_color.rgb = COLOR_CARD_BG
card_gear.line.color.rgb = COLOR_TEMPER
card_gear.line.width = Pt(1.5)

tf_g = card_gear.text_frame
tf_g.word_wrap = True
tf_g.margin_left = tf_g.margin_right = Inches(0.2)
tf_g.margin_top = Inches(0.15)

pg1 = tf_g.paragraphs[0]
pg1.text = "CASE STUDY 2: BÁNH RĂNG HỘP SỐ & NHÍP ĐÀN HỒI"
pg1.font.name = 'Arial'
pg1.font.size = Pt(10.5)
pg1.font.bold = True
pg1.font.color.rgb = COLOR_TEMPER

gear_details = [
    ("• BÁNH RĂNG HỘP SỐ (Thép 20Cr, 20CrMnTi):", ""),
    ("  - Phôi dập → Thường hóa (180 HBW) → Phay/xọc răng hoàn thiện."),
    ("  - Thấm Cacbon bề mặt răng (900-930°C) tạo lớp giàu C 0.8-1.0%."),
    ("  - Tôi + Ram thấp (180°C): Mặt răng đạt HRC 58-62 chống tróc mòn cực đỉnh, trong khi lõi ít C giữ nguyên độ dai chịu uốn va đập chân răng."),
    ("• NHÍP LÁ & LÒ XO Ô TÔ (Thép 65Mn, C70):", ""),
    ("  - Tạo hình nóng → Tôi dầu 830°C → Ram trung bình 420°C."),
    ("  - Tạo pha Troostite ram đạt giới hạn đàn hồi cao nhất (HRC 40-45).")
]
for item in gear_details:
    if isinstance(item, (tuple, list)):
        txt = " ".join(str(x) for x in item).strip()
    else:
        txt = str(item)
    pi = tf_g.add_paragraph()
    pi.text = txt
    pi.font.name = 'Arial'
    pi.font.size = Pt(8.5)
    pi.font.color.rgb = COLOR_TEXT_MAIN

# ==============================================================================
# SLIDE 10: TỔNG KẾT & QUY TẮC LỰA CHỌN NHIỆT LUYỆN
# ==============================================================================
s10 = prs.slides.add_slide(blank_layout)
add_header(s10, "Tổng Kết & Quy Tắc Lựa Chọn Nhiệt Luyện Trong Thực Tế", 
           "Cẩm nang ghi nhớ nhanh cho kỹ sư cơ khí chế tạo và nghiên cứu vật liệu", "10 / 10")

# 4 Card quyết định chọn phương pháp
rules = [
    ("YÊU CẦU: MỀM, DỄ DẬP, KHỬ ỨNG SUẤT", "👉 CHỌN PHƯƠNG PHÁP Ủ (ANNEALING)", COLOR_ANNEAL, 
     "• Dùng cho phôi đúc, phôi hàn, dây thép kéo nguội dập ốc vít.\n• Nguội rất chậm cùng lò để đưa vật liệu về tổ chức cân bằng mềm dẻo nhất."),
    
    ("YÊU CẦU: ĐỒNG NHẤT, MỊN HẠT, DỄ TIỆN PHAY", "👉 CHỌN THƯỜNG HÓA (NORMALIZING)", COLOR_NORM, 
     "• Bước nhiệt luyện sơ bộ cho mọi phôi rèn, phôi đúc bánh răng, trục trước khi gia công cắt gọt.\n• Nguội không khí tĩnh, phoi cắt dễ thoát, dao không bị bết dính."),
    
    ("YÊU CẦU: ĐỘ CỨNG CAO NHẤT, CHỐNG MÒN", "👉 CHỌN TÔI + RAM THẤP (150 - 250°C)", COLOR_QUENCH, 
     "• Dùng cho dao tiện, dao phay, mũi khoan, khuôn dập nguội, ổ bi, bề mặt răng bánh răng.\n• Nguội nhanh tạo Mactenxit, bắt buộc ram thấp ngay để tránh nứt vỡ."),
    
    ("YÊU CẦU: ĐÀN HỒI HOẶC BỀN DAI TỐI ƯU", "👉 CHỌN TÔI + RAM TB / RAM CAO", COLOR_TEMPER, 
     "• Đàn hồi (Nhíp, lò xo): Tôi + Ram trung bình 400°C (Troostite ram, HRC 40-48).\n• Bền dai (Trục khuỷu, trục 40Cr): Tôi + Ram cao 560°C (Nhiệt luyện cải thiện, Sorbit ram).")
]

for idx, (r_target, r_choice, r_col, r_desc) in enumerate(rules):
    row = idx // 2
    col = idx % 2
    cx = Inches(0.8 + col * 5.95)
    cy = Inches(1.5 + row * 2.7)
    cw = Inches(5.75)
    ch = Inches(2.5)
    
    card = s10.shapes.add_shape(MSO_SHAPE.ROUNDED_RECTANGLE, cx, cy, cw, ch)
    card.fill.solid()
    card.fill.fore_color.rgb = COLOR_CARD_BG
    card.line.color.rgb = r_col
    card.line.width = Pt(1.5)
    
    ctf = card.text_frame
    ctf.word_wrap = True
    ctf.margin_left = ctf.margin_right = Inches(0.2)
    ctf.margin_top = Inches(0.15)
    
    cp1 = ctf.paragraphs[0]
    cp1.text = r_target
    cp1.font.name = 'Arial'
    cp1.font.size = Pt(10)
    cp1.font.bold = True
    cp1.font.color.rgb = COLOR_TEXT_MUTED
    
    cp2 = ctf.add_paragraph()
    cp2.text = r_choice
    cp2.font.name = 'Arial'
    cp2.font.size = Pt(11.5)
    cp2.font.bold = True
    cp2.font.color.rgb = r_col
    
    cp3 = ctf.add_paragraph()
    cp3.text = r_desc
    cp3.font.name = 'Arial'
    cp3.font.size = Pt(9.2)
    cp3.font.color.rgb = COLOR_TEXT_MAIN

# Lưu file PPTX trực tiếp ra Desktop
prs.save(DESKTOP_PATH)
print(f"SUCCESS: Successfully created presentation with {len(prs.slides)} slides at: {DESKTOP_PATH}")
