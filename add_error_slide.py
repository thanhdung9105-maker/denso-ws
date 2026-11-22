import pptx
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR
from pptx.enum.shapes import MSO_SHAPE

pptx_path = 'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
prs = Presentation(pptx_path)

# Slide 20 is at index 19
s20 = prs.slides[19]

# Create new slide using s20 layout
new_slide = prs.slides.add_slide(s20.slide_layout)

# Move new slide to index 20 (right after slide 20)
sldId = prs.slides._sldIdLst[-1]
prs.slides._sldIdLst.insert(20, sldId)

# Add decorative rectangles on left edge like other slides
# Shape 1: Rectangle 16 (Top left blue strip)
rect16 = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 0, 244475, 840828)
rect16.fill.solid()
rect16.fill.fore_color.rgb = RGBColor(0, 112, 192)
rect16.line.fill.background()

# Shape 2: Rectangle 10 (Bottom left blue accent)
rect10 = new_slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, 0, 430886, 244475, 861885)
rect10.fill.solid()
rect10.fill.fore_color.rgb = RGBColor(0, 112, 192)
rect10.line.fill.background()

# Shape 3: Slide Number Placeholder
slide_num_box = new_slide.shapes.add_textbox(11655911, 6444466, 536089, 413534)
tf_num = slide_num_box.text_frame
tf_num.word_wrap = True
p_num = tf_num.paragraphs[0]
p_num.text = "21"
p_num.font.name = "Calibri"
p_num.font.size = Pt(14)
p_num.font.color.rgb = RGBColor(120, 144, 156)

# Shape 4: Slide Title
title_box = new_slide.shapes.add_textbox(402336, 283464, 11521440, 784830)
tf_title = title_box.text_frame
tf_title.word_wrap = True
p_title = tf_title.paragraphs[0]
p_title.text = "ĐÁNH GIÁ SAI SỐ VÀ ĐỘ CHÍNH XÁC HỆ THỐNG"
p_title.font.name = "Calibri"
p_title.font.size = Pt(28)
p_title.font.bold = True
p_title.font.color.rgb = RGBColor(0, 112, 192)

# Shape 5: Main Evaluation Table
# Width: 10424160, Left: 914400, Top: 1200000, Height: 3300000
rows = 6
cols = 5
left = 914400
top = 1180000
width = 10424160
height = 3600000

table_shape = new_slide.shapes.add_table(rows, cols, left, top, width, height)
table = table_shape.table

# Set column widths
col_widths = [1900000, 2600000, 1800000, 1600000, 2524160]
for idx, w in enumerate(col_widths):
    table.columns[idx].width = w

table_data = [
    # Header
    ["Dải nồng độ CO₂", "Môi trường khảo sát", "Sai số tuyệt đối (Δ)", "Sai số tương đối (δ)", "Đánh giá & Đáp ứng quạt"],
    # Row 1
    ["400 – 650 ppm", "Ngoài trời / Phòng thoáng khí", "±20 – ±28 ppm", "±4,8 % – ±5,3 %", "Rất tốt · Nền sạch, quạt tắt (0 PWM)"],
    # Row 2
    ["650 – 950 ppm", "Phòng kín sinh hoạt bình thường", "±35 – ±50 ppm", "±5,2 % – ±5,9 %", "Tốt · ASHRAE, quạt chạy êm (35–55%)"],
    # Row 3
    ["950 – 1300 ppm", "Phòng kín đông người, bí khí", "±55 – ±78 ppm", "±5,8 % – ±6,5 %", "Đạt IAQ · Cảnh báo CAO, quạt tăng tốc"],
    # Row 4
    ["≥ 1300 ppm (đỉnh 4000)", "Thổi trực tiếp hơi thở vào cảm biến", "±80 – ±145 ppm", "±6,2 % – ±7,5 %", "Nhạy tức thì · Quạt full 100% (255 PWM)"],
    # Row 5 (Tổng hợp)
    ["Tổng hợp toàn dải", "Độ không đảm bảo đo mở rộng", "MAE ≈ 42 ppm", "U ≈ ±6,1 % (k = 2)", "Đạt cấp 2 thiết bị IAQ cảnh báo xu hướng"]
]

for r_i, row in enumerate(table_data):
    for c_i, val in enumerate(row):
        cell = table.rows[r_i].cells[c_i]
        cell.vertical_anchor = MSO_ANCHOR.MIDDLE
        cell.text = val
        p = cell.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER if c_i in [0, 2, 3] else PP_ALIGN.LEFT
        p.font.name = "Calibri"
        if r_i == 0:
            p.font.bold = True
            p.font.size = Pt(13)
            cell.fill.solid()
            cell.fill.fore_color.rgb = RGBColor(13, 71, 161)
            p.font.color.rgb = RGBColor(255, 255, 255)
        elif r_i == 5:
            p.font.bold = True
            p.font.size = Pt(11.5)
            cell.fill.solid()
            cell.fill.fore_color.rgb = RGBColor(227, 242, 253)
            p.font.color.rgb = RGBColor(13, 71, 161)
        else:
            p.font.size = Pt(11)
            p.font.color.rgb = RGBColor(38, 50, 56)
            if r_i % 2 == 1:
                cell.fill.solid()
                cell.fill.fore_color.rgb = RGBColor(248, 249, 250)
            else:
                cell.fill.solid()
                cell.fill.fore_color.rgb = RGBColor(255, 255, 255)

# Shape 6: Bottom Summary & Solutions Box
card_box = new_slide.shapes.add_textbox(914400, 5000000, 10424160, 1300000)
tf_card = card_box.text_frame
tf_card.word_wrap = True

p_c1 = tf_card.paragraphs[0]
p_c1.text = "NHẬN XÉT VÀ GIẢI PHÁP TRIỆT TIÊU SAI SỐ ĐÃ TRIỂN KHAI TRONG CODE MỚI:"
p_c1.font.name = "Calibri"
p_c1.font.size = Pt(12)
p_c1.font.bold = True
p_c1.font.color.rgb = RGBColor(0, 112, 192)

points = [
    "• Khử nhiễu & chống đơ số liệu: Thuật toán Oversampling 20 mẫu ADC liên tục (chu kỳ 400 µs) triệt tiêu hoàn toàn hiện tượng đứng số, đồ thị 6 kênh dao động live.",
    "• Bù trôi nhiệt ẩm môi trường: Áp dụng phương trình đa thức Hanwei (theo nhiệt độ T và độ ẩm RH) giúp hệ số COR tự thích nghi, triệt tiêu sai số trôi thời tiết.",
    "• Triệt tiêu sai số điều khiển quạt: Vùng trễ Schmitt Trigger 25 ppm kết hợp khởi động mềm Soft-ramp (+14/−6 mỗi 100 ms) chống rung giật rơ-le và sụt áp USB."
]

for pt in points:
    p = tf_card.add_paragraph()
    p.text = pt
    p.font.name = "Calibri"
    p.font.size = Pt(11)
    p.font.color.rgb = RGBColor(55, 71, 79)

# Update slide numbers for all slides from 21 onwards
for idx, s in enumerate(prs.slides):
    s_actual_num = idx + 1
    for sh in s.shapes:
        if sh.name == "Slide Number Placeholder 1" and sh.has_text_frame:
            sh.text_frame.text = str(s_actual_num)
            for p in sh.text_frame.paragraphs:
                p.font.name = "Calibri"
                p.font.size = Pt(14)
                p.font.color.rgb = RGBColor(120, 144, 156)

# Save to all target paths
output_paths = [
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/OneDrive/Máy tính/Thuyet_trinh_He_thong_giam_sat_CO2_Moi.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
]

for out_p in output_paths:
    prs.save(out_p)
    print(f"Successfully saved: {out_p}")

print(f"Slide addition complete! Total slides now: {len(prs.slides)}")
