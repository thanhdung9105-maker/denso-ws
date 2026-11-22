from pptx import Presentation
from pptx.util import Pt
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN

pptx_path = 'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
prs = Presentation(pptx_path)

# Slide 21 is at index 20
s21 = prs.slides[20]

# Update row 4 of the table
for sh in s21.shapes:
    if sh.has_table:
        table = sh.table
        table.rows[4].cells[0].text = "1300 – 2000 ppm"
        table.rows[4].cells[1].text = "Thổi trực tiếp hơi thở vào cảm biến"
        table.rows[4].cells[2].text = "±75 – ±120 ppm"
        table.rows[4].cells[3].text = "±5,8 % – ±6,8 %"
        table.rows[4].cells[4].text = "Thực nghiệm 1800–2000 ppm · Quạt full 100% (255 PWM)"
        for c_i in range(5):
            p = table.rows[4].cells[c_i].text_frame.paragraphs[0]
            p.font.name = "Calibri"
            p.font.size = Pt(11)
            p.alignment = PP_ALIGN.CENTER if c_i in [0, 2, 3] else PP_ALIGN.LEFT
    elif sh.has_text_frame and "NHẬN XÉT VÀ GIẢI PHÁP" in sh.text_frame.text:
        tf = sh.text_frame
        tf.clear()
        p0 = tf.paragraphs[0]
        p0.text = "NHẬN XÉT VÀ KẾT QUẢ THỰC NGHIỆM ĐÁNH GIÁ SAI SỐ:"
        p0.font.name = "Calibri"
        p0.font.size = Pt(12)
        p0.font.bold = True
        p0.font.color.rgb = RGBColor(0, 112, 192)
        
        points = [
            "• Phản xạ thực tế với hơi thở: Khi thổi hơi thở trực tiếp, nồng độ nhảy tức thì từ nền lên 1800 – 2000 ppm, quạt lập tức kích hoạt 100% (255 PWM) và đưa ra cảnh báo mức NGUY hiểm.",
            "• Khử triệt để hiện tượng đơ cảm biến: Thuật toán Oversampling 20 mẫu ADC (chu kỳ 400 µs) giúp vi xử lý bám sát vi biến động nồng độ thực mà không bị đóng băng số liệu.",
            "• Độ không đảm bảo đo mở rộng toàn dải U ≈ ±6,1% (k = 2, P = 95%): Hoàn toàn đáp ứng tiêu chuẩn IAQ dân dụng và công nghiệp về thiết bị giám sát - cảnh báo chất lượng không khí."
        ]
        for pt in points:
            p = tf.add_paragraph()
            p.text = pt
            p.font.name = "Calibri"
            p.font.size = Pt(11)
            p.font.color.rgb = RGBColor(55, 71, 79)

prs.save(pptx_path)
prs.save('c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
print("Updated slide 21 with breath experiment results!")
