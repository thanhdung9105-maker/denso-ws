import pptx
from pptx import Presentation
from pptx.util import Pt
from pptx.dml.color import RGBColor

# Open the working presentation
prs = Presentation('c:/Users/thanh/denso_ws/temp_slide18.pptx')
slide18 = prs.slides[17] # Slide 18

for sh in slide18.shapes:
    if sh.name == "Thẻ Độ chính xác" and sh.has_text_frame:
        tf = sh.text_frame
        tf.clear()
        
        # Title "Ưu điểm"
        p0 = tf.paragraphs[0]
        p0.text = "Ưu điểm nổi bật"
        p0.font.name = "Calibri"
        p0.font.size = Pt(15)
        p0.font.bold = True
        p0.font.color.rgb = RGBColor(13, 71, 161)
        p0.space_after = Pt(8)
        
        # 5 Easy-to-understand bullet points
        points = [
            ("Đo tỷ lệ điện áp:", " Nguồn USB dao động kết quả vẫn chuẩn xác (Vc tự triệt tiêu)."),
            ("Chống đơ cảm biến:", " Oversampling 20 mẫu ADC, số nhảy tức thì khi có hơi thở."),
            ("Đồ thị trực quan:", " Xuất đồng thời 6 đường sóng động trên Arduino Serial Plotter."),
            ("Quạt chạy êm & bền:", " Khởi động mềm (+14/−6), có trễ 25 ppm chống rung giật quạt."),
            ("Tự phục hồi LCD 1602:", " Tự động kết nối lại nếu dây I²C bị lỏng hay chập chờn.")
        ]
        
        for bold_txt, norm_txt in points:
            p = tf.add_paragraph()
            p.space_after = Pt(6)
            
            # Bullet symbol + Bold keyword
            r_bold = p.add_run()
            r_bold.text = "• " + bold_txt
            r_bold.font.name = "Calibri"
            r_bold.font.size = Pt(11.5)
            r_bold.font.bold = True
            r_bold.font.color.rgb = RGBColor(13, 71, 161)
            
            # Normal explanation
            r_norm = p.add_run()
            r_norm.text = norm_txt
            r_norm.font.name = "Calibri"
            r_norm.font.size = Pt(11)
            r_norm.font.bold = False
            r_norm.font.color.rgb = RGBColor(38, 50, 56)

    elif sh.name == "Thẻ Xu hướng sai số" and sh.has_text_frame:
        tf = sh.text_frame
        tf.clear()
        
        p0 = tf.paragraphs[0]
        p0.text = "Giới hạn & Hướng khắc phục"
        p0.font.name = "Calibri"
        p0.font.size = Pt(15)
        p0.font.bold = True
        p0.font.color.rgb = RGBColor(198, 40, 40)
        p0.space_after = Pt(8)
        
        points_limit = [
            ("Cảm biến bán dẫn MQ-135:", " Đo CO₂ tương đương, nhạy chéo với khí khử khác."),
            ("Cảm biến nhiệt ẩm DHT11:", " Sai số ±2 °C làm lệch CO₂ ≈ 4 % (đã bù nhiệt ẩm Hanwei)."),
            ("Thời gian sấy ban đầu:", " Cần sấy nóng dây may-so 24–48 h để điện trở Rs đạt độ ổn định cao nhất.")
        ]
        for bold_txt, norm_txt in points_limit:
            p = tf.add_paragraph()
            p.space_after = Pt(8)
            
            r_bold = p.add_run()
            r_bold.text = "• " + bold_txt
            r_bold.font.name = "Calibri"
            r_bold.font.size = Pt(11.5)
            r_bold.font.bold = True
            r_bold.font.color.rgb = RGBColor(198, 40, 40)
            
            r_norm = p.add_run()
            r_norm.text = norm_txt
            r_norm.font.name = "Calibri"
            r_norm.font.size = Pt(11)
            r_norm.font.bold = False
            r_norm.font.color.rgb = RGBColor(55, 71, 79)

# Save to destination files
save_targets = [
    'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx',
    'c:/Users/thanh/Desktop/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx',
    'c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
]

for t in save_targets:
    try:
        prs.save(t)
        print(f"[OK] Saved to: {t}")
    except Exception as e:
        print(f"[ERR] Failed saving to {t}: {e}")

# Also try saving to the original Desktop path if unlocked
try:
    prs.save('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
    print("[OK] Overwritten original Desktop file successfully!")
except Exception as e:
    print("[INFO] Original Desktop file is currently open in PowerPoint (kept open by user).")
