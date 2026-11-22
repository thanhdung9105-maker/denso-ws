from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx')
s23 = prs.slides[22]
print(f"Slide 23 shapes count: {len(s23.shapes)}")
for i, sh in enumerate(s23.shapes):
    t = sh.text_frame.text if sh.has_text_frame else ""
    print(f"Shape {i}: name='{sh.name}', type={sh.shape_type}, left={sh.left}, top={sh.top}, w={sh.width}, h={sh.height}")
    if t:
        print(f"   text: {t[:60]}")
