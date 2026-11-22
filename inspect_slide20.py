from pptx import Presentation

prs = Presentation('c:/Users/thanh/denso_ws/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
print("Slide width:", prs.slide_width, "height:", prs.slide_height)
# check slide 20 layout and shapes
s20 = prs.slides[19]
print("Slide 20 layout:", s20.slide_layout.name)
for i, sh in enumerate(s20.shapes):
    print(f"Shape {i}: name='{sh.name}', type={sh.shape_type}, left={sh.left}, top={sh.top}, w={sh.width}, h={sh.height}")
