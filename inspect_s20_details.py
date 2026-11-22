from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
s20 = prs.slides[19]

for sh in s20.shapes:
    print(f"Name: {sh.name}, shape_type: {sh.shape_type}")
    print(f"  left={sh.left}, top={sh.top}, width={sh.width}, height={sh.height}")
    if sh.has_text_frame:
        print(f"  Text: {sh.text_frame.text[:60]}")
