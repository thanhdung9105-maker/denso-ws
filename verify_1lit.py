from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_1Lit.pptx')
s24 = prs.slides[23]
print("Verified Slide 24 title:")
for sh in s24.shapes:
    if sh.has_text_frame and "TÍNH TOÁN" in sh.text_frame.text:
        print(" ", sh.text_frame.text)
