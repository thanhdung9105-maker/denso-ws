from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_LocNhieu.pptx')
s9 = prs.slides[8] # index 8 = Slide 9
print("Total slides:", len(prs.slides))
print("Slide 9 shapes:")
for sh in s9.shapes:
    if sh.has_text_frame and sh.text_frame.text.strip():
        print(" -", sh.text_frame.text.strip()[:60])
