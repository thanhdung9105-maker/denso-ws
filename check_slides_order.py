from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_300ml.pptx')
print(f"Total slides: {len(prs.slides)}")
for i, s in enumerate(prs.slides):
    t = ""
    for sh in s.shapes:
        if sh.has_text_frame and sh.text_frame.text.strip():
            txt = sh.text_frame.text.strip().replace('\n', ' ')
            if not t and len(txt) > 2:
                t = txt
    print(f"Slide {i+1}: {t[:60]}")
