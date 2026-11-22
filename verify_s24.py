from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
print(f"Total slides: {len(prs.slides)}")

s23 = prs.slides[22]
s24 = prs.slides[23]

print("=== SLIDE 23 ===")
for sh in s23.shapes:
    if sh.has_text_frame and sh.text_frame.text.strip():
        print(" ", sh.text_frame.text.strip()[:60])

print("\n=== SLIDE 24 ===")
for sh in s24.shapes:
    if sh.has_text_frame and sh.text_frame.text.strip():
        print(" ", sh.text_frame.text.strip()[:60])
