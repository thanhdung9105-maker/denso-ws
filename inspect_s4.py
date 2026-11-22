from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_300ml.pptx')
s4 = prs.slides[3] # slide 4 (0-indexed 3)

print("=== SLIDE 4 ===")
for sh in s4.shapes:
    if sh.has_text_frame:
        print(f"Shape: {sh.name}")
        for p in sh.text_frame.paragraphs:
            if p.text.strip():
                print("  -", p.text.strip())
