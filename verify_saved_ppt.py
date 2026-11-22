import pptx
from pptx import Presentation

test_path = 'c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx'
prs = Presentation(test_path)
print(f"Verified opening {test_path}, total slides: {len(prs.slides)}")

check_slides = [5, 7, 8, 9, 10, 14, 18, 19, 20, 21, 25]

for s_num in check_slides:
    slide = prs.slides[s_num - 1]
    print(f"\n--- SLIDE {s_num} ---")
    for sh in slide.shapes:
        if sh.has_text_frame:
            for p in sh.text_frame.paragraphs:
                if p.text.strip():
                    print(f"  [Text]: {p.text.strip()}")
        elif sh.has_table:
            print("  [Table]:")
            for r in sh.table.rows:
                print("    " + " | ".join([c.text.strip() for c in r.cells]))
        elif sh.shape_type == pptx.enum.shapes.MSO_SHAPE_TYPE.PICTURE:
            print(f"  [Picture]: {sh.name} (size {len(sh.image.blob)} bytes)")

