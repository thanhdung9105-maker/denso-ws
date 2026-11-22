import pptx
from pptx import Presentation

prs = Presentation('c:/Users/thanh/denso_ws/temp_presentation.pptx')

for s_idx in [5, 7, 8, 9, 10, 11, 14, 18, 19, 20, 21, 25]:
    slide = prs.slides[s_idx - 1]
    print(f'=== SLIDE {s_idx} SHAPES ===')
    for i, shape in enumerate(slide.shapes):
        sh_type = str(shape.shape_type)
        text_preview = ""
        if shape.has_text_frame:
            text_preview = " | ".join([p.text.strip() for p in shape.text_frame.paragraphs if p.text.strip()])[:60]
        elif shape.has_table:
            text_preview = f"TABLE ({len(shape.table.rows)}x{len(shape.table.columns)})"
        print(f'  Shape {i}: name="{shape.name}", type={sh_type}, text="{text_preview}"')

