import pptx
from pptx import Presentation
import os

prs = Presentation('c:/Users/thanh/denso_ws/temp_presentation.pptx')

# Extract images from slide 7 and slide 8
for s_idx in [7, 8, 9, 10, 14]:
    slide = prs.slides[s_idx - 1]
    for shape in slide.shapes:
        if shape.shape_type == pptx.enum.shapes.MSO_SHAPE_TYPE.PICTURE:
            img = shape.image
            filename = f"c:/Users/thanh/denso_ws/extracted_{s_idx}_{shape.name}"
            with open(filename, "wb") as f:
                f.write(img.blob)
            print(f"Saved {filename} ({len(img.blob)} bytes)")

