from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_Moi_DeHieu.pptx')
s23 = prs.slides[22]
for sh in s23.shapes:
    if sh.name == 'box_breath.png':
        with open('c:/Users/thanh/denso_ws/extracted_box_breath.png', 'wb') as f:
            f.write(sh.image.blob)
        print("Extracted box_breath.png successfully")
