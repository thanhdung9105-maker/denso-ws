from PIL import Image

for name in ['extracted_7_eq_ppm.png', 'extracted_8_eq_cor.png', 'extracted_8_eq_ema.png', 'extracted_9_dac_tuyen_b.png', 'extracted_10_dieu_khien_quat.png']:
    im = Image.open(f'c:/Users/thanh/denso_ws/{name}')
    print(f'{name}: size={im.size}, mode={im.mode}')

