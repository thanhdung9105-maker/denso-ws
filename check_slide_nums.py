from pptx import Presentation

prs = Presentation('c:/Users/thanh/OneDrive/Máy tính/Bao_Cao_He_Thong_Giam_Sat_CO2_CapNhat.pptx')
print(f'Total slides: {len(prs.slides)}')
for idx, s in enumerate(prs.slides):
    num_txt = ''
    title = ''
    for sh in s.shapes:
        if sh.name == 'Slide Number Placeholder 1' and sh.has_text_frame:
            num_txt = sh.text_frame.text.strip()
        elif sh.has_text_frame and not title:
            title = sh.text_frame.text.strip().replace('\n', ' ')
    if idx >= 18:
        print(f'Slide index {idx}: actual slide #{idx+1}, placeholder number="{num_txt}", title="{title[:50]}"')
