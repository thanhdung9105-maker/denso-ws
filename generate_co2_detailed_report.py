import os
import sys
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import cm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether, HRFlowable
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

# Đăng ký font Times New Roman
pdfmetrics.registerFont(TTFont('TimesNewRoman', r'C:\Windows\Fonts\times.ttf'))
pdfmetrics.registerFont(TTFont('TimesNewRoman-Bold', r'C:\Windows\Fonts\timesbd.ttf'))
pdfmetrics.registerFont(TTFont('TimesNewRoman-Italic', r'C:\Windows\Fonts\timesi.ttf'))
pdfmetrics.registerFont(TTFont('TimesNewRoman-BoldItalic', r'C:\Windows\Fonts\timesbi.ttf'))

pdfmetrics.registerFontFamily(
    'TimesNewRoman',
    normal='TimesNewRoman',
    bold='TimesNewRoman-Bold',
    italic='TimesNewRoman-Italic',
    boldItalic='TimesNewRoman-BoldItalic'
)

OUTPUT_PDF = r'C:\Users\thanh\Desktop\Bao_Cao_Chi_Tiet_He_Thong_Do_CO2.pdf'

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        if self._pageNumber == 1:
            return  # Trang bìa không vẽ header/footer
        self.saveState()
        self.setFont('TimesNewRoman-Italic', 9)
        self.setFillColor(colors.HexColor('#475569'))
        
        # Header
        self.drawString(2.0 * cm, 28.5 * cm, "ĐH Bách Khoa TP.HCM - Báo cáo Chuyên sâu Hệ thống Đo CO2 & Điều khiển Thông gió")
        self.setStrokeColor(colors.HexColor('#cbd5e1'))
        self.setLineWidth(0.5)
        self.line(2.0 * cm, 28.3 * cm, 19.0 * cm, 28.3 * cm)
        
        # Footer
        self.line(2.0 * cm, 1.8 * cm, 19.0 * cm, 1.8 * cm)
        self.drawString(2.0 * cm, 1.4 * cm, "Khoa Khoa Học Ứng Dụng | Môn học: Vật Lý / Kỹ thuật Đo lường")
        page_text = f"Trang {self._pageNumber} / {page_count}"
        self.drawRightString(19.0 * cm, 1.4 * cm, page_text)
        self.restoreState()

def build_pdf():
    doc = SimpleDocTemplate(
        OUTPUT_PDF,
        pagesize=A4,
        leftMargin=2.0 * cm,
        rightMargin=2.0 * cm,
        topMargin=2.0 * cm,
        bottomMargin=2.2 * cm
    )

    styles = getSampleStyleSheet()
    
    # Định nghĩa các style chuẩn với cỡ chữ 13 pt và Times New Roman
    style_title = ParagraphStyle(
        'DocTitle',
        fontName='TimesNewRoman-Bold',
        fontSize=18,
        leading=22,
        alignment=1, # Căn giữa
        textColor=colors.HexColor('#0f172a'),
        spaceAfter=12
    )

    style_subtitle = ParagraphStyle(
        'DocSubtitle',
        fontName='TimesNewRoman-BoldItalic',
        fontSize=13,
        leading=17,
        alignment=1,
        textColor=colors.HexColor('#1e40af'),
        spaceAfter=18
    )

    style_h1 = ParagraphStyle(
        'Heading1_Custom',
        fontName='TimesNewRoman-Bold',
        fontSize=14,
        leading=18,
        textColor=colors.HexColor('#0f2942'),
        spaceBefore=14,
        spaceAfter=8,
        keepWithNext=True
    )

    style_h2 = ParagraphStyle(
        'Heading2_Custom',
        fontName='TimesNewRoman-Bold',
        fontSize=13,
        leading=17,
        textColor=colors.HexColor('#1d4ed8'),
        spaceBefore=10,
        spaceAfter=6,
        keepWithNext=True
    )

    style_body = ParagraphStyle(
        'Body_Custom',
        fontName='TimesNewRoman',
        fontSize=13,
        leading=18,
        textColor=colors.HexColor('#1e293b'),
        alignment=4, # Justify
        spaceAfter=7
    )

    style_body_bold = ParagraphStyle(
        'Body_Bold_Custom',
        fontName='TimesNewRoman-Bold',
        fontSize=13,
        leading=18,
        textColor=colors.HexColor('#0f172a'),
        spaceAfter=6
    )

    style_bullet = ParagraphStyle(
        'Bullet_Custom',
        fontName='TimesNewRoman',
        fontSize=13,
        leading=18,
        textColor=colors.HexColor('#1e293b'),
        leftIndent=18,
        spaceAfter=4,
        alignment=4
    )

    style_code_explain = ParagraphStyle(
        'CodeExplain',
        fontName='TimesNewRoman',
        fontSize=12,
        leading=16,
        textColor=colors.HexColor('#0f172a'),
        spaceAfter=4
    )

    style_callout = ParagraphStyle(
        'CalloutText',
        fontName='TimesNewRoman-Italic',
        fontSize=12.5,
        leading=17,
        textColor=colors.HexColor('#991b1b'),
        alignment=4
    )

    story = []

    # ==========================================
    # TRANG BÌA
    # ==========================================
    header_cover = ParagraphStyle('CoverHeader', fontName='TimesNewRoman-Bold', fontSize=12, leading=15, alignment=1)
    story.append(Paragraph("ĐẠI HỌC QUỐC GIA THÀNH PHỐ HỒ CHÍ MINH<br/>TRƯỜNG ĐẠI HỌC BÁCH KHOA<br/>KHOA KHOA HỌC ỨNG DỤNG", header_cover))
    story.append(Spacer(1, 15))
    story.append(HRFlowable(width="60%", thickness=1.5, color=colors.HexColor('#1e3a8a'), spaceBefore=5, spaceAfter=25))
    
    story.append(Paragraph("BÁO CÁO THUYẾT MINH BÀI TẬP LỚN", ParagraphStyle('ReportType', fontName='TimesNewRoman-Bold', fontSize=14, leading=17, alignment=1, textColor=colors.HexColor('#b91c1c'))))
    story.append(Spacer(1, 10))
    story.append(Paragraph("HỆ THỐNG GIÁM SÁT NỒNG ĐỘ KHÍ CO₂, NHIỆT ẨM ĐỘ VÀ ĐIỀU KHIỂN THÔNG GIÓ TỰ ĐỘNG", style_title))
    story.append(Paragraph("NGHIÊN CỨU CHUYÊN SÂU: PHƯƠNG PHÁP HIỆU CHUẨN, KIỂM SOÁT TÍN HIỆU & SAI SỐ, GIẢI THÍCH MÃ NGUỒN ARDUINO VÀ GIỚI HẠN VẬT LÝ CỦA CẢM BIẾN OXIT KIM LOẠI (MOX)", style_subtitle))
    story.append(Spacer(1, 20))

    meta_table_data = [
        [Paragraph("<b>Giảng viên hướng dẫn:</b>", style_body), Paragraph("TS. Nguyễn Duy Khương", style_body)],
        [Paragraph("<b>Môn học:</b>", style_body), Paragraph("Vật liệu học và Xử lý / Kỹ thuật Đo lường", style_body)],
        [Paragraph("<b>Nền tảng phần cứng:</b>", style_body), Paragraph("Arduino Uno R3, Cảm biến MQ-135, DHT11, MOSFET D4184, LCD 1602 I2C", style_body)],
        [Paragraph("<b>Quy chuẩn hiệu chuẩn:</b>", style_body), Paragraph("6 Lần đo thực nghiệm khí quyển ngoài trời sáng sớm (400.0 ppm chuẩn)", style_body)],
        [Paragraph("<b>Độ phân giải hiển thị:</b>", style_body), Paragraph("Oversampling 20 mẫu ADC (12-bit tương đương), 6 kênh Serial Plotter", style_body)]
    ]
    meta_table = Table(meta_table_data, colWidths=[6.0*cm, 11.0*cm])
    meta_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor('#f8fafc')),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor('#cbd5e1')),
        ('INNERGRID', (0,0), (-1,-1), 0.5, colors.HexColor('#e2e8f0')),
        ('TOPPADDING', (0,0), (-1,-1), 6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6),
        ('LEFTPADDING', (0,0), (-1,-1), 10),
        ('RIGHTPADDING', (0,0), (-1,-1), 10),
    ]))
    story.append(meta_table)
    story.append(Spacer(1, 35))
    story.append(Paragraph("<i>Thành phố Hồ Chí Minh, Năm 2026</i>", ParagraphStyle('DateFoot', fontName='TimesNewRoman-Italic', fontSize=12, alignment=1, textColor=colors.HexColor('#64748b'))))
    story.append(PageBreak())

    # ==========================================
    # PHẦN 1: TỔNG QUAN HỆ THỐNG & SƠ ĐỒ NGUYÊN LÝ
    # ==========================================
    story.append(Paragraph("1. TỔNG QUAN HỆ THỐNG VÀ NGUYÊN LÝ HOẠT ĐỘNG", style_h1))
    story.append(Paragraph(
        "Hệ thống được thiết kế với mục tiêu giám sát thời gian thực nồng độ khí CO₂ trong không gian phòng kín, kết hợp đo nhiệt độ - độ ẩm môi trường và tự động kích hoạt quạt hút thông gió theo thuật toán điều chế độ rộng xung (PWM) 4 cấp phân loại chất lượng không khí. Cấu trúc phần cứng cốt lõi gồm:",
        style_body
    ))
    story.append(Paragraph("• <b>Bộ vi điều khiển:</b> Arduino Uno R3 (ATmega328P, 16 MHz), chịu trách nhiệm thu thập tín hiệu ADC, tính toán bù trừ phi tuyến, điều khiển LCD 1602 qua giao thức I²C và xuất xung PWM điều tốc quạt.", style_bullet))
    story.append(Paragraph("• <b>Khối cảm biến khí MQ-135:</b> Cảm biến bán dẫn oxit kim loại (SnO₂), ngõ ra điện áp Analog đưa về chân A0 của Arduino.", style_bullet))
    story.append(Paragraph("• <b>Khối cảm biến nhiệt ẩm DHT11:</b> Đọc tín hiệu số đơn tuyến (1-wire digital) tại chân D2 để cung cấp thông số nhiệt độ (°C) và độ ẩm (%RH) phục vụ thuật toán bù nhiệt ẩm thời gian thực.", style_bullet))
    story.append(Paragraph("• <b>Khối chấp hành công suất (MOSFET D4184):</b> Nhận xung PWM 490 Hz từ chân D9 qua cổng kích logic để đóng cắt tải quạt 5V DC một cách êm ái mà không gây sụt áp nguồn vi xử lý.", style_bullet))
    story.append(Paragraph("• <b>Khối giao tiếp & Hiển thị:</b> Màn hình LCD 1602 kèm chip mở rộng PCF8574 (địa chỉ I²C 0x27 hoặc 0x3F) và cổng Serial UART (9600 baud) truyền dữ liệu 6 kênh đồng bộ lên Arduino Serial Plotter.", style_bullet))

    story.append(Spacer(1, 10))
    story.append(Paragraph("Bảng 1: Sơ đồ phân bổ chân phần cứng và chức năng tín hiệu", style_body_bold))
    pin_table_data = [
        [Paragraph("<b>Mô-đun</b>", style_body), Paragraph("<b>Chân trên linh kiện</b>", style_body), Paragraph("<b>Chân Arduino Uno</b>", style_body), Paragraph("<b>Dạng tín hiệu / Vai trò công nghệ</b>", style_body)],
        [Paragraph("MQ-135", style_body), Paragraph("VCC, GND, AO", style_body), Paragraph("5V, GND, A0", style_body), Paragraph("Analog 0 - 5V, đo điện áp cầu phân áp Rs/(Rs+RL)", style_body)],
        [Paragraph("DHT11", style_body), Paragraph("VCC, GND, DATA", style_body), Paragraph("5V, GND, D2", style_body), Paragraph("Digital 1-Wire, cập nhật chu kỳ 2.0 s (nhiệt ẩm bù sai số)", style_body)],
        [Paragraph("LCD 1602 I²C", style_body), Paragraph("VCC, GND, SDA, SCL", style_body), Paragraph("5V, GND, A4 (SDA), A5 (SCL)", style_body), Paragraph("Giao tiếp I²C 100 kHz, địa chỉ tự nhận 0x27 / 0x3F", style_body)],
        [Paragraph("Mạch MOSFET D4184", style_body), Paragraph("PWM+, PWM-, VIN, VOUT", style_body), Paragraph("D9, GND, 5V, Cực dương Quạt", style_body), Paragraph("PWM 490 Hz (0-255), điều tốc quạt thông gió 4 cấp", style_body)]
    ]
    pin_table = Table(pin_table_data, colWidths=[2.8*cm, 3.6*cm, 4.0*cm, 6.6*cm])
    pin_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), colors.HexColor('#e2e8f0')),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor('#94a3b8')),
        ('INNERGRID', (0,0), (-1,-1), 0.5, colors.HexColor('#cbd5e1')),
        ('TOPPADDING', (0,0), (-1,-1), 5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 5),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(pin_table)

    # ==========================================
    # PHẦN 2: PHƯƠNG PHÁP HIỆU CHUẨN (CALIBRATION)
    # ==========================================
    story.append(Spacer(1, 14))
    story.append(Paragraph("2. PHƯƠNG PHÁP HIỆU CHUẨN CẢM BIẾN (CALIBRATION METHODOLOGY)", style_h1))
    story.append(Paragraph(
        "Cảm biến khí oxit bán dẫn MQ-135 không thể tự động cho ra giá trị ppm tuyệt đối nếu không được hiệu chuẩn điểm mốc tham chiếu ($R_0$). Giá trị điện trở bề mặt $R_s$ của màng nhạy khí thay đổi tỷ lệ nghịch với nồng độ khí theo hàm mũ. Quy trình hiệu chuẩn thực nghiệm được tiến hành nghiêm ngặt qua 3 giai đoạn:",
        style_body
    ))
    
    story.append(Paragraph("2.1. Quá trình già hóa nhiệt (Burn-in / Pre-heating)", style_h2))
    story.append(Paragraph(
        "Màng SnO₂ cần được đốt nóng liên tục ở nhiệt độ khoảng 300°C ÷ 350°C bằng dây sấy nội tại (chân sấy H-H ăn dòng khoảng 150 mA). Để bề mặt cảm biến đạt trạng thái cân bằng động hóa học, khử sạch hơi ẩm và tạp chất bám dính trong quá trình lưu kho, cảm biến bắt buộc phải được cắm điện chạy sấy ổn định liên tục ít nhất <b>24 giờ đến 48 giờ</b> trước khi lấy số liệu hiệu chuẩn.",
        style_body
    ))

    story.append(Paragraph("2.2. Đo thực nghiệm ngoài trời xác lập mốc tham chiếu sạch ($R_{s,\\text{clean}}$)", style_h2))
    story.append(Paragraph(
        "Vì môi trường phòng kín luôn chứa khí thở của người, CO₂ tích tụ và các hợp chất hữu cơ dễ bay hơi (VOCs), nhóm tác giả đã mang toàn bộ hệ thống ra <b>vườn cây thoáng đãng vào sáng sớm</b> (nơi nồng độ CO₂ tiệm cận mốc khí quyển chuẩn toàn cầu 400.0 ppm). Tiến hành đo đạc độc lập 6 lần tại các thời điểm khác nhau, ghi nhận giá trị điện trở cảm biến trung bình:",
        style_body
    ))
    story.append(Paragraph("• Điện trở tải mạch đo: $R_L = 10.0\\text{ k}\\Omega$ (điện trở dán chuẩn trên module).", style_bullet))
    story.append(Paragraph("• Giá trị $R_s$ đo được ngoài trời: $R_{s,\\text{clean}} = 51.30\\text{ k}\\Omega$ (trung bình cộng 6 lần đo).", style_bullet))
    story.append(Paragraph("• Nhiệt độ môi trường lúc calib: $T_{\\text{cal}} = 31.4^\\circ\\text{C}$; Độ ẩm lúc calib: $H_{\\text{cal}} = 62.4\\%\\text{RH}$.", style_bullet))

    story.append(Paragraph("2.3. Quy đổi điện trở chuẩn $R_0$ và thuật toán tự thích ứng ($k_{\\text{EnvAdapt}}$)", style_h2))
    story.append(Paragraph(
        "Điện trở chuẩn $R_0$ là điện trở màng cảm biến trong không khí sạch chuẩn (tại 20°C, 33% RH). Sử dụng hàm thực nghiệm bù trừ nhiệt ẩm Hanwei, hệ số hiệu chỉnh tại thời điểm ngoài trời được tính toán:",
        style_body
    ))
    story.append(Paragraph(
        "$$\\text{cor}_{\\text{cal}} = \\left(0.00035 \\cdot T_{\\text{cal}}^2 - 0.02718 \\cdot T_{\\text{cal}} + 1.39538\\right) - \\left(H_{\\text{cal}} - 33.0\\right) \\cdot 0.0018 = 0.8341$$",
        style_body
    ))
    story.append(Paragraph(
        "Từ đó, điện trở chuẩn $R_0$ gốc của cảm biến được xác định chính xác và nạp cố định vào mã nguồn:",
        style_body
    ))
    story.append(Paragraph(
        "$$R_{0,\\text{base}} = \\frac{R_{s,\\text{clean}}}{\\text{cor}_{\\text{cal}}} = \\frac{51.30\\text{ k}\\Omega}{0.8341} = 61.50\\text{ k}\\Omega$$",
        style_body
    ))
    story.append(Paragraph(
        "<b>Thuật toán neo mốc phòng ($k_{\\text{EnvAdapt}}$):</b> Khi khởi động trong nhà, trong 2 giây đầu tiên của hàm <code>setup()</code>, vi xử lý đo 20 mẫu điện trở nền phòng ($R_{s,\\text{room}}$). Tỷ số $k_{\\text{EnvAdapt}} = R_{s,\\text{clean}} / R_{s,\\text{room}}$ (dao động từ 0.57 đến 1.74) đóng vai trò hệ số thích ứng, đảm bảo chuyển tiếp dữ liệu mượt mà từ môi trường calib ngoài vườn vào không gian làm việc thực tế mà không gây sốc nồng độ ảo.",
        style_body
    ))

    # ==========================================
    # PHẦN 3: KIỂM SOÁT TÍN HIỆU & KHỬ NHIỄU TOÀN DIỆN
    # ==========================================
    story.append(Spacer(1, 14))
    story.append(Paragraph("3. KIỂM SOÁT SAI SỐ VÀ CÁC GIẢI PHÁP XỬ LÝ TÍN HIỆU TOÀN DIỆN", style_h1))
    story.append(Paragraph(
        "Để đảm bảo hệ thống đo lường ổn định, tin cậy và không bao giờ gặp sự cố đơ đứng số liệu trong quá trình hoạt động, nhóm tác giả đã thiết kế và lập trình 5 tầng kiểm soát tín hiệu đồng bộ:",
        style_body
    ))

    story.append(Paragraph("3.1. Kỹ thuật đo tỷ lệ (Ratiometric Measurement) triệt tiêu sụt áp nguồn", style_h2))
    story.append(Paragraph(
        "Khi cắm Arduino qua cổng USB máy tính, điện áp 5V cấp thường xuyên bị trồi sụt từ 4.75V đến 5.10V tùy thuộc vào tải của quạt và cổng kết nối. Mạch phân áp MQ-135 gồm cảm biến $R_s$ mắc nối tiếp điện trở tải $R_L$ nối vào nguồn $V_c$. Điện áp ngõ ra $V_{\\text{out}} = V_c \\cdot R_L / (R_s + R_L)$. Đồng thời bộ chuyển đổi ADC 10-bit dùng chính nguồn $V_c$ làm điện áp tham chiếu $V_{\\text{ref}}$:",
        style_body
    ))
    story.append(Paragraph(
        "$$\\text{ADC} = \\frac{V_{\\text{out}}}{V_c} \\cdot 1023 = \\frac{R_L}{R_s + R_L} \\cdot 1023 \\implies R_s = R_L \\cdot \\left(\\frac{1023}{\\text{ADC}} - 1\\right)$$",
        style_body
    ))
    story.append(Paragraph(
        "Điện áp nguồn $V_c$ hoàn toàn bị triệt tiêu khỏi phương trình tính toán. Do đó, sự dao động điện áp nguồn USB hoàn toàn không gây sai số cho phép đo điện trở $R_s$.",
        style_body
    ))

    story.append(Paragraph("3.2. Thuật toán trích mẫu tăng cường (Oversampling 20 mẫu ADC)", style_h2))
    story.append(Paragraph(
        "Bộ chuyển đổi ADC của vi điều khiển ATmega328P vốn có độ phân giải 10-bit (1024 mức, bước nhảy $\\Delta V = 5.0 / 1023 \\approx 4.89\\text{ mV}$). Kỹ thuật Oversampling lấy 20 mẫu liên tiếp, mỗi mẫu cách nhau $400\\ \\mu\\text{s}$ (tổng thời gian quét $8\\text{ ms}$ cho một chu kỳ đọc):",
        style_body
    ))
    story.append(Paragraph("• Tăng độ phân giải hiệu dụng từ 10-bit lên tương đương gần 12-bit, giúp ghi nhận các dao động vi mô cấp micro-volt ($< 1\\text{ mV}$).", style_bullet))
    story.append(Paragraph("• Cửa sổ lấy mẫu 8 ms nằm trọn trong chu kỳ bán sóng của lưới điện 50 Hz (10 ms), có tác dụng triệt tiêu nhiễu hài xoay chiều công nghiệp cảm ứng vào dây dẫn.", style_bullet))
    story.append(Paragraph("• Triệt tiêu hoàn toàn hiện tượng 'đơ số' (giá trị nồng độ bị giữ nguyên hàng chục giây), giúp nồng độ CO₂ trên đồ thị Serial Plotter luôn uốn lượn liên tục theo đúng động học tự nhiên của không khí.", style_bullet))

    story.append(Paragraph("3.3. Mô hình bù phi tuyến nhiệt ẩm Hanwei", style_h2))
    story.append(Paragraph(
        "Màng nhạy khí oxit kim loại rất nhạy cảm với độ ẩm và nhiệt độ không khí do các phân tử nước tranh chấp vị trí hấp phụ oxy trên bề mặt tinh thể. Hệ thống sử dụng đa thức Hanwei cập nhật liên tục từ cảm biến DHT11:",
        style_body
    ))
    story.append(Paragraph(
        "$$\\text{cor}(T, H) = \\left(0.00035 \\cdot T^2 - 0.02718 \\cdot T + 1.39538\\right) - \\left(H - 33.0\\right) \\cdot 0.0018$$",
        style_body
    ))
    story.append(Paragraph(
        "Giá trị điện trở $R_s$ đo được chia cho $\\text{cor}(T, H)$ để đưa về trạng thái quy chuẩn, giới hạn trong khoảng $[0.35;\\ 2.50]$ nhằm loại bỏ các xung lỗi đột biến.",
        style_body
    ))

    story.append(Paragraph("3.4. Vùng trễ Schmitt Trigger 25 ppm chống rung giật quạt", style_h2))
    story.append(Paragraph(
        "Khi nồng độ CO₂ dao động sát ngưỡng chuyển cấp (ví dụ quanh 650, 950 hoặc 1300 ppm), việc đóng ngắt tức thì sẽ làm quạt quay - dừng liên tục gây rung giật cơ khí và phát ra tiếng ồn khó chịu. Thuật toán Schmitt Trigger tạo một vùng trễ 25 ppm:",
        style_body
    ))
    story.append(Paragraph("• Quạt chỉ tăng cấp khi nồng độ vượt ngưỡng trên (ví dụ $PPM \\ge 950$).", style_bullet))
    story.append(Paragraph("• Quạt chỉ hạ cấp khi nồng độ giảm sâu xuống dưới ngưỡng trừ đi độ trễ (ví dụ $PPM < 950 - 25 = 925\\text{ ppm}$).", style_bullet))

    story.append(Paragraph("3.5. Thuật toán Soft-ramp (+14 / -6) điều tốc quạt êm dịu", style_h2))
    story.append(Paragraph(
        "Khi chuyển từ trạng thái nghỉ (0 PWM) lên nguy cấp (255 PWM), dòng khởi động của động cơ quạt có thể vọt lên gấp 3 đến 5 lần dòng định mức, gây sụt áp đường cấp 5V và làm vi điều khiển bị Reset đột ngột. Hàm <code>setFanSoftPWM()</code> điều khiển tăng dần với bước $+14\\text{ PWM}/100\\text{ ms}$ (mất khoảng 1.8 giây để đạt tốc độ tối đa) và giảm êm với bước $-6\\text{ PWM}/100\\text{ ms}$, triệt tiêu hoàn toàn sốc điện áp.",
        style_body
    ))

    # ==========================================
    # PHẦN 4: GIẢI THÍCH SƠ ĐỒ THUẬT TOÁN & MÃ NGUỒN CODE
    # ==========================================
    story.append(Spacer(1, 14))
    story.append(Paragraph("4. GIẢI THÍCH KIẾN TRÚC MÃ NGUỒN ARDUINO VÀ LƯU ĐỒ THỰC THI", style_h1))
    story.append(Paragraph(
        "Mã nguồn được cấu trúc theo mô hình máy trạng thái thời gian thực không sử dụng lệnh <code>delay()</code> lớn gây khóa cứng vi xử lý. Chu kỳ quét chính là 100 ms (~10 chu kỳ/giây), phân tách nhiệm vụ rõ ràng:",
        style_body
    ))

    code_steps = [
        ("1. Khởi tạo setup():", "Khởi động giao tiếp Serial 9600 baud, cài đặt chân I/O, kích hoạt chống treo bus I²C (<code>setWireTimeout(25000, true)</code>), tự động quét địa chỉ LCD (0x27 hoặc 0x3F). Tính toán $R_0 = 61.50\\text{ k}\\Omega$ và lấy mẫu 20 lần tính hệ số thích ứng $k_{\\text{EnvAdapt}}$. Cuối cùng, chạy test nhẹ quat 0.8 giây để thông báo phần cứng sẵn sàng."),
        ("2. Định thời đọc DHT11 (2.0 giây):", "Cảm biến DHT11 có quán tính nhiệt ẩm lớn, chỉ cho phép đọc mỗi 2.0 giây một lần. Sử dụng biến đếm thời gian <code>millis()</code> để đọc không nghẽn. Nếu dữ liệu hợp lệ, đảo trạng thái <code>dhtHeartbeat</code> để hiển thị dấu '*' sống trên LCD; nếu lỗi quá 2 chu kỳ liên tiếp, kích hoạt cờ cảnh báo lỗi cảm biến."),
        ("3. Trích xuất ADC và tính CO₂:", "Gọi hàm <code>readOversampledADC()</code> đọc 20 mẫu analog chân A0. Tính $R_s$, nhân hệ số thích ứng $k_{\\text{EnvAdapt}}$, chia hệ số bù nhiệt ẩm $\\text{cor}$. Nồng độ CO₂ được tính theo phương trình đặc tuyến Winsen: $PPM = 400.0 \\cdot (R_{s,\\text{comp}} / R_{0,\\text{base}})^{-2.76903}$."),
        ("4. Phân cấp chất lượng không khí:", "Phân loại vào 4 cấp độ: TỐT (< 650 ppm, quạt tắt), TRUNG BÌNH (650 - 950 ppm, quạt chạy 35% - 55%), CAO (950 - 1300 ppm, quạt chạy 60% - 80%), NGUY CẤP (≥ 1300 ppm, quạt chạy 100% full tải 255 PWM). Áp dụng vùng trễ 25 ppm."),
        ("5. Xuất dữ liệu đồng bộ Serial Plotter & LCD:", "Định dạng xuất chuẩn 6 kênh tín hiệu theo cú pháp nhãn: <code>CO2_ppm</code>, <code>Quat_PWM</code>, <code>ADC_Raw</code>, <code>NhietDo_C</code>, <code>DoAm_pct</code>, <code>DHT_Live</code>. Màn hình LCD 1602 cập nhật đệm chuỗi ký tự cố định 16 ô tránh nhấp nháy.")
    ]

    for title_c, desc_c in code_steps:
        story.append(Paragraph(f"<b>{title_c}</b> {desc_c}", style_body))

    # ==========================================
    # PHẦN 5: GIỚI HẠN VẬT LÝ CỦA CẢM BIẾN MQ-135 VÀ NỒNG ĐỘ CAO
    # ==========================================
    story.append(Spacer(1, 14))
    story.append(Paragraph("5. ĐÁNH GIÁ GIỚI HẠN VẬT LÝ CỦA THIẾT BỊ: VÌ SAO MQ-135 KHÔNG ĐO CHÍNH XÁC ĐƯỢC CO₂ Ở NỒNG ĐỘ LỚN?", style_h1))
    story.append(Paragraph(
        "Một trong những phát hiện khoa học quan trọng nhất trong quá trình thực nghiệm là: <b>Cảm biến MQ-135 hoạt động rất nhạy và tin cậy trong dải giám sát chất lượng không khí dân dụng (400 ÷ 1500 ppm), nhưng khi nồng độ CO₂ tăng cao vượt quá 2000 ÷ 3000 ppm (hoặc thổi khí thở đậm đặc), kết quả đo sẽ mất dần độ chính xác và xuất hiện sai số phi tuyến rất lớn.</b> Các nguyên nhân vật lý cốt lõi bao gồm:",
        style_body
    ))

    limits = [
        ("5.1. Giới hạn bão hòa bề mặt hấp phụ (Langmuir Adsorption Saturation)", 
         "Cảm biến MQ-135 hoạt động dựa trên cơ chế bề mặt: Khi nung nóng, các phân tử oxy trong không khí bám vào bề mặt các hạt nano SnO₂ và bẫy electron tự do, tạo ra lớp rào thế cản trở dòng điện. Khí mục tiêu (khí khử) tràn vào sẽ phản ứng với oxy hấp phụ, giải phóng electron trả lại cho vùng dẫn, làm giảm điện trở $R_s$. "
         "Tuy nhiên, số lượng tâm hấp phụ hoạt tính trên màng SnO₂ là có hạn. Ở nồng độ thấp (400 - 1000 ppm), mỗi phân tử CO₂ tăng lên đều tìm được vị trí phản ứng, điện trở thay đổi rõ rệt. Khi nồng độ tăng cao vượt mức 2500 - 3000 ppm, hầu hết các vị trí liên kết oxy đều đã bị chiếm đóng hoặc phản ứng cạn kiệt. Hiện tượng <b>bão hòa hóa học bề mặt</b> xảy ra khiến điện trở $R_s$ tiệm cận một giá trị tiệm cận đáy tối thiểu, dù nồng độ CO₂ có tăng gấp đôi thì điện áp ra cũng chỉ tăng thêm vài milivolt."),
        
        ("5.2. Độ nhạy và độ dốc đặc tuyến suy giảm theo hàm mũ nghịch đảo", 
         "Đặc tuyến chuyển đổi nồng độ của MQ-135 tuân theo hàm lũy thừa: $PPM = A \\cdot (R_s/R_0)^{-2.769}$. "
         "Đạo hàm độ nhạy suy giảm nghiêm trọng ở dải cao: $\\frac{d(PPM)}{d(R_s)} \\propto -2.769 \\cdot R_s^{-3.769}$. "
         "Ở nồng độ thấp (400 - 600 ppm), một bước nhảy ADC (4.89 mV) chỉ tương đương khoảng 5 - 10 ppm CO₂. Ngược lại, ở nồng độ trên 3000 ppm, đặc tuyến nằm gần như nằm ngang; khi đó một sai số nhỏ chỉ 1 bước ADC (do nhiễu nguồn hoặc nhiễu lượng tử hóa) cũng bị khuếch đại lên thành sai lệch hàng trăm ppm!"),
        
        ("5.3. Độ chọn lọc kém và hiện tượng phản ứng chéo (Cross-Sensitivity)", 
         "MQ-135 bản chất là cảm biến chất lượng không khí tổng hợp nhạy với nhiều loại khí khử khác nhau (CO, NH₃, Cồn/Ethanol, Benzene, khói). Mức độ nhạy của SnO₂ với cồn và khí NH₃ thậm chí còn cao hơn so với CO₂. "
         "Trong thí nghiệm thổi hơi thở của người: Hơi thở không chỉ chứa khí CO₂ (~40,000 ppm) mà còn chứa hơi nước bão hòa (100% RH) và các hợp chất hữu cơ chuyển hóa từ miệng. Dù thuật toán Hanwei đã bù trừ độ ẩm, nhưng sự phối hợp giữa độ ẩm cực cao và nồng độ khí lớn lập tức đẩy màng oxit kim loại vào trạng thái quá tải hóa học."),
        
        ("5.4. So sánh với cảm biến quang học NDIR (Non-Dispersive Infrared)", 
         "Để đo chính xác CO₂ ở nồng độ cao (lên tới 5000 hoặc 10,000 ppm), tiêu chuẩn công nghiệp bắt buộc phải sử dụng cảm biến quang phổ hồng ngoại NDIR (như SenseAir S8, MH-Z19C). Cảm biến NDIR đo độ suy giảm bức xạ hồng ngoại tại bước sóng hấp thụ đặc trưng của phân tử CO₂ ($\lambda = 4.26\\ \\mu\\text{m}$), hoạt động dựa trên định luật Beer-Lambert nên có tính chọn lọc tuyệt đối, không bị ảnh hưởng bởi các khí khác và không bị bão hòa bề mặt. "
         "Cảm biến MQ-135 là giải pháp tối ưu về mặt kinh tế (giá thành rẻ gấp 10 lần NDIR), rất phù hợp cho bài toán <b>phát hiện xu hướng tăng nồng độ và phân cấp cảnh báo nguy cấp</b>, nhưng không thể dùng làm thiết bị định lượng tuyệt đối ở dải nồng độ cao.")
    ]

    for title_l, desc_l in limits:
        story.append(Paragraph(title_l, style_h2))
        story.append(Paragraph(desc_l, style_body))

    # Hộp ghi chú cảnh báo nổi bật
    warning_data = [[
        Paragraph(
            "<b>KẾT LUẬN THỰC NGHIỆM VỀ GIỚI HẠN THIẾT BỊ:</b><br/>"
            "MQ-135 đóng vai trò hoàn hảo như một 'cảm biến chỉ thị ngưỡng và xu hướng động' (Trend & Threshold Indicator). "
            "Khi nồng độ trong phòng tăng từ 500 ppm lên 1000 ppm, hệ thống nhận biết chính xác từng nấc tăng để điều tốc quạt tương ứng. "
            "Tuy nhiên, khi có sự cố xả khí đậm đặc hoặc thổi hơi thở trực tiếp, hệ thống xác định đúng trạng thái 'NGUY CẤP' để quạt chạy 100% công suất, "
            "song giá trị số tuyệt đối hiển thị (ví dụ 3500 ppm hay 4500 ppm) chỉ mang ý nghĩa tương đối do hiện tượng bão hòa màng SnO₂.",
            style_callout
        )
    ]]
    warning_box = Table(warning_data, colWidths=[17.0*cm])
    warning_box.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor('#fef2f2')),
        ('BOX', (0,0), (-1,-1), 1.5, colors.HexColor('#ef4444')),
        ('TOPPADDING', (0,0), (-1,-1), 8),
        ('BOTTOMPADDING', (0,0), (-1,-1), 8),
        ('LEFTPADDING', (0,0), (-1,-1), 12),
        ('RIGHTPADDING', (0,0), (-1,-1), 12),
    ]))
    story.append(Spacer(1, 6))
    story.append(warning_box)

    # ==========================================
    # PHẦN 6: KẾT LUẬN & HƯỚNG PHÁT TRIỂN
    # ==========================================
    story.append(Spacer(1, 14))
    story.append(Paragraph("6. TỔNG KẾT VÀ HƯỚNG PHÁT TRIỂN ĐỀ TÀI", style_h1))
    story.append(Paragraph("• <b>Kết quả đạt được:</b> Xây dựng hoàn chỉnh hệ thống quan trắc chất lượng không khí với độ ổn định cao; thuật toán hiệu chuẩn 6 lần ngoài trời xác lập mốc $R_0 = 61.50\\text{ k}\\Omega$ vững chắc; kỹ thuật Oversampling 20 mẫu và đo tỷ lệ triệt tiêu hoàn toàn hiện tượng treo đơ số và sụt áp nguồn; quạt thông gió 4 cấp PWM vận hành êm ái nhờ Soft-ramp và trễ Schmitt Trigger.", style_bullet))
    story.append(Paragraph("• <b>Độ không đảm bảo đo mở rộng:</b> Đạt $U \\approx \\pm 6.1\\%$ tại điểm kiểm tra 1000 ppm trong dải phòng kín dân dụng (400 ÷ 1300 ppm), đáp ứng đầy đủ tiêu chuẩn đánh giá IAQ.", style_bullet))
    story.append(Paragraph("• <b>Hướng phát triển:</b> Nâng cấp tích hợp cảm biến quang học NDIR (SenseAir S8) kết hợp thuật toán lọc Kalman mở rộng để kết hợp ưu điểm giá thành rẻ của MQ-135 và độ chính xác tuyệt đối dải cao của NDIR.", style_bullet))

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"SUCCESS: Generated comprehensive PDF at: {OUTPUT_PDF}")

if __name__ == '__main__':
    build_pdf()
