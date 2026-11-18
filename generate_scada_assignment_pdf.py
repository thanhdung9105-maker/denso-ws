import os
import sys
import shutil
from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, KeepTogether
)
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

# 1. Đăng ký Font tiếng Việt có dấu
pdfmetrics.registerFont(TTFont('Arial', 'C:/Windows/Fonts/arial.ttf'))
pdfmetrics.registerFont(TTFont('Arial-Bold', 'C:/Windows/Fonts/arialbd.ttf'))
pdfmetrics.registerFont(TTFont('Arial-Italic', 'C:/Windows/Fonts/ariali.ttf'))
pdfmetrics.registerFont(TTFont('Arial-BoldItalic', 'C:/Windows/Fonts/arialbi.ttf'))

# 2. Numbered Canvas vẽ Header & Footer tự động
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
        self.saveState()
        self.setFont("Arial", 8)
        self.setFillColor(colors.HexColor("#64748B"))
        
        # Header (từ trang 2 trở đi)
        if self._pageNumber > 1:
            self.drawString(13 * mm, 286 * mm, "KẾ HOẠCH BTL SCADA • CHẠY MÔ PHỎNG & QUAY VIDEO THỰC TẾ (NHÓM 5 SINH VIÊN)")
            self.drawRightString(197 * mm, 286 * mm, "TIA Portal (LAD & SCL) | Factory I/O | WinCC | SQL Server")
            self.setStrokeColor(colors.HexColor("#CBD5E1"))
            self.setLineWidth(0.5)
            self.line(13 * mm, 283 * mm, 197 * mm, 283 * mm)
            
        # Footer (tất cả các trang)
        page_str = f"Trang {self._pageNumber} / {page_count}"
        self.drawRightString(197 * mm, 9 * mm, page_str)
        self.drawString(13 * mm, 9 * mm, "BTL SCADA Thực Hành • Tập trung 100% Chạy Phần Mềm & Quay Video Minh Chứng Hoàn Chỉnh")
        self.setStrokeColor(colors.HexColor("#CBD5E1"))
        self.setLineWidth(0.5)
        self.line(13 * mm, 12 * mm, 197 * mm, 12 * mm)
        
        self.restoreState()

def build_pdf(output_path):
    doc = SimpleDocTemplate(
        output_path,
        pagesize=A4,
        leftMargin=13 * mm,
        rightMargin=13 * mm,
        topMargin=13 * mm,
        bottomMargin=14 * mm
    )

    PRIMARY = colors.HexColor("#0B2545")     # Deep Navy
    SECONDARY = colors.HexColor("#134074")   # Industrial Blue
    DARK_TEXT = colors.HexColor("#1E293B")   # Slate Charcoal
    LIGHT_BG = colors.HexColor("#F8FAFC")    # Cool white
    BORDER_COLOR = colors.HexColor("#CBD5E1")# Subtle border

    styles = getSampleStyleSheet()
    
    title_style = ParagraphStyle(
        'DocTitle',
        fontName='Arial-Bold',
        fontSize=13.5,
        leading=17,
        textColor=PRIMARY,
        alignment=1,
        spaceAfter=2
    )
    
    subtitle_style = ParagraphStyle(
        'DocSubtitle',
        fontName='Arial-Bold',
        fontSize=10,
        leading=13,
        textColor=SECONDARY,
        alignment=1,
        spaceAfter=2
    )
    
    desc_style = ParagraphStyle(
        'DocDesc',
        fontName='Arial',
        fontSize=7.8,
        leading=10.5,
        textColor=colors.HexColor("#475569"),
        alignment=1,
        spaceAfter=5
    )

    h1_style = ParagraphStyle(
        'Heading1_Custom',
        fontName='Arial-Bold',
        fontSize=9,
        leading=12,
        textColor=PRIMARY,
        spaceBefore=4,
        spaceAfter=3,
        keepWithNext=True
    )

    body_style = ParagraphStyle(
        'Body_Custom',
        fontName='Arial',
        fontSize=7.4,
        leading=10.4,
        textColor=DARK_TEXT,
        spaceAfter=2
    )

    table_header = ParagraphStyle(
        'TableHeader',
        fontName='Arial-Bold',
        fontSize=7.2,
        leading=10,
        textColor=colors.white,
        alignment=1
    )

    table_cell = ParagraphStyle(
        'TableCell',
        fontName='Arial',
        fontSize=7,
        leading=9.5,
        textColor=DARK_TEXT
    )

    table_cell_bold = ParagraphStyle(
        'TableCellBold',
        fontName='Arial-Bold',
        fontSize=7,
        leading=9.5,
        textColor=DARK_TEXT
    )

    table_cell_center = ParagraphStyle(
        'TableCellCenter',
        fontName='Arial',
        fontSize=7,
        leading=9.5,
        textColor=DARK_TEXT,
        alignment=1
    )

    code_style = ParagraphStyle(
        'CodeStyle',
        fontName='Arial',
        fontSize=6.5,
        leading=8.8,
        textColor=colors.HexColor("#0F172A")
    )

    story = []

    # ========================== TRANG 1 ==========================
    story.append(Paragraph("KẾ HOẠCH BÀI TẬP LỚN SCADA • NHÓM 5 SINH VIÊN", title_style))
    story.append(Paragraph("HỆ THỐNG PHÂN LOẠI & ĐÓNG GÓI SẢN PHẨM THEO CHIỀU CAO", subtitle_style))
    story.append(Paragraph("Tập trung 100% Kỹ thuật Thực hành • <b>TIA Portal (LAD & SCL)</b> | <b>Factory I/O 3D</b> | <b>WinCC Runtime</b> | <b>SQL Server</b> | <b>Quay Video Demo</b>", desc_style))

    story.append(Paragraph("1. QUY TRÌNH HOẠT ĐỘNG THỰC TẾ & KIẾN TRÚC KẾT NỐI LIÊN THÔNG", h1_style))
    story.append(Paragraph(
        "• <b>Mục tiêu trọng tâm:</b> Không làm báo cáo Word hay Slide lý thuyết, nhóm tập trung toàn lực vào <b>xây dựng phần mềm chạy thực tế mượt mà 100%</b> và <b>quay video clip minh chứng</b> đạt chuẩn nộp bài.<br/>"
        "• <b>Quy trình vận hành 3 bước cần quay video:</b><br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;1. <i>Vận hành Manual:</i> Nhấn chuyển công tắc sang Manual, bấm các nút test tay trên WinCC để kiểm tra độc lập băng tải chạy và 2 xy-lanh khí nén tác động.<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;2. <i>Vận hành Auto & Phân loại:</i> Bật công tắc Auto, nhấn START; Băng tải (%Q0.0) đưa phôi qua cảm biến quang; Nếu cả I0.3 (Cao) và I0.4 (Thấp) báo ➔ Xy-lanh 1 (%Q0.1) đẩy vào Máng A; Nếu chỉ I0.4 báo ➔ Xy-lanh 2 (%Q0.2) đẩy vào Máng B; Phôi lỗi đi thẳng vào thùng phế phẩm.<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;3. <i>Đóng thùng đủ 5 SP & Tự động lưu SQL:</i> Cảm biến đếm từng phôi rơi vào thùng; khi đủ 5 SP/thùng ➔ tăng số thùng <code>Box_Total</code> lên 1, kích cờ <code>Trig_SQL</code> để WinCC tự lưu bản ghi vào SQL Server và bấm nút xuất file Excel (.CSV).",
        body_style
    ))

    story.append(Spacer(1, 2))
    story.append(Paragraph("2. BẢNG PHÂN CÔNG KỸ THUẬT THỰC HÀNH 5 THÀNH VIÊN (20% ĐÓNG GÓP / NGƯỜI)", h1_style))

    members_data = [
        [
            Paragraph("<b>Thành Viên / Vai Trò</b>", table_header),
            Paragraph("<b>Họ Tên & MSSV</b>", table_header),
            Paragraph("<b>Nhiệm Vụ Kỹ Thuật Phụ Trách Chính (100% Thực Hành)</b>", table_header),
            Paragraph("<b>Sản Phẩm Kỹ Thuật Bàn Giao</b>", table_header),
            Paragraph("<b>Tỷ Lệ</b>", table_header)
        ],
        [
            Paragraph("<b>Thành viên 1<br/>(Nhóm trưởng)</b><br/><font color='#134074'>Lập trình PLC LAD & Mạch Động Lực</font>", table_cell_bold),
            Paragraph("[Họ tên SV 1]<br/>MSSV: [...........]", table_cell_center),
            Paragraph("• Khởi tạo Project trên TIA Portal, cấu hình phần cứng PLC S7-1200 CPU 1214C DC/DC/DC.<br/>"
                      "• Lập trình khối <b>OB1</b> và <b>FC1_Control_Conveyor</b> (LAD): Mạch tự giữ Start/Stop (kết hợp nút vật lý và nút HMI), chuyển mạch chế độ Auto/Manual.<br/>"
                      "• Điều khiển động cơ băng tải chính (%Q0.0), định thời trễ TON kích mở van điện từ 2 xy-lanh (%Q0.1, %Q0.2) và đèn báo trạng thái Run/Error.", table_cell),
            Paragraph("• Cấu hình phần cứng TIA Portal<br/>• Khối OB1 & FC1 (ngôn ngữ LAD hoàn chỉnh)", table_cell),
            Paragraph("<b>20%</b>", table_cell_center)
        ],
        [
            Paragraph("<b>Thành viên 2</b><br/><font color='#134074'>Lập trình PLC SCL & Thuật toán Đóng Thùng</font>", table_cell_bold),
            Paragraph("[Họ tên SV 2]<br/>MSSV: [...........]", table_cell_center),
            Paragraph("• Tạo khối dữ liệu <b>DB1 (DB_Data)</b> quản lý toàn bộ biến hệ thống (bỏ tích chọn 'Optimized block access' để cố định địa chỉ tuyệt đối).<br/>"
                      "• Lập trình khối <b>FB1_Counter_And_Batch</b> (SCL): Bắt sườn xung cảm biến quang phân loại chiều cao, thuật toán đếm số lượng phôi A và B.<br/>"
                      "• Xử lý logic đóng thùng khi đủ định mức 5 SP/thùng, tăng tổng số thùng <code>Box_Total</code> và kích hoạt cờ tín hiệu <code>Trig_SQL = TRUE</code>.", table_cell),
            Paragraph("• Khối DB1 (DB_Data)<br/>• Khối FB1 (SCL xử lý đếm, đóng thùng & cờ SQL)", table_cell),
            Paragraph("<b>20%</b>", table_cell_center)
        ],
        [
            Paragraph("<b>Thành viên 3</b><br/><font color='#134074'>Mô phỏng 3D Factory I/O & Mapping Driver</font>", table_cell_bold),
            Paragraph("[Họ tên SV 3]<br/>MSSV: [...........]", table_cell_center),
            Paragraph("• Cấu hình mô hình 3D trên <b>Factory I/O</b> (Scene 5: <i>Sorting by Height</i>).<br/>"
                      "• Bố trí bảng tủ điện 3D (nút Start, Stop, Reset, công tắc Auto/Man, đèn báo Run/Err), cụm cảm biến quang nhận diện và cảm biến đếm tại 2 máng xả.<br/>"
                      "• Cấu hình Driver <b>Siemens S7-PLCSIM</b>: Mapping chính xác các cổng ngõ vào %I0.0 – %I0.6 và ngõ ra %Q0.0 – %Q0.4, test vật lý phôi rơi mượt mà.", table_cell),
            Paragraph("• File cảnh 3D (.factoryio)<br/>• Cấu hình Driver S7-PLCSIM hoạt động 100%", table_cell),
            Paragraph("<b>20%</b>", table_cell_center)
        ],
        [
            Paragraph("<b>Thành viên 4</b><br/><font color='#134074'>Thiết Kế Giao Diện SCADA WinCC Runtime</font>", table_cell_bold),
            Paragraph("[Họ tên SV 4]<br/>MSSV: [...........]", table_cell_center),
            Paragraph("• Tạo trạm PC Station (WinCC Runtime Advanced) trong TIA Portal, liên kết HMI Tags với PLC S7-1200.<br/>"
                      "• Thiết kế <b>Main Screen</b>: Bố cục P&ID đồ họa trực quan, cụm nút bấm Start/Stop/Reset, bảng điều khiển tay Manual (hiệu ứng Visibility ẩn/hiện), hiệu ứng chuyển động đổi màu động xy-lanh & băng tải, hiển thị số liệu SP/thùng.<br/>"
                      "• Cấu hình màn hình <b>Trend View</b> (đồ thị sản lượng) & <b>Alarm View</b> (báo sự cố).", table_cell),
            Paragraph("• Trạm WinCC Runtime hoàn chỉnh<br/>• Hệ thống 3 màn hình SCADA vận hành thời gian thực", table_cell),
            Paragraph("<b>20%</b>", table_cell_center)
        ],
        [
            Paragraph("<b>Thành viên 5</b><br/><font color='#134074'>Kỹ sư CSDL SQL, VBScript & Dữ Liệu SCADA</font>", table_cell_bold),
            Paragraph("[Họ tên SV 5]<br/>MSSV: [...........]", table_cell_center),
            Paragraph("• Cài đặt CSDL <code>SCADA_BTL</code> trên <b>SQL Server</b> với 2 bảng quan hệ: <code>Production_Log</code> (lưu thùng hàng) và <code>System_Alarm_Log</code> (lưu nhật ký sự cố dừng máy).<br/>"
                      "• Viết <b>4 kịch bản VBScript chuyên sâu</b> trên WinCC:<br/>"
                      "&nbsp;&nbsp;+ <i>Script 1:</i> Tự động ghi bản ghi đóng thùng khi có cờ <code>Trig_SQL</code>.<br/>"
                      "&nbsp;&nbsp;+ <i>Script 2:</i> Tự động ghi nhật ký lỗi sự cố E-Stop / dừng khẩn vào CSDL SQL.<br/>"
                      "&nbsp;&nbsp;+ <i>Script 3:</i> <b>Truy vấn SQL (SELECT TOP 5) đọc ngược hiển thị lên bảng Table trên WinCC!</b><br/>"
                      "&nbsp;&nbsp;+ <i>Script 4:</i> Xuất báo cáo Excel (.CSV) theo thời gian kèm tiêu đề bảng biểu.<br/>"
                      "• Setup phần mềm quay màn hình chia đôi (OBS), thực hiện kịch bản test và biên tập <b>Video Clip minh chứng HD</b>.", table_cell),
            Paragraph("• CSDL SQL 2 bảng (.sql)<br/>• 4 kịch bản VBScript (.vbs)<br/>• Giao diện đọc SQL ngược lên SCADA<br/>• File Excel mẫu (.csv)<br/>• <b>Video minh chứng MP4</b>", table_cell),
            Paragraph("<b>20%</b>", table_cell_center)
        ]
    ]

    members_table = Table(members_data, colWidths=[34 * mm, 23 * mm, 74 * mm, 39 * mm, 14 * mm])
    members_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, LIGHT_BG]),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(members_table)

    # ========================== TRANG 2 ==========================
    story.append(PageBreak())
    story.append(Paragraph("3. MA TRẬN TRÁCH NHIỆM KỸ THUẬT THỰC HÀNH (RACI MATRIX)", h1_style))
    story.append(Paragraph(
        "<i>Quy ước RACI:</i> <b>R (Responsible)</b>: Trực tiếp cấu hình/lập trình | <b>A (Accountable)</b>: Chịu trách nhiệm kiểm tra kết quả | <b>C (Consulted)</b>: Phối hợp kết nối | <b>I (Informed)</b>: Nhận kết quả để test.",
        body_style
    ))

    raci_data = [
        [
            Paragraph("<b>STT</b>", table_header),
            Paragraph("<b>Hạng Mục Kỹ Thuật Thực Hành Cụ Thể</b>", table_header),
            Paragraph("<b>TV1<br/>(LAD)</b>", table_header),
            Paragraph("<b>TV2<br/>(SCL)</b>", table_header),
            Paragraph("<b>TV3<br/>(3D Sim)</b>", table_header),
            Paragraph("<b>TV4<br/>(WinCC)</b>", table_header),
            Paragraph("<b>TV5<br/>(SQL/Vid)</b>", table_header)
        ],
        [
            Paragraph("1", table_cell_center),
            Paragraph("Tạo dự án TIA Portal, cấu hình phần cứng PLC S7-1200 CPU 1214C", table_cell),
            Paragraph("<b>A / R</b>", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center)
        ],
        [
            Paragraph("2", table_cell_center),
            Paragraph("Tạo bảng PLC Tags và khai báo biến trung gian trong DB1 (DB_Data)", table_cell),
            Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center)
        ],
        [
            Paragraph("3", table_cell_center),
            Paragraph("Viết code LAD (OB1, FC1): Tự giữ Start/Stop, Auto/Manual, Băng tải, Xy-lanh", table_cell),
            Paragraph("<b>A / R</b>", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center)
        ],
        [
            Paragraph("4", table_cell_center),
            Paragraph("Viết code SCL (FB1): Bắt sườn cảm biến, đếm sản phẩm A/B, đóng thùng 5 SP", table_cell),
            Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("C", table_cell_center), Paragraph("I", table_cell_center), Paragraph("C", table_cell_center)
        ],
        [
            Paragraph("5", table_cell_center),
            Paragraph("Thiết lập cảnh 3D Factory I/O (Scene 5 Sorting by Height, tủ điện 3D)", table_cell),
            Paragraph("C", table_cell_center), Paragraph("I", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center)
        ],
        [
            Paragraph("6", table_cell_center),
            Paragraph("Cấu hình Driver S7-PLCSIM trong Factory I/O, gán toàn bộ I/O Tags", table_cell),
            Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center)
        ],
        [
            Paragraph("7", table_cell_center),
            Paragraph("Thiết kế Main Screen trên WinCC Runtime (P&ID động, nút nhấn, đèn báo)", table_cell),
            Paragraph("I", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("C", table_cell_center)
        ],
        [
            Paragraph("8", table_cell_center),
            Paragraph("Cấu hình bảng nút nhấn Manual (Visibility) & Màn hình Trend View, Alarm View", table_cell),
            Paragraph("I", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center), Paragraph("I", table_cell_center)
        ],
        [
            Paragraph("9", table_cell_center),
            Paragraph("Cài đặt SSMS, tạo CSDL SCADA_BTL với 2 bảng: Production_Log & System_Alarm_Log", table_cell),
            Paragraph("I", table_cell_center), Paragraph("I", table_cell_center), Paragraph("I", table_cell_center), Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center)
        ],
        [
            Paragraph("10", table_cell_center),
            Paragraph("Viết VBScripts: Tự lưu thùng hàng, ghi sự cố, truy vấn đọc SQL lên WinCC & Xuất CSV", table_cell),
            Paragraph("I", table_cell_center), Paragraph("C", table_cell_center), Paragraph("I", table_cell_center), Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center)
        ],
        [
            Paragraph("11", table_cell_center),
            Paragraph("Chạy liên thông 4 tầng (Factory I/O ⇄ PLCSIM ⇄ WinCC ⇄ SQL) kiểm tra toàn bộ lỗi", table_cell),
            Paragraph("<b>A</b>", table_cell_center), Paragraph("R", table_cell_center), Paragraph("R", table_cell_center), Paragraph("R", table_cell_center), Paragraph("R", table_cell_center)
        ],
        [
            Paragraph("12", table_cell_center),
            Paragraph("Setup quay màn hình chia đôi, thực hiện kịch bản test và dựng Video Clip hoàn chỉnh", table_cell),
            Paragraph("I", table_cell_center), Paragraph("I", table_cell_center), Paragraph("C", table_cell_center), Paragraph("C", table_cell_center), Paragraph("<b>A / R</b>", table_cell_center)
        ]
    ]

    raci_table = Table(raci_data, colWidths=[10 * mm, 94 * mm, 16 * mm, 16 * mm, 16 * mm, 16 * mm, 16 * mm])
    raci_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, LIGHT_BG]),
        ('TOPPADDING', (0,0), (-1,-1), 1.8),
        ('BOTTOMPADDING', (0,0), (-1,-1), 1.8),
        ('LEFTPADDING', (0,0), (-1,-1), 3),
        ('RIGHTPADDING', (0,0), (-1,-1), 3),
    ]))
    story.append(raci_table)
    story.append(Spacer(1, 4))

    story.append(Paragraph("4. QUY HOẠCH BẢNG BIẾN PLC TAGS & DATA BLOCK DB1 (DB_DATA)", h1_style))
    story.append(Paragraph(
        "Khai báo bảng PLC Tags và khối dữ liệu DB1 (chuột phải DB1 ➔ Properties ➔ Attributes ➔ Bỏ tích 'Optimized block access'):",
        body_style
    ))

    tags_data = [
        [
            Paragraph("<b>Tên Biến (Symbol)</b>", table_header),
            Paragraph("<b>Địa Chỉ</b>", table_header),
            Paragraph("<b>Kiểu Dữ Liệu</b>", table_header),
            Paragraph("<b>Chức Năng Kỹ Thuật & Cổng Kết Nối Hệ Thống</b>", table_header)
        ],
        [
            Paragraph("I_Start / I_Stop", table_cell_bold),
            Paragraph("%I0.0 / %I0.1", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Nút nhấn Start (NO) và Stop (NC) trên bảng tủ điện điều khiển Factory I/O.", table_cell)
        ],
        [
            Paragraph("I_Auto_Man", table_cell_bold),
            Paragraph("%I0.2", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Công tắc chuyển đổi chế độ vận hành (1 = Tự động / Auto, 0 = Bằng tay / Manual).", table_cell)
        ],
        [
            Paragraph("I_Sensor_High / Low", table_cell_bold),
            Paragraph("%I0.3 / %I0.4", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Cụm cảm biến quang nhận diện: Cả 2 cùng tác động = Phôi Cao (A); Chỉ Low = Phôi Thấp (B).", table_cell)
        ],
        [
            Paragraph("I_Count_A / Count_B", table_cell_bold),
            Paragraph("%I0.5 / %I0.6", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Cảm biến quang đếm số lượng sản phẩm rơi vào thùng tại Máng A và Máng B.", table_cell)
        ],
        [
            Paragraph("Q_Conveyor", table_cell_bold),
            Paragraph("%Q0.0", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Ngõ ra cấp nguồn chạy động cơ băng tải chính đưa phôi vào buồng phân loại.", table_cell)
        ],
        [
            Paragraph("Q_Pusher_A / Pusher_B", table_cell_bold),
            Paragraph("%Q0.1 / %Q0.2", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Kích van điện từ đẩy Xy-lanh 1 (đẩy phôi A) và Xy-lanh 2 (đẩy phôi B).", table_cell)
        ],
        [
            Paragraph("Q_Light_Run / Err", table_cell_bold),
            Paragraph("%Q0.3 / %Q0.4", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Đèn báo trạng thái hệ thống: Đang chạy (Xanh lá) và Cảnh báo lỗi dừng (Đỏ).", table_cell)
        ],
        [
            Paragraph("DB_Data.Count_A / Count_B", table_cell_bold),
            Paragraph("DB1.DBW0 / DBW2", table_cell_center),
            Paragraph("Int", table_cell_center),
            Paragraph("Tổng sản lượng tích lũy sản phẩm Loại A và Loại B (hiển thị SCADA & lưu SQL).", table_cell)
        ],
        [
            Paragraph("DB_Data.Current_Box_A / B", table_cell_bold),
            Paragraph("DB1.DBW4 / DBW6", table_cell_center),
            Paragraph("Int", table_cell_center),
            Paragraph("Số sản phẩm hiện có trong thùng A và thùng B đang đóng dở (từ 0 đến 5).", table_cell)
        ],
        [
            Paragraph("DB_Data.Box_Total", table_cell_bold),
            Paragraph("DB1.DBW8", table_cell_center),
            Paragraph("Int", table_cell_center),
            Paragraph("Tổng số thùng hàng đã hoàn tất đóng gói trong ca làm việc.", table_cell)
        ],
        [
            Paragraph("DB_Data.Trig_SQL", table_cell_bold),
            Paragraph("DB1.DBX10.0", table_cell_center),
            Paragraph("Bool", table_cell_center),
            Paragraph("Cờ báo hiệu vừa đóng xong 1 thùng hàng ➔ kích hoạt VBScript tự động ghi dữ liệu SQL.", table_cell)
        ]
    ]

    tags_table = Table(tags_data, colWidths=[40 * mm, 26 * mm, 22 * mm, 96 * mm])
    tags_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, LIGHT_BG]),
        ('TOPPADDING', (0,0), (-1,-1), 1.6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 1.6),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(tags_table)

    # ========================== TRANG 3 ==========================
    story.append(PageBreak())
    story.append(Paragraph("5. LẬP TRÌNH CẤU TRÚC ĐA NGÔN NGỮ TRÊN TIA PORTAL (LAD & SCL)", h1_style))
    story.append(Paragraph(
        "Mã nguồn thực hành chuẩn xác: FC1 (ngôn ngữ LAD) phụ trách động lực, FB1 (ngôn ngữ SCL) phụ trách đếm và đóng thùng:",
        body_style
    ))

    lad_scl_text = Paragraph(
        "<b>[KHỐI 1 - FC1_Control_Conveyor (Lập trình LAD - Thành viên 1 thực hiện)]:</b><br/>"
        "• <i>Network 1 (Khởi động hệ thống):</i> Mạch tự giữ Start/Stop (kết hợp nút vật lý I_Start và nút HMI_Start, ngắt bằng I_Stop hoặc HMI_Stop) điều khiển cờ <code>M_System_Run</code> và đèn <code>Q_Light_Run</code>.<br/>"
        "• <i>Network 2 (Chạy băng tải):</i> Khi <code>M_System_Run = 1</code> và Auto (<code>I_Auto_Man = 1</code>), bật <code>Q_Conveyor</code>. Khi Manual, điều khiển băng tải độc lập qua nút bấm tay trên WinCC.<br/>"
        "• <i>Network 3 & 4 (Đẩy Xy-lanh):</i> Phôi Cao (<code>I_Sensor_High = 1 & I_Sensor_Low = 1</code>) ➔ Kích Timer trễ <code>TON_Push_A</code> (PT = 2.5s) ➔ Bật van <code>Q_Pusher_A</code> trong 0.8s. Tương tự cho phôi Thấp với <code>TON_Push_B</code> (PT = 4.5s) đẩy <code>Q_Pusher_B</code>.<br/><br/>"
        "<b>[KHỐI 2 - FB1_Counter_And_Batch (Lập trình SCL - Thành viên 2 thực hiện)]:</b><br/>"
        "<code>// 1. Cài đặt định mức đóng thùng mặc định = 5 SP/thùng nếu chưa nhập trên SCADA</code><br/>"
        "IF \"DB_Data\".Set_Qty_Per_Box &lt;= 0 THEN \"DB_Data\".Set_Qty_Per_Box := 5; END_IF;<br/>"
        "<code>// 2. Bắt sườn lên xung cảm biến đếm tại Máng A (phôi Cao) và Máng B (phôi Thấp)</code><br/>"
        "#R_Trig_A(CLK := \"I_Count_A\"); #R_Trig_B(CLK := \"I_Count_B\");<br/>"
        "IF #R_Trig_A.Q THEN<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Count_A := \"DB_Data\".Count_A + 1; \"DB_Data\".Current_Box_A := \"DB_Data\".Current_Box_A + 1;<br/>"
        "END_IF;<br/>"
        "IF #R_Trig_B.Q THEN<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Count_B := \"DB_Data\".Count_B + 1; \"DB_Data\".Current_Box_B := \"DB_Data\".Current_Box_B + 1;<br/>"
        "END_IF;<br/>"
        "<code>// 3. Kiểm tra hoàn tất đóng gói 1 thùng Loại A hoặc B -&gt; Bật cờ Trig_SQL ghi CSDL</code><br/>"
        "IF (\"DB_Data\".Current_Box_A &gt;= \"DB_Data\".Set_Qty_Per_Box) THEN<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Box_Total := \"DB_Data\".Box_Total + 1; \"DB_Data\".Last_Box_Type := 'THUNG_LOAI_A_CAO';<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Current_Box_A := 0; \"DB_Data\".Trig_SQL := TRUE;<br/>"
        "ELSIF (\"DB_Data\".Current_Box_B &gt;= \"DB_Data\".Set_Qty_Per_Box) THEN<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Box_Total := \"DB_Data\".Box_Total + 1; \"DB_Data\".Last_Box_Type := 'THUNG_LOAI_B_THAP';<br/>"
        "&nbsp;&nbsp;\"DB_Data\".Current_Box_B := 0; \"DB_Data\".Trig_SQL := TRUE;<br/>"
        "END_IF;",
        code_style
    )

    lad_scl_box = Table([[lad_scl_text]], colWidths=[184 * mm])
    lad_scl_box.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#F8FAFC")),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor("#94A3B8")),
        ('TOPPADDING', (0,0), (-1,-1), 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 3),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(lad_scl_box)
    story.append(Spacer(1, 4))

    story.append(Paragraph("6. THIẾT KẾ HỆ THỐNG GIAO DIỆN SCADA (WINCC RUNTIME ADVANCED)", h1_style))
    story.append(Paragraph(
        "Thành viên 4 cấu hình PC Station trong TIA Portal, phối hợp Thành viên 5 tạo bảng hiển thị dữ liệu lịch sử SQL trực quan:",
        body_style
    ))

    wincc_features = [
        [
            Paragraph("<b>Phân Khu Chức Năng</b>", table_header),
            Paragraph("<b>Chi Tiết Cấu Hình & Thuộc Tính Thực Hiện Trên WinCC Runtime</b>", table_header)
        ],
        [
            Paragraph("<b>1. Màn hình Vận hành Chính<br/>(Main Screen)</b>", table_cell_bold),
            Paragraph("• <i>Bảng điều khiển:</i> Các nút nhấn START, STOP, RESET dùng sự kiện <code>Press ➔ SetBit</code> và <code>Release ➔ ResetBit</code>.<br/>"
                      "• <i>Chế độ Manual (Bằng tay):</i> Dùng thuộc tính <code>Animations ➔ Visibility</code> ẩn/hiện bảng nút nhấn chạy thử (chỉ hiển thị khi <code>I_Auto_Man = 0</code>) để kỹ thuật viên test độc lập băng tải và từng xy-lanh khi quay video demo.<br/>"
                      "• <i>Hiệu ứng động (Animations):</i> Băng tải quay đổi màu, Xy-lanh đổi màu Xanh lá khi tác động (Appearance), hiển thị số lượng phôi trong thùng (VD: 3/5 SP) và tổng số thùng hoàn tất.", table_cell)
        ],
        [
            Paragraph("<b>2. Đồ thị Thời gian thực<br/>(Trend View)</b>", table_cell_bold),
            Paragraph("• Cấu hình <i>Historical Data (Data_Log_1)</i> lưu trữ lịch sử các biến <code>Count_A</code>, <code>Count_B</code>, <code>Box_Total</code> theo chu kỳ 1 giây.<br/>"
                      "• Kéo công cụ <b>Trend view</b> ra màn hình giám sát để vẽ biểu đồ trực quan tốc độ gia tăng sản lượng theo thời gian thực.", table_cell)
        ],
        [
            Paragraph("<b>3. Bảng Lịch Sử & Báo Lỗi<br/>(Alarm & SQL History)</b>", table_cell_bold),
            Paragraph("• Cấu hình <b>Alarm View</b> giám sát sự cố: Dừng khẩn cấp (E-Stop), cảnh báo thùng đầy hoặc kẹt phôi.<br/>"
                      "• <b>Bảng dữ liệu SQL trên SCADA (Thành viên 5 hỗ trợ):</b> Tạo vùng hiển thị bảng Table trên WinCC, bấm nút <i>'ĐỌC DỮ LIỆU SQL'</i> để gọi VBScript đọc 5 bản ghi mới nhất từ SQL Server hiển thị ngay trên giao diện mà không cần mở SSMS.", table_cell)
        ]
    ]

    wincc_table = Table(wincc_features, colWidths=[42 * mm, 142 * mm])
    wincc_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, LIGHT_BG]),
        ('TOPPADDING', (0,0), (-1,-1), 2.5),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2.5),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(wincc_table)

    # ========================== TRANG 4 ==========================
    story.append(PageBreak())
    story.append(Paragraph("7. CSDL SQL SERVER & 4 KỊCH BẢN VBSCRIPT NÂNG CAO (THÀNH VIÊN 5 PHỤ TRÁCH)", h1_style))
    story.append(Paragraph(
        "Thành viên 5 tạo CSDL với 2 bảng quan hệ trên SQL Server và lập trình 4 kịch bản VBScript hoàn chỉnh trong WinCC:",
        body_style
    ))

    sql_vbs_code = Paragraph(
        "<b>-- [1] LỆNH TẠO 2 BẢNG TRONG SQL SERVER (Thành viên 5 chạy trong SSMS)</b><br/>"
        "CREATE DATABASE SCADA_BTL; GO<br/>"
        "USE SCADA_BTL;<br/>"
        "-- Bảng 1: Lịch sử đóng thùng hàng thành phẩm | Bảng 2: Lịch sử sự cố dừng máy<br/>"
        "CREATE TABLE Production_Log (ID INT IDENTITY(1,1) PRIMARY KEY, ThoiGian DATETIME DEFAULT GETDATE(), MaThung INT, LoaiThung NVARCHAR(50), DinhMuc INT, TongLoaiA INT, TongLoaiB INT);<br/>"
        "CREATE TABLE System_Alarm_Log (AlarmID INT IDENTITY(1,1) PRIMARY KEY, ThoiGian DATETIME DEFAULT GETDATE(), MaLoi NVARCHAR(20), NoiDungLoi NVARCHAR(100), TrangThai NVARCHAR(20));<br/><br/>"
        "<b>' [2] VBSCRIPT 1: TỰ ĐỘNG GHI SẢN LƯỢNG VÀO SQL (Sự kiện Value Change của DB_Data_Trig_SQL)</b><br/>"
        "Sub Write_Production_SQL()<br/>"
        "&nbsp;&nbsp;If SmartTags(\"DB_Data_Trig_SQL\") = True Then<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;Dim conn, sql : Set conn = CreateObject(\"ADODB.Connection\")<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;conn.Open \"Provider=SQLOLEDB;Data Source=.\\SQLEXPRESS;Initial Catalog=SCADA_BTL;Integrated Security=SSPI;\"<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;sql = \"INSERT INTO Production_Log (MaThung, LoaiThung, DinhMuc, TongLoaiA, TongLoaiB) VALUES (\" & SmartTags(\"DB_Data_Box_Total\") & \", '\" & SmartTags(\"DB_Data_Last_Box_Type\") & \"', \" & SmartTags(\"DB_Data_Set_Qty_Per_Box\") & \", \" & SmartTags(\"DB_Data_Count_A\") & \", \" & SmartTags(\"DB_Data_Count_B\") & \")\"<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;conn.Execute sql : conn.Close : Set conn = Nothing<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;SmartTags(\"DB_Data_Trig_SQL\") = False ' Reset cờ tránh ghi trùng bản ghi<br/>"
        "&nbsp;&nbsp;End If<br/>"
        "End Sub<br/><br/>"
        "<b>' [3] VBSCRIPT 2: ĐỌC DỮ LIỆU TỪ SQL SERVER HIỂN THỊ NGƯỢC LÊN GIAO DIỆN SCADA (Nút Đọc SQL)</b><br/>"
        "Sub Read_SQL_To_SCADA()<br/>"
        "&nbsp;&nbsp;Dim conn, rs, i : Set conn = CreateObject(\"ADODB.Connection\")<br/>"
        "&nbsp;&nbsp;conn.Open \"Provider=SQLOLEDB;Data Source=.\\SQLEXPRESS;Initial Catalog=SCADA_BTL;Integrated Security=SSPI;\"<br/>"
        "&nbsp;&nbsp;Set rs = conn.Execute(\"SELECT TOP 5 MaThung, LoaiThung, TongLoaiA, TongLoaiB FROM Production_Log ORDER BY ID DESC\")<br/>"
        "&nbsp;&nbsp;i = 1<br/>"
        "&nbsp;&nbsp;Do While Not rs.EOF And i &lt;= 5<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;SmartTags(\"HMI_History_Box_\" & i) = \"Thùng #\" & rs(0) & \" - \" & rs(1) & \" (A:\" & rs(2) & \" | B:\" & rs(3) & \")\"<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;i = i + 1 : rs.MoveNext<br/>"
        "&nbsp;&nbsp;Loop<br/>"
        "&nbsp;&nbsp;rs.Close : conn.Close : Set conn = Nothing<br/>"
        "End Sub<br/><br/>"
        "<b>' [4] VBSCRIPT 3: XUẤT BÁO CÁO EXCEL (.CSV) THEO ĐỊNH DẠNG CA SẢN XUẤT (Nút Xuất Báo Cáo)</b><br/>"
        "Sub Export_Report_Excel()<br/>"
        "&nbsp;&nbsp;Dim conn, rs, fso, ts, filePath : filePath = \"C:\\SCADA_Report\\BaoCao_SanLuong_\" & Year(Now) & Month(Now) & Day(Now) & \".csv\"<br/>"
        "&nbsp;&nbsp;Set conn = CreateObject(\"ADODB.Connection\") : conn.Open \"Provider=SQLOLEDB;Data Source=.\\SQLEXPRESS;Initial Catalog=SCADA_BTL;Integrated Security=SSPI;\"<br/>"
        "&nbsp;&nbsp;Set rs = conn.Execute(\"SELECT ID, ThoiGian, MaThung, LoaiThung, DinhMuc, TongLoaiA, TongLoaiB FROM Production_Log\")<br/>"
        "&nbsp;&nbsp;Set fso = CreateObject(\"Scripting.FileSystemObject\") : Set ts = fso.CreateTextFile(filePath, True)<br/>"
        "&nbsp;&nbsp;ts.WriteLine \"BÁO CÁO SẢN XUẤT & ĐÓNG GÓI - HỆ THỐNG SCADA PHÂN LOẠI CHIỀU CAO\"<br/>"
        "&nbsp;&nbsp;ts.WriteLine \"STT,Thời Gian,Mã Thùng,Loại Thùng,SL Định Mức,Tổng SP Loại A,Tổng SP Loại B\"<br/>"
        "&nbsp;&nbsp;Do While Not rs.EOF<br/>"
        "&nbsp;&nbsp;&nbsp;&nbsp;ts.WriteLine rs(0) & \",\" & rs(1) & \",\" & rs(2) & \",\" & rs(3) & \",\" & rs(4) & \",\" & rs(5) & \",\" & rs(6) : rs.MoveNext<br/>"
        "&nbsp;&nbsp;Loop<br/>"
        "&nbsp;&nbsp;ts.Close : rs.Close : conn.Close<br/>"
        "End Sub",
        code_style
    )

    sql_vbs_box = Table([[sql_vbs_code]], colWidths=[184 * mm])
    sql_vbs_box.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,-1), colors.HexColor("#F8FAFC")),
        ('BOX', (0,0), (-1,-1), 1, colors.HexColor("#94A3B8")),
        ('TOPPADDING', (0,0), (-1,-1), 2),
        ('BOTTOMPADDING', (0,0), (-1,-1), 2),
        ('LEFTPADDING', (0,0), (-1,-1), 6),
        ('RIGHTPADDING', (0,0), (-1,-1), 6),
    ]))
    story.append(sql_vbs_box)
    story.append(Spacer(1, 4))

    story.append(Paragraph("8. KỊCH BẢN QUAY VIDEO DEMO THỰC NGHIỆM & HỒ SƠ SOURCE NỘP BÀI", h1_style))
    story.append(Paragraph(
        "Toàn bộ điểm số dựa trên kết quả phần mềm chạy thực tế và video minh chứng. Nhóm thực hiện đúng kịch bản quay dưới đây:",
        body_style
    ))

    video_steps = [
        [
            Paragraph("<b>Phần Việc / Giai Đoạn</b>", table_header),
            Paragraph("<b>Thao Tác Thực Hiện Chi Tiết Cần Quay Video Minh Chứng (1080p)</b>", table_header),
            Paragraph("<b>Phụ Trách Chính</b>", table_header)
        ],
        [
            Paragraph("<b>1. Bố cục Màn hình & Test Manual</b>", table_cell_bold),
            Paragraph("Chia đôi màn hình (Trái: WinCC Runtime, Phải: Factory I/O 3D). Gạt công tắc sang <b>MANUAL</b> ➔ Bảng Manual hiện ra. Bấm lần lượt: Bật/Tắt Băng tải, Test Xy-lanh 1 (thụt thò), Test Xy-lanh 2 (thụt thò).", table_cell),
            Paragraph("TV4 & TV1 thao tác<br/>TV5 setup OBS", table_cell_center)
        ],
        [
            Paragraph("<b>2. Chạy Auto & Phân loại Phôi</b>", table_cell_bold),
            Paragraph("Gạt công tắc sang <b>AUTO</b> ➔ Nhấn <b>START</b>; Băng tải chạy cấp phôi; Quay rõ phôi Cao bị Xy-lanh 1 đẩy vào Máng A, phôi Thấp bị Xy-lanh 2 đẩy vào Máng B; Số lượng SP trên WinCC nhảy từ 1 đến 5.", table_cell),
            Paragraph("TV2 & TV3<br/>(Giám sát đếm phôi)", table_cell_center)
        ],
        [
            Paragraph("<b>3. Đóng Thùng & Tự Lưu SQL</b>", table_cell_bold),
            Paragraph("Khi thùng A đủ 5 SP ➔ Biến <code>Box_Total</code> tăng lên 1; Mở cửa sổ SSMS chạy <code>SELECT * FROM Production_Log</code> chứng minh vừa ghi tự động 1 dòng mới vào CSDL SQL Server.", table_cell),
            Paragraph("Thành viên 5<br/>(Kiểm tra SQL)", table_cell_center)
        ],
        [
            Paragraph("<b>4. Đọc SQL lên SCADA & Xuất CSV</b>", table_cell_bold),
            Paragraph("Bấm nút <b>'ĐỌC DỮ LIỆU SQL'</b> trên WinCC ➔ 5 bản ghi mới nhất hiển thị ngay trên bảng lịch sử SCADA. Bấm nút <b>'XUẤT BÁO CÁO EXCEL'</b> ➔ Mở thư mục <code>C:\\SCADA_Report</code> mở file CSV bằng Excel.", table_cell),
            Paragraph("Thành viên 5<br/>(Biên tập Video MP4)", table_cell_center)
        ]
    ]

    video_table = Table(video_steps, colWidths=[38 * mm, 114 * mm, 32 * mm])
    video_table.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), PRIMARY),
        ('GRID', (0,0), (-1,-1), 0.5, BORDER_COLOR),
        ('VALIGN', (0,0), (-1,-1), 'MIDDLE'),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, LIGHT_BG]),
        ('TOPPADDING', (0,0), (-1,-1), 1.6),
        ('BOTTOMPADDING', (0,0), (-1,-1), 1.6),
        ('LEFTPADDING', (0,0), (-1,-1), 4),
        ('RIGHTPADDING', (0,0), (-1,-1), 4),
    ]))
    story.append(video_table)

    # Build PDF
    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"Xuat PDF thanh cong tai: {output_path}")

if __name__ == '__main__':
    target_onedrive = r'C:\Users\thanh\OneDrive\Máy tính\Ke_Hoach_Phan_Cong_BTL_SCADA_5_Thanh_Vien.pdf'
    target_desktop = r'C:\Users\thanh\Desktop\Ke_Hoach_Phan_Cong_BTL_SCADA_5_Thanh_Vien.pdf'
    
    build_pdf(target_onedrive)
    
    try:
        shutil.copyfile(target_onedrive, target_desktop)
        print(f"Da sao chep sang Desktop: {target_desktop}")
    except Exception as e:
        print(f"Luu Desktop mac dinh loi: {e}")
