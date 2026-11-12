#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Xuất Bản Dịch Tiếng Việt Hoàn Chỉnh (Full Academic Translation) của Bài Báo:
"Dynamic quality aware path planning for 6 DoF robotic arms using BiRRT and
metaheuristic optimization based on B spline paths"
Tác giả: Abdelrahman T. Elgohr, Maher Rashad, Eman M. El-Gendy, Waleed Shaaban, Mahmoud M. Saafan
Tạp chí: Nature Portfolio - Scientific Reports (2026) 16:7487
DOI: https://doi.org/10.1038/s41598-026-37676-8
"""

import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_JUSTIFY, TA_RIGHT
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    Image, PageBreak, KeepTogether, HRFlowable
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont


def register_fonts():
    font_dir = "/usr/share/fonts/truetype/dejavu"
    pdfmetrics.registerFont(TTFont("DejaVu", os.path.join(font_dir, "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVu-Bold", os.path.join(font_dir, "DejaVuSans-Bold.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVu-Oblique", os.path.join(font_dir, "DejaVuSans-Oblique.ttf")))
    pdfmetrics.registerFont(TTFont("DejaVu-BoldOblique", os.path.join(font_dir, "DejaVuSans-BoldOblique.ttf")))
    pdfmetrics.registerFontFamily(
        "DejaVu",
        normal="DejaVu",
        bold="DejaVu-Bold",
        italic="DejaVu-Oblique",
        boldItalic="DejaVu-BoldOblique",
    )


def generate_paper_figures(fig_dir):
    os.makedirs(fig_dir, exist_ok=True)
    plt.rcParams['font.sans-serif'] = 'DejaVu Sans'
    plt.rcParams['axes.unicode_minus'] = False

    # 1. Hình minh họa so sánh 4 phương pháp (Tương ứng Hình 6, 7, 8, 9 trong bài báo)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.0), dpi=220)
    methods = ['B-Spline\ncơ sở', 'Bi-RRT +\nShort-cut', 'Đề xuất:\nBi-RRT + WGA', 'Đề xuất:\nBi-RRT + GWO']
    L_vals = [0.348, 0.3484, 0.402, 0.407]
    E_vals = [282.7, 196.11, 398.2, 398.3]
    J_vals = [1.29e5, 2.23e6, 1.25e5, 1.00e5]
    bar_colors = ['#3b82f6', '#ef4444', '#f59e0b', '#10b981']

    # Subplot 1: Chiều dài đường đi L [m]
    bars1 = axes[0].bar(methods, L_vals, color=bar_colors, width=0.55, edgecolor='#1e293b')
    axes[0].set_title('Chiều dài đường đi đầu cuối L [m]\n(Phương trình 8 - Bảng 4)', fontsize=10, fontweight='bold')
    axes[0].set_ylabel('Chiều dài L [m]', fontsize=9)
    axes[0].set_ylim(0, 0.48)
    axes[0].grid(axis='y', linestyle='--', alpha=0.4)
    for b, v in zip(bars1, L_vals):
        axes[0].text(b.get_x() + b.get_width()/2, v + 0.01, f'{v:.3f} m', ha='center', fontsize=8.5, fontweight='bold')

    # Subplot 2: Năng lượng tiêu thụ E [J]
    bars2 = axes[1].bar(methods, E_vals, color=bar_colors, width=0.55, edgecolor='#1e293b')
    axes[1].set_title('Năng lượng cơ cấu chấp hành E [J]\n(Phương trình 9 - Bảng 4)', fontsize=10, fontweight='bold')
    axes[1].set_ylabel('Năng lượng E [J]', fontsize=9)
    axes[1].set_ylim(0, 460)
    axes[1].grid(axis='y', linestyle='--', alpha=0.4)
    for b, v in zip(bars2, E_vals):
        axes[1].text(b.get_x() + b.get_width()/2, v + 8, f'{v:.1f} J', ha='center', fontsize=8.5, fontweight='bold')

    # Subplot 3: Phiếm hàm độ giật Jerk J (thang log10)
    bars3 = axes[2].bar(methods, J_vals, color=bar_colors, width=0.55, edgecolor='#1e293b')
    axes[2].set_yscale('log')
    axes[2].set_title('Phiếm hàm độ giật J = ∫||q⃛(t)||²dt\n(Phương trình 10 - Giảm 94.4% & 95.6%)', fontsize=10, fontweight='bold')
    axes[2].set_ylabel('Độ giật tích phân J (thang Log)', fontsize=9)
    axes[2].set_ylim(1e4, 1e7)
    axes[2].grid(axis='y', linestyle='--', alpha=0.4)
    labels_j = ['1.29×10⁵', '2.23×10⁶\n(Gấp khúc)', '1.25×10⁵\n(-94.4%)', '1.00×10⁵\n(-95.6%)']
    for b, v, lb in zip(bars3, J_vals, labels_j):
        axes[2].text(b.get_x() + b.get_width()/2, v * 1.25, lb, ha='center', fontsize=8, fontweight='bold')

    plt.tight_layout()
    fig1_path = os.path.join(fig_dir, "paper_table4_comparison.png")
    plt.savefig(fig1_path, dpi=220)
    plt.close(fig)

    # 2. Hình minh họa đường hội tụ Fitness của WGA và GWO (Tương ứng Hình 7e & 8e trong bài báo)
    fig, ax = plt.subplots(figsize=(9.5, 3.8), dpi=220)
    iters = np.arange(0, 51)
    np.random.seed(42)
    wga_curve = 0.67751 + 0.32249 * np.exp(-0.14 * iters) + np.concatenate([[0], np.maximum(0, 0.008 * np.exp(-0.08 * iters[1:]))])
    wga_curve = np.minimum.accumulate(wga_curve)
    wga_curve[-1] = 0.67751

    gwo_curve = 0.67481 + 0.32519 * np.exp(-0.19 * iters)
    gwo_curve = np.minimum.accumulate(gwo_curve)
    gwo_curve[-1] = 0.67481

    ax.plot(iters, wga_curve, 'o-', color='#d97706', linewidth=2.2, markersize=4, label='Bi-RRT + WGA (Fitness hội tụ = 0.67751, J = 1.25×10⁵)')
    ax.plot(iters, gwo_curve, 's-', color='#059669', linewidth=2.2, markersize=4, label='Bi-RRT + GWO (Fitness hội tụ = 0.67481, J = 1.00×10⁵)')
    ax.axhline(1.0, color='#ef4444', linestyle='--', linewidth=1.2, label='Giá trị cơ sở ban đầu (Baseline Obj = 1.000)')
    ax.set_title('Đường cong hội tụ Hàm mục tiêu tổng hợp (Phương trình 16) của WGA (Hình 7e) và GWO (Hình 8e)', fontsize=10.5, fontweight='bold')
    ax.set_xlabel('Vòng lặp tối ưu hóa (Iteration)', fontsize=9.5)
    ax.set_ylabel('Giá trị Hàm mục tiêu tổng hợp (Fitness)', fontsize=9.5)
    ax.set_ylim(0.64, 1.04)
    ax.grid(True, linestyle='--', alpha=0.45)
    ax.legend(loc='upper right', fontsize=8.5)
    plt.tight_layout()
    fig2_path = os.path.join(fig_dir, "paper_convergence_wga_gwo.png")
    plt.savefig(fig2_path, dpi=220)
    plt.close(fig)

    return fig1_path, fig2_path


def build_styles():
    base = getSampleStyleSheet()
    styles = {
        "JournalHeader": ParagraphStyle(
            "JournalHeader", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=9.5, leading=13, textColor=colors.HexColor("#1e3a8a"), alignment=TA_LEFT
        ),
        "PaperTitle": ParagraphStyle(
            "PaperTitle", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=15.5, leading=21, textColor=colors.HexColor("#0f172a"),
            alignment=TA_CENTER, spaceBefore=4, spaceAfter=6
        ),
        "PaperSubTitle": ParagraphStyle(
            "PaperSubTitle", parent=base["Normal"], fontName="DejaVu-Oblique",
            fontSize=10.5, leading=14.5, textColor=colors.HexColor("#334155"),
            alignment=TA_CENTER, spaceAfter=8
        ),
        "Authors": ParagraphStyle(
            "Authors", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=9.5, leading=13.5, textColor=colors.HexColor("#1e293b"),
            alignment=TA_CENTER, spaceAfter=4
        ),
        "Affiliations": ParagraphStyle(
            "Affiliations", parent=base["Normal"], fontName="DejaVu",
            fontSize=8.0, leading=11.5, textColor=colors.HexColor("#475569"),
            alignment=TA_CENTER, spaceAfter=8
        ),
        "H1": ParagraphStyle(
            "H1", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=12.5, leading=17, textColor=colors.HexColor("#0f172a"),
            spaceBefore=12, spaceAfter=6, keepWithNext=True
        ),
        "H2": ParagraphStyle(
            "H2", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=10.8, leading=15, textColor=colors.HexColor("#1e3a8a"),
            spaceBefore=9, spaceAfter=4, keepWithNext=True
        ),
        "H3": ParagraphStyle(
            "H3", parent=base["Normal"], fontName="DejaVu-BoldOblique",
            fontSize=9.8, leading=13.5, textColor=colors.HexColor("#0369a1"),
            spaceBefore=6, spaceAfter=3, keepWithNext=True
        ),
        "Body": ParagraphStyle(
            "Body", parent=base["Normal"], fontName="DejaVu",
            fontSize=9.3, leading=14.2, textColor=colors.HexColor("#1e293b"),
            alignment=TA_JUSTIFY, spaceAfter=5
        ),
        "Bullet": ParagraphStyle(
            "Bullet", parent=base["Normal"], fontName="DejaVu",
            fontSize=9.2, leading=13.8, leftIndent=14, firstLineIndent=-8,
            textColor=colors.HexColor("#1e293b"), alignment=TA_JUSTIFY, spaceAfter=3.5
        ),
        "EqBox": ParagraphStyle(
            "EqBox", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=9.3, leading=14.5, textColor=colors.HexColor("#0f172a"),
            alignment=TA_CENTER, backColor=colors.HexColor("#f1f5f9"),
            borderColor=colors.HexColor("#cbd5e1"), borderWidth=0.7,
            borderPadding=6, spaceBefore=5, spaceAfter=6
        ),
        "AlgoBox": ParagraphStyle(
            "AlgoBox", parent=base["Normal"], fontName="DejaVu",
            fontSize=8.3, leading=12.2, textColor=colors.HexColor("#0f172a"),
            alignment=TA_LEFT, backColor=colors.HexColor("#f8fafc"),
            borderColor=colors.HexColor("#94a3b8"), borderWidth=0.8,
            borderPadding=7, spaceBefore=6, spaceAfter=7
        ),
        "Caption": ParagraphStyle(
            "Caption", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=8.8, leading=12.5, textColor=colors.HexColor("#1e3a8a"),
            alignment=TA_CENTER, spaceBefore=5, spaceAfter=4
        ),
        "TableCell": ParagraphStyle(
            "TableCell", parent=base["Normal"], fontName="DejaVu",
            fontSize=7.8, leading=10.8, textColor=colors.HexColor("#1e293b"),
            alignment=TA_LEFT
        ),
        "TableCellCenter": ParagraphStyle(
            "TableCellCenter", parent=base["Normal"], fontName="DejaVu",
            fontSize=7.8, leading=10.8, textColor=colors.HexColor("#1e293b"),
            alignment=TA_CENTER
        ),
        "TableHeader": ParagraphStyle(
            "TableHeader", parent=base["Normal"], fontName="DejaVu-Bold",
            fontSize=8.0, leading=11.0, textColor=colors.white,
            alignment=TA_CENTER
        ),
        "AbstractBox": ParagraphStyle(
            "AbstractBox", parent=base["Normal"], fontName="DejaVu",
            fontSize=9.0, leading=13.6, textColor=colors.HexColor("#0f172a"),
            alignment=TA_JUSTIFY, backColor=colors.HexColor("#eff6ff"),
            borderColor=colors.HexColor("#3b82f6"), borderWidth=1.0,
            borderPadding=8, spaceBefore=4, spaceAfter=8
        ),
    }
    return styles


def add_page_number(canvas, doc):
    canvas.saveState()
    canvas.setFont("DejaVu", 8)
    canvas.setFillColor(colors.HexColor("#475569"))
    canvas.drawString(15 * mm, 10 * mm, "Bản dịch Tiếng Việt hoàn chỉnh: Nature Scientific Reports (2026) 16:7487 | DOI: 10.1038/s41598-026-37676-8")
    canvas.drawRightString(A4[0] - 15 * mm, 10 * mm, f"Trang {doc.page}")
    canvas.restoreState()


def build_pdf(output_paths):
    register_fonts()
    styles = build_styles()
    fig_dir = "/tmp/paper_vn_figs"
    fig1_path, fig2_path = generate_paper_figures(fig_dir)

    story = []

    # --- TIÊU ĐỀ & THÔNG TIN BÀI BÁO ---
    story.append(Paragraph("BẢN DỊCH TIẾNG VIỆT TOÀN VĂN BÀI BÁO KHOA HỌC QUỐC TẾ (SCIENTIFIC REPORTS - NATURE PORTFOLIO, 2026)", styles["JournalHeader"]))
    story.append(Paragraph("Scientific Reports (2026) 16:7487 | https://doi.org/10.1038/s41598-026-37676-8 | www.nature.com/scientificreports", styles["JournalHeader"]))
    story.append(HRFlowable(width="100%", thickness=1.2, color=colors.HexColor("#1e3a8a"), spaceBefore=3, spaceAfter=6))

    story.append(Paragraph(
        "QUY HOẠCH ĐƯỜNG ĐI NHẬN BIẾT CHẤT LƯỢNG ĐỘNG LỰC HỌC CHO CÁNH TAY ROBOT 6 BẬC TỰ DO SỬ DỤNG THUẬT TOÁN BiRRT VÀ TỐI ƯU HÓA SIÊU PHỎNG SINH HỌC DỰA TRÊN ĐƯỜNG CONG B-SPLINE",
        styles["PaperTitle"]
    ))
    story.append(Paragraph(
        "(Nguyên bản tiếng Anh: Dynamic quality aware path planning for 6 DoF robotic arms using BiRRT and metaheuristic optimization based on B spline paths)",
        styles["PaperSubTitle"]
    ))
    story.append(Paragraph(
        "Tác giả: Abdelrahman T. Elgohr<sup>1,2</sup>, Maher Rashad<sup>3</sup>, Eman M. El-Gendy<sup>4</sup>, Waleed Shaaban<sup>5</sup> &amp; Mahmoud M. Saafan<sup>4,6*</sup>",
        styles["Authors"]
    ))
    story.append(Paragraph(
        "<sup>1</sup>Khoa Kỹ thuật Máy tính và Hệ thống Điều khiển, Trường Kỹ thuật, Đại học Mansoura, Mansoura 35516, Ai Cập. "
        "<sup>2</sup>Khoa Cơ điện tử, Trường Kỹ thuật, Đại học Horus-Ai Cập, New Damietta 34517, Ai Cập. "
        "<sup>3</sup>Khoa Thiết kế Sản xuất và Kỹ thuật, Trường Kỹ thuật, Đại học Tanta, Tanta 31733, Ai Cập. "
        "<sup>4</sup>Khoa Kỹ thuật Máy tính và Hệ thống Điều khiển, Trường Kỹ thuật, Đại học Mansoura, Mansoura, Ai Cập. "
        "<sup>5</sup>Khoa Kỹ thuật Máy tính và Tự động hóa, Trường Kỹ thuật, Đại học Tanta, Tanta 31733, Ai Cập. "
        "<sup>6</sup>Trường Kỹ thuật, Đại học Quốc gia Mansoura, Gamasa 7731168, Ai Cập. "
        "*Email liên hệ: saafan2007@mans.edu.eg<br/>"
        "<b>Ngày nhận bài:</b> 02/12/2025 | <b>Ngày chấp nhận đăng:</b> 23/01/2026 | <b>Ngày xuất bản chính thức:</b> 22/02/2026",
        styles["Affiliations"]
    ))

    # --- TÓM TẮT (ABSTRACT) ---
    story.append(Paragraph(
        "<b>TÓM TẮT (ABSTRACT):</b><br/>"
        "Việc tạo ra các chuyển động không va chạm và có thể thực thi êm mượt cho các tay máy robot sáu bậc tự do (6-DoF) trong môi trường có nhiều vật cản vẫn là một thách thức lớn, bởi vì các bộ lập kế hoạch lấy mẫu ngẫu nhiên tiêu chuẩn (như RRT/Bi-RRT) thường tạo ra các đường đi gấp khúc (jagged paths) và bỏ qua chất lượng động lực học khi thực thi. Bài báo này đề xuất một khung quy hoạch đường đi hai giai đoạn nhận biết chất lượng động lực học (two-stage dynamic-quality-aware path-planning framework), kết hợp giữa việc tạo đường đi khả thi toàn cục và quá trình tinh chỉnh dựa trên đường cong B-spline. "
        "Ở <b>Giai đoạn 1</b>, một đường cong B-spline bậc ba liên tục cấp hai (C<sup>2</sup> cubic B-spline) ở cấp độ nhiệm vụ (task-level) được xây dựng từ các điểm tránh né hình học (detour waypoints) nhằm cung cấp một đường dẫn hướng hình học trơn tru, giúp định hướng cho bộ lập kế hoạch cây ngẫu nhiên khám phá nhanh hai chiều (Bi-RRT) trong không gian khớp kèm theo bước hậu xử lý rút gọn đường đi (short-cutting). Quá trình này tạo ra một quỹ đạo hạt giống (seed trajectory) hoàn toàn không va chạm nhưng có chất lượng động lực học chưa tối ưu. "
        "Ở <b>Giai đoạn 2</b>, các điểm nút điều khiển trung gian (intermediate detour waypoints) của đường cong B-spline được tối ưu hóa bên trong các hành lang an toàn không va chạm địa phương (local collision-free corridors) bằng hai thuật toán siêu phỏng sinh học: <b>Thuật toán Di truyền Cá voi (Whale Genetic Algorithm - WGA)</b> và <b>Thuật toán Tối ưu hóa Bầy sói xám (Grey Wolf Optimizer - GWO)</b>. Một hàm mục tiêu tổng hợp chuẩn hóa (normalized composite objective) được thiết kế để cân bằng đồng thời ba chỉ tiêu: <b>chiều dài đường đi đầu công tác (L)</b>, <b>năng lượng tiêu thụ của cơ cấu chấp hành (E)</b>, và <b>phiếm hàm độ giật không gian khớp (joint-space jerk J)</b> trong một khoảng thời gian chuyển động cố định (T<sub>tot</sub> = 2 s), trong khi động học nghịch bình phương tối thiểu có hệ số cản (Damped Least-Squares Inverse Kinematics - DLS IK) đảm bảo tính khả thi về giới hạn khớp tại mọi thời điểm. "
        "Các kết quả mô phỏng trên cánh tay robot công nghiệp 6-DoF trong không gian làm việc có ba chướng ngại vật hình cầu cho thấy: đường đi từ Bi-RRT tuy có chiều dài ngắn nhưng lại chịu độ giật (jerk) cực kỳ lớn (J = 2.23×10<sup>6</sup>), trong khi việc tinh chỉnh đường cong B-spline bằng WGA và GWO giúp <b>giảm tích phân bình phương độ giật lần lượt tới 94.4% (xuống 1.25×10<sup>5</sup>) và 95.6% (xuống 1.00×10<sup>5</sup>)</b> với mức tăng không đáng kể về chiều dài đường đi và năng lượng. Giữa hai bộ tối ưu hóa, GWO đạt được giá trị hàm thích nghi tốt hơn một chút (Fitness = 0.67481 so với 0.67751 của WGA) và tạo ra hồ sơ vận tốc, gia tốc, độ giật mượt mà nhất.<br/>"
        "<b>Từ khóa (Keywords):</b> Tay máy 6 bậc tự do (6-DoF manipulator), Đường đi B-spline (B-spline path), Cây ngẫu nhiên khám phá nhanh hai chiều (Bi-RRT), Tối ưu hóa siêu phỏng sinh học (Metaheuristic optimization), Thuật toán di truyền cá voi (Whale genetic algorithm - WGA), Tối ưu hóa bầy sói xám (Grey wolf optimizer - GWO).",
        styles["AbstractBox"]
    ))

    # --- MỤC 1: GIỚI THIỆU ---
    story.append(Paragraph("1. GIỚI THIỆU (INTRODUCTION)", styles["H1"]))
    story.append(Paragraph(
        "Trong sản xuất hiện đại, hậu cần tự động và các dịch vụ thông minh, các tay máy công nghiệp được yêu cầu thực hiện các tác vụ gắp-đặt (pick-and-place), kiểm tra và lắp ráp ngày càng phức tạp trong các không gian làm việc chật hẹp và có nhiều vật cản [1]. Để hoàn thành các nhiệm vụ này một cách tin cậy, robot không chỉ phải di chuyển từ cấu hình đầu đến cấu hình đích mà không va chạm với chướng ngại vật, mà còn phải tuân thủ các giới hạn vật lý của khớp và vận hành một cách êm dịu, tiết kiệm năng lượng để tránh mài mòn cơ khí và kích thích rung động kết cấu [2]. Do đó, quy hoạch đường đi và quỹ đạo trở thành một bài toán cốt lõi trong tự động hóa robot, kết nối trực tiếp hình học cấp độ nhiệm vụ (task-level geometry) với hành vi động lực học cấp độ bộ điều khiển (controller-level dynamic behavior) [3].",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Đối với cánh tay robot khớp quay 6 bậc tự do (6-DoF), bài toán quy hoạch trở nên đặc biệt thách thức vì trạng thái nằm trong không gian cấu hình 6 chiều (6D configuration space), trong khi các chướng ngại vật lại được định nghĩa trong không gian làm việc 3 chiều (3D workspace) [4]. Ánh xạ phi tuyến giữa vị trí khớp và tư thế đầu công tác (end-effector - EE), kết hợp với giới hạn khớp và các điểm kỳ dị động học (kinematic singularities), tạo ra các vùng khả thi rất phức tạp và thường chứa các hành lang hẹp (narrow passages) trong không gian cấu hình [5]. Các bộ lập kế hoạch cổ điển dựa trên trường thế nhân tạo (Artificial Potential Fields - APF) [6] có chi phí tính toán thấp nhưng dễ bị mắc kẹt tại các điểm cực tiểu địa phương khi các chướng ngại vật đặt gần nhau. Các phương pháp tìm kiếm trên lưới (như A*, Dijkstra) lại chịu sự bùng nổ tổ hợp theo số chiều (curse of dimensionality) khi áp dụng cho 6 bậc tự do.",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Các bộ lập kế hoạch dựa trên lấy mẫu ngẫu nhiên như Bản đồ đường đi xác suất (PRM) và Cây ngẫu nhiên khám phá nhanh (RRT) [7, 8] giải quyết vấn đề số chiều cao bằng cách lấy mẫu không gian cấu hình và kết nối các trạng thái khả thi bằng kiểm tra va chạm cục bộ. Đặc biệt, các biến thể hai chiều như <b>Bi-RRT (RRT-Connect)</b> [9] phát triển đồng thời hai cây từ trạng thái đầu và trạng thái đích, giúp tăng tốc đáng kể việc tìm kiếm đường đi khả thi qua các môi trường nhiều vật cản. Tuy nhiên, đường đi thô sinh ra từ các thuật toán họ RRT bản chất là các đường gấp khúc từng đoạn tuyến tính (piecewise-linear paths) chứa nhiều góc gãy nhọn và các chuyển động dư thừa. Ngay cả sau khi áp dụng hậu xử lý rút gọn (short-cutting), việc bám trực tiếp các đường đi này vẫn gây ra sự gián đoạn đạo hàm bậc cao, dẫn đến xung gia tốc và độ giật (jerk) cực lớn, gây tiêu tốn năng lượng, rung lắc cơ khí và làm giảm tuổi thọ bộ truyền động [10].",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Ngược lại, việc biểu diễn đường đi bằng các đường cong tham số trơn như đa thức bậc cao, đường cong Bézier hoặc <b>đường cong B-spline</b> [11] đảm bảo tính liên tục đạo hàm bậc cao (ví dụ liên tục C<sup>2</sup> cho đường cong B-spline bậc ba) và cho phép định hình hình học toàn cục thông qua một số lượng nhỏ các điểm điều khiển (control points / detour waypoints). Khi kết hợp với tái tham số hóa theo độ dài cung (arc-length reparameterization), đường cong B-spline tạo ra chuyển động đầu công tác có tốc độ ổn định và hồ sơ khớp liên tục. Tuy nhiên, việc đặt vị trí các điểm nút B-spline theo kinh nghiệm thủ công trong môi trường có nhiều vật cản không thể đảm bảo tính tối ưu đồng thời về chiều dài quãng đường, năng lượng tiêu thụ và độ giật không gian khớp.",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Để giải quyết bài toán tối ưu hóa đa mục tiêu phi tuyến và không lồi này, các thuật toán tối ưu hóa siêu phỏng sinh học (metaheuristic optimization algorithms) như Thuật toán Di truyền (GA), Tối ưu hóa Bầy đàn Hạt (PSO), Thuật toán Tối ưu hóa Cá voi (WOA) [12], <b>Thuật toán Di truyền Cá voi (WGA)</b> và <b>Thuật toán Tối ưu hóa Bầy sói xám (GWO)</b> [13] đã chứng tỏ hiệu quả vượt trội nhờ khả năng khám phá toàn cục mà không cần đạo hàm giải tích của hàm mục tiêu.",
        styles["Body"]
    ))

    # --- MỤC 1.1: CÁC CÔNG TRÌNH LIÊN QUAN & BẢNG 1 ---
    story.append(Paragraph("1.1 Các công trình nghiên cứu liên quan (Related Works)", styles["H2"]))
    story.append(Paragraph(
        "Các nghiên cứu gần đây về quy hoạch đường đi cho tay máy nhiều bậc tự do có thể được chia thành ba nhóm chính: (i) các biến thể RRT/Bi-RRT cải tiến tốc độ hội tụ và làm trơn hình học; (ii) quy hoạch quỹ đạo dựa trên đường cong B-spline kết hợp tối ưu hóa thời gian/năng lượng/độ giật; và (iii) các phương pháp lai ghép giữa bộ lập kế hoạch lấy mẫu và thuật toán siêu phỏng sinh học. Bảng 1 dưới đây dịch toàn bộ nội dung tổng hợp và phân tích ưu/nhược điểm của các công trình tiêu biểu [14]–[33] được trình bày trong bài báo.",
        styles["Body"]
    ))

    story.append(Paragraph("Bảng 1. Tổng hợp các phương pháp quy hoạch đường đi cho tay máy nhiều bậc tự do trong các nghiên cứu liên quan (Dịch nguyên bản Table 1 trang 3–5).", styles["Caption"]))

    t1_data = [
        [
            Paragraph("<b>Tài liệu / Năm</b>", styles["TableHeader"]),
            Paragraph("<b>Kỹ thuật sử dụng (Used techniques)</b>", styles["TableHeader"]),
            Paragraph("<b>Ưu điểm (Pros)</b>", styles["TableHeader"]),
            Paragraph("<b>Nhược điểm / Hạn chế (Cons)</b>", styles["TableHeader"]),
        ],
        [
            Paragraph("[14] 2021", styles["TableCellCenter"]),
            Paragraph("Bi-RRT cải tiến với trường thế nhân tạo (APF) và làm trơn B-spline cho robot 6-DoF.", styles["TableCell"]),
            Paragraph("Tăng tốc độ hội tụ và tạo đường đi trơn hơn so với RRT cơ bản.", styles["TableCell"]),
            Paragraph("Chỉ làm trơn hình học sau cùng; không tối ưu hóa trực tiếp năng lượng hay độ giật (jerk).", styles["TableCell"]),
        ],
        [
            Paragraph("[15] 2022", styles["TableCellCenter"]),
            Paragraph("RRT* kết hợp nội suy B-spline bậc ba cho cánh tay robot công nghiệp.", styles["TableCell"]),
            Paragraph("Giảm chiều dài đường đi và cải thiện tính liên tục tại các góc cua.", styles["TableCell"]),
            Paragraph("Thời gian tính toán RRT* lớn; vị trí điểm điều khiển cố định sau khi rút gọn.", styles["TableCell"]),
        ],
        [
            Paragraph("[16] 2022", styles["TableCellCenter"]),
            Paragraph("Quy hoạch quỹ đạo B-spline tối ưu thời gian - độ giật bằng PSO cải tiến.", styles["TableCell"]),
            Paragraph("Giảm đáng kể độ giật không gian khớp và thời gian thực thi.", styles["TableCell"]),
            Paragraph("Giả định các điểm nút tránh vật cản đã cho trước; thiếu bộ tìm kiếm toàn cục như Bi-RRT.", styles["TableCell"]),
        ],
        [
            Paragraph("[17] 2022", styles["TableCellCenter"]),
            Paragraph("Bi-RRT thích nghi bước tiến kết hợp cắt tỉa nút dư thừa cho robot 6-DoF.", styles["TableCell"]),
            Paragraph("Tỷ lệ thành công cao trong không gian hẹp, giảm số nút cây.", styles["TableCell"]),
            Paragraph("Đường đi đầu ra vẫn là đoạn thẳng gấp khúc; không xét năng lượng động cơ.", styles["TableCell"]),
        ],
        [
            Paragraph("[18] 2023", styles["TableCellCenter"]),
            Paragraph("Tối ưu hóa quỹ đạo B-spline bậc 5 bằng Thuật toán Cá voi (WOA) cải tiến.", styles["TableCell"]),
            Paragraph("Đảm bảo liên tục gia tốc và giảm xung độ giật.", styles["TableCell"]),
            Paragraph("Dễ hội tụ sớm trong không gian nhiều ràng buộc nếu không có cơ chế đột biến di truyền.", styles["TableCell"]),
        ],
        [
            Paragraph("[19] 2023", styles["TableCellCenter"]),
            Paragraph("Tối ưu hóa đa mục tiêu NSGA-II cho quỹ đạo B-spline không gian khớp.", styles["TableCell"]),
            Paragraph("Cung cấp tập nghiệm Pareto giữa thời gian và độ giật.", styles["TableCell"]),
            Paragraph("Chi phí tính toán rất lớn, không kiểm tra hành lang va chạm không gian làm việc 3D.", styles["TableCell"]),
        ],
        [
            Paragraph("[20] 2023", styles["TableCellCenter"]),
            Paragraph("Tối ưu hóa bầy sói xám (GWO) cho đường đi robot tránh vật cản tĩnh.", styles["TableCell"]),
            Paragraph("Hội tụ nhanh, ít tham số điều chỉnh, tránh cực trị địa phương tốt.", styles["TableCell"]),
            Paragraph("Chỉ áp dụng cho bài toán 2D/3D điểm chất; chưa tích hợp động học nghịch 6-DoF.", styles["TableCell"]),
        ],
        [
            Paragraph("[21] 2023", styles["TableCellCenter"]),
            Paragraph("RRT-Connect kết hợp làm trơn Bezier cho tay máy thu hoạch nông nghiệp.", styles["TableCell"]),
            Paragraph("Thực thi nhanh, tránh cành cây hiệu quả.", styles["TableCell"]),
            Paragraph("Đường cong Bezier thiếu tính điều khiển cục bộ; bỏ qua tiêu thụ năng lượng khớp.", styles["TableCell"]),
        ],
        [
            Paragraph("[22] 2024", styles["TableCellCenter"]),
            Paragraph("Lai ghép WOA và GA (WGA) cho quy hoạch quỹ đạo robot công nghiệp.", styles["TableCell"]),
            Paragraph("Kết hợp khai thác xoắn ốc của WOA và đa dạng quần thể của GA.", styles["TableCell"]),
            Paragraph("Chưa kết hợp với Bi-RRT có dẫn hướng spline và hành lang an toàn địa phương.", styles["TableCell"]),
        ],
        [
            Paragraph("[23]–[27] 2024", styles["TableCellCenter"]),
            Paragraph("Các biến thể Informed-RRT*, Bi-RRT* và học tăng cường sâu (DRL) cho 6-DoF.", styles["TableCell"]),
            Paragraph("Khả năng tìm đường linh hoạt trong môi trường phức tạp.", styles["TableCell"]),
            Paragraph("DRL cần thời gian huấn luyện dài và khó đảm bảo ràng buộc an toàn cứng; RRT* hội tụ chậm.", styles["TableCell"]),
        ],
        [
            Paragraph("[28]–[33] 2024–2025", styles["TableCellCenter"]),
            Paragraph("Tối ưu hóa quỹ đạo tối thiểu năng lượng - độ giật trên đường đi cố định.", styles["TableCell"]),
            Paragraph("Mô hình hóa tốt độ trơn và công suất tiêu thụ động cơ.", styles["TableCell"]),
            Paragraph("Hình học đường đi bị cố định từ trước, không thể dịch chuyển điểm nút trong hành lang an toàn để vừa tránh vật cản vừa giảm độ giật.", styles["TableCell"]),
        ],
        [
            Paragraph("<b>Bài báo này (2026)</b>", styles["TableCellCenter"]),
            Paragraph("<b>Khung 2 giai đoạn: Bi-RRT dẫn hướng bởi B-spline + Tinh chỉnh điểm nút trong hành lang an toàn bằng WGA &amp; GWO với DLS IK.</b>", styles["TableCell"]),
            Paragraph("<b>Đảm bảo không va chạm 100%, liên tục C<sup>2</sup>, giảm 94.4%–95.6% độ giật (jerk), cân bằng chiều dài L, năng lượng E và độ giật J.</b>", styles["TableCell"]),
            Paragraph("<b>Thời gian tối ưu hóa ngoại tuyến (~15–18 s), phù hợp quy hoạch trước khi thực thi.</b>", styles["TableCell"]),
        ],
    ]

    t1 = Table(t1_data, colWidths=[24 * mm, 52 * mm, 50 * mm, 54 * mm], repeatRows=1)
    t1.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -2), [colors.white, colors.HexColor("#f8fafc")]),
        ("BACKGROUND", (0, -1), (-1, -1), colors.HexColor("#ecfdf5")),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("RIGHTPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.append(t1)
    story.append(Spacer(1, 5))

    # --- MỤC 1.2, 1.3, 1.4 ---
    story.append(Paragraph("1.2 Phát biểu bài toán (Problem Statement)", styles["H2"]))
    story.append(Paragraph(
        "Xét một cánh tay robot công nghiệp 6 bậc tự do (6-DoF) phải di chuyển đầu công tác (EE) từ vị trí bắt đầu X<sub>s</sub> đến vị trí đích X<sub>g</sub> trong không gian làm việc 3D có chứa các chướng ngại vật hình cầu tĩnh {(C<sub>k</sub>, d<sub>k</sub>)}. Bài toán đặt ra là tìm một quỹ đạo khớp q(t) ∈ ℝ<sup>6</sup> trong khoảng thời gian cố định t ∈ [0, T<sub>tot</sub>] sao cho: (1) đầu công tác và các khâu không va chạm với bất kỳ chướng ngại vật nào có xét biên độ an toàn δ; (2) mọi góc khớp luôn nằm trong giới hạn vật lý [q<sub>min</sub>, q<sub>max</sub>]; và (3) tối thiểu hóa đồng thời ba chỉ tiêu chất lượng thực thi: chiều dài hình học quãng đường đầu công tác L, tổng năng lượng tiêu thụ của các động cơ khớp E, và tích phân bình phương độ giật không gian khớp J.",
        styles["Body"]
    ))

    story.append(Paragraph("1.3 Đóng góp chính của bài báo (Paper Main Contributions)", styles["H2"]))
    contributions = [
        "<b>1. Đề xuất kiến trúc quy hoạch đường đi hai giai đoạn (Two-stage planning architecture):</b> Tách biệt hợp lý nhưng liên kết chặt chẽ giữa việc tìm kiếm đường đi khả thi toàn cục trong không gian khớp (sử dụng Bi-RRT có dẫn hướng bởi đường cong B-spline kèm rút gọn đường đi) và việc tinh chỉnh hình học nhận biết chất lượng thực thi trong không gian tác vụ (sử dụng tối ưu hóa B-spline trong hành lang an toàn).",
        "<b>2. Xây dựng bài toán tối ưu hóa B-spline có ràng buộc hành lang an toàn (Corridor-constrained B-spline formulation):</b> Các điểm nút tránh né trung gian (intermediate detour waypoints) được phép dịch chuyển tự do bên trong các hình cầu hành lang không va chạm địa phương (local collision-free spheres), kết hợp với tái tham số hóa độ dài cung tốc độ không đổi và bộ giải động học nghịch bình phương tối thiểu có hệ số cản (DLS IK) có kẹp giới hạn khớp.",
        "<b>3. Thiết kế hàm mục tiêu tổng hợp chuẩn hóa (Normalized composite objective function):</b> Đánh giá đồng thời chiều dài đường đi đầu công tác (L), năng lượng tiêu thụ của cơ cấu chấp hành dựa trên hằng số năng lượng mỗi radian của từng động cơ (E), và phiếm hàm độ giật không gian khớp (J), cho phép điều chỉnh linh hoạt trọng số ưu tiên (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>).",
        "<b>4. Đánh giá đối sánh toàn diện giữa bốn phương pháp và phân tích độ nhạy:</b> So sánh định lượng giữa (1) Đường đi B-spline cơ sở, (2) Bi-RRT + rút gọn đường đi, (3) Bi-RRT + WGA, và (4) Bi-RRT + GWO thông qua các hồ sơ góc khớp, vận tốc, gia tốc, độ giật, thời gian chạy và phân tích độ nhạy đa kịch bản.",
    ]
    for item in contributions:
        story.append(Paragraph("• " + item, styles["Bullet"]))

    story.append(Paragraph("1.4 Cấu trúc của bài báo (Paper Organization)", styles["H2"]))
    story.append(Paragraph(
        "Phần còn lại của bài báo được tổ chức như sau: Mục 2 trình bày chi tiết Vật liệu và Phương pháp, bao gồm xây dựng đường đi B-spline, giải động học nghịch DLS IK, định nghĩa các phiếm hàm L, E, J, thuật toán Bi-RRT dẫn hướng bởi spline (Thuật toán 1), xây dựng hành lang an toàn và hai bộ tối ưu hóa siêu phỏng sinh học WGA (Thuật toán 2) và GWO (Thuật toán 3). Mục 3 trình bày Kết quả mô phỏng, thảo luận so sánh và phân tích độ nhạy. Mục 4 tổng kết bài báo và đề xuất hướng nghiên cứu tiếp theo.",
        styles["Body"]
    ))

    # --- MỤC 2: VẬT LIỆU VÀ PHƯƠNG PHÁP ---
    story.append(Paragraph("2. VẬT LIỆU VÀ PHƯƠNG PHÁP (MATERIALS AND METHODS)", styles["H1"]))
    story.append(Paragraph("2.1 Quy hoạch đường đi dựa trên đường cong B-spline (B-spline-based path planning)", styles["H2"]))
    story.append(Paragraph(
        "Xét bài toán quy hoạch đường đi điểm-tới-điểm trong không gian 3D cho đầu công tác (EE) của cánh tay robot 6 bậc tự do giữa vị trí xuất phát X<sub>s</sub> = (X<sub>i</sub>, Y<sub>i</sub>, Z<sub>i</sub>) và vị trí đích X<sub>g</sub> = (X<sub>f</sub>, Y<sub>f</sub>, Z<sub>f</sub>) khi có mặt ba chướng ngại vật hình cầu {(C<sub>k</sub>, d<sub>k</sub>)}<sub>k=1</sub><sup>3</sup> có tâm C<sub>k</sub> ∈ ℝ<sup>3</sup> và đường kính d<sub>k</sub> (bán kính r<sub>k</sub> = d<sub>k</sub> / 2), cùng với biên độ an toàn δ > 0. "
        "Đặt d = X<sub>g</sub> − X<sub>s</sub> là vector hướng nối điểm đầu và điểm đích, đoạn thẳng nối trực tiếp được tham số hóa bởi X(t) = X<sub>s</sub> + t·d với t ∈ [0, 1]. Để phát hiện các chướng ngại vật cản trở đoạn thẳng này và tạo các điểm nút tránh né (detour waypoints), tâm C<sub>k</sub> của từng hình cầu được chiếu vuông góc lên đoạn thẳng thông qua tham số kẹp trong đoạn [0, 1]:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "t<sub>k</sub><sup>*</sup> = clip<sub>[0, 1]</sub>( [(C<sub>k</sub> − X<sub>s</sub>) · d] / ||d||<sup>2</sup> ), &nbsp;&nbsp;&nbsp;&nbsp; P<sub>k</sub><sup>*</sup> = X<sub>s</sub> + t<sub>k</sub><sup>*</sup> · d &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 1)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "trong đó P<sub>k</sub><sup>*</sup> là điểm trên đoạn thẳng X<sub>s</sub>X<sub>g</sub> nằm gần tâm chướng ngại vật C<sub>k</sub> nhất. Khoảng cách từ tâm vật cản đến đoạn thẳng là d<sub>k</sub><sup>*</sup> = ||P<sub>k</sub><sup>*</sup> − C<sub>k</sub>||. Nếu d<sub>k</sub><sup>*</sup> &lt; r<sub>k</sub> + δ (tức đoạn thẳng xâm phạm vùng an toàn của vật cản thứ k), một điểm nút tránh né ứng viên w<sub>k</sub> (detour waypoint) được sinh ra bằng cách đẩy điểm P<sub>k</sub><sup>*</sup> ra ngoài theo phương bán kính từ C<sub>k</sub> qua P<sub>k</sub><sup>*</sup>:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "w<sub>k</sub> = P<sub>k</sub><sup>*</sup> + [ (r<sub>k</sub> + δ) − d<sub>k</sub><sup>*</sup> + ε ] · (P<sub>k</sub><sup>*</sup> − C<sub>k</sub>) / ||P<sub>k</sub><sup>*</sup> − C<sub>k</sub>|| &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 2)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "với ε &gt; 0 là một khoảng dự trữ nhỏ bổ sung (ví dụ ε = 0.02 m) để đảm bảo điểm nút nằm hoàn toàn bên ngoài mặt cầu đã mở rộng. Các điểm tránh né được giữ lại và sắp xếp theo thứ tự tăng dần của t<sub>k</sub><sup>*</sup> để tạo thành dãy điểm nút có thứ tự W = {X<sub>s</sub>, w<sub>1</sub>, ..., w<sub>m</sub>, X<sub>g</sub>} với m ≤ 3. "
        "Tiếp theo, một đường cong B-spline bậc ba (p = 3) liên tục cấp hai (C<sup>2</sup>) trong không gian ℝ<sup>3</sup> được nội suy đi qua tập điểm nút W:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "C(u) = ∑<sub>i=0</sub><sup>n</sup> N<sub>i, p</sub>(u) · P<sub>i</sub>, &nbsp;&nbsp;&nbsp;&nbsp; u ∈ [u<sub>0</sub>, u<sub>n</sub>] &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 3)",
        styles["EqBox"]
    ))
    pts_fig_path = "/tmp/paper_vn_figs/minh_hoa_cac_diem_eq1_eq2_eq3.png"
    if os.path.exists(pts_fig_path):
        story.append(Spacer(1, 3))
        story.append(Image(pts_fig_path, width=180 * mm, height=137.8 * mm))
        story.append(Paragraph(
            "Hình minh họa 2.1 (Bổ sung trực quan cho Phương trình 1, 2, 3): (a) Sơ đồ nguyên lý chiếu vuông góc tâm C<sub>k</sub> lên đoạn X<sub>s</sub>X<sub>g</sub> tại P<sub>k</sub><sup>*</sup> và đẩy hướng kính ra điểm nút tránh né w<sub>k</sub>; "
            "(b) Tọa độ thực tế trên mặt phẳng XY của 3 vật cản C<sub>1..3</sub>, điểm chiếu P<sub>1..3</sub><sup>*</sup>, điểm tránh né w<sub>1..3</sub> và đường cong B-spline C(u); "
            "(c) Phối cảnh không gian 3D; (d) Bảng giải thích ký hiệu &amp; tọa độ.",
            styles["Caption"]
        ))
        story.append(Spacer(1, 4))
    story.append(Paragraph(
        "trong đó {P<sub>i</sub>}<sub>i=0</sub><sup>n</sup> ∈ ℝ<sup>3</sup> là các điểm điều khiển B-spline và {N<sub>i, p</sub>(u)} là các hàm cơ sở B-spline bậc p = 3 được định nghĩa trên vector nút không giảm U (sử dụng tham số hóa độ dài dây cung tích lũy kèm điều kiện biên tự nhiên để triệt tiêu độ cong bậc hai tại hai đầu mút). "
        "Bởi vì tham số hình học u của spline không tỷ lệ tuyến tính với quãng đường vật lý, việc lấy mẫu đều theo u sẽ gây ra sự biến thiên vận tốc không mong muốn. Do đó, bài báo thực hiện <b>tái tham số hóa theo độ dài cung (arc-length reparameterization)</b> bằng cách tính độ dài cung tích lũy S(u):",
        styles["Body"]
    ))
    story.append(Paragraph(
        "S(u) = ∫<sub>u<sub>0</sub></sub><sup>u</sup> ||C'(ξ)|| dξ, &nbsp;&nbsp;&nbsp;&nbsp; S<sub>tot</sub> = S(u<sub>n</sub>) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 4)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Áp đặt quy luật vận tốc dài đầu công tác không đổi trên toàn bộ thời gian chuyển động cố định T<sub>tot</sub> = 2 s, quỹ đạo đầu công tác theo thời gian X(t) được xác định bởi:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "X(t) = C(u(t)), &nbsp;&nbsp; 0 ≤ t ≤ T<sub>tot</sub>, &nbsp;&nbsp; trong đó &nbsp; S(u(t)) = (S<sub>tot</sub> / T<sub>tot</sub>) · t &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 5)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Tại các thời điểm lấy mẫu rời rạc t<sub>k</sub> = k·Δt (k = 0, ..., K), vị trí đầu công tác mong muốn x(t<sub>k</sub>) ∈ ℝ<sup>3</sup> và hướng mong muốn R<sub>0</sub> ∈ SO(3) (được giữ cố định dọc theo quỹ đạo) được ánh xạ sang cấu hình khớp q<sub>k</sub> ∈ ℝ<sup>6</sup> thông qua <b>bộ giải Động học nghịch Bình phương tối thiểu có hệ số cản (Damped Least-Squares Inverse Kinematics - DLS IK)</b>. Tại mỗi bước lặp nội bộ, vector sai số tư thế 6 chiều e ∈ ℝ<sup>6</sup> gồm sai số vị trí e<sub>p</sub> và sai số định hướng trục-góc e<sub>o</sub> được tính bởi:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "e = [ e<sub>p</sub><sup>T</sup> &nbsp;&nbsp; e<sub>o</sub><sup>T</sup> ]<sup>T</sup> ∈ ℝ<sup>6</sup>, &nbsp;&nbsp;&nbsp; e<sub>p</sub> = x(t<sub>k</sub>) − x(q), &nbsp;&nbsp;&nbsp; e<sub>o</sub> = ω · θ &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 6)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Số gia góc khớp Δq được tính thông qua ma trận Jacobian hình học J(q) ∈ ℝ<sup>6×6</sup> và hệ số cản λ &gt; 0, sau đó kẹp trực tiếp vào miền giới hạn vật lý của khớp [q<sub>min</sub>, q<sub>max</sub>]:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Δq = J<sup>T</sup> (J · J<sup>T</sup> + λ<sup>2</sup> I<sub>6</sub>)<sup>−1</sup> e, &nbsp;&nbsp;&nbsp;&nbsp; q ← Π<sub>[q<sub>min</sub>, q<sub>max</sub>]</sub>(q + Δq) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 7)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Để đánh giá toàn diện chất lượng động học và động lực học của quỹ đạo thu được, ba phiếm hàm vô hướng (scalar performance functionals) được định nghĩa trên khoảng thời gian [0, T<sub>tot</sub>]:<br/>"
        "• <b>1. Chiều dài hình học đường đi đầu công tác (Geometric EE trajectory length L):</b>",
        styles["Body"]
    ))
    story.append(Paragraph(
        "L = ∫<sub>0</sub><sup>T<sub>tot</sub></sup> ||Ẋ(t)|| dt &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 8)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "• <b>2. Năng lượng tiêu thụ ước lượng của cơ cấu chấp hành (Actuator energy consumption E):</b> Dựa trên mô hình công suất cơ-điện của từng động cơ khớp, trong đó mỗi khớp thứ i có công suất định mức P<sub>i</sub> [W] và vận tốc góc danh định ω<sub>i</sub> [rad/s], tạo thành hệ số tiêu thụ năng lượng trên mỗi đơn vị góc quay e<sub>i</sub> = P<sub>i</sub> / ω<sub>i</sub> [J/rad]:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "E = ∑<sub>i=1</sub><sup>6</sup> e<sub>i</sub> ∫<sub>0</sub><sup>T<sub>tot</sub></sup> |q̇<sub>i</sub>(t)| dt &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 9)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "• <b>3. Độ trơn không gian khớp / Phiếm hàm tích phân bình phương độ giật (Joint-space smoothness / Jerk functional J):</b> Phạt nặng các dao động tần số cao và sự thay đổi gia tốc đột ngột gây rung lắc cơ khí:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "J = ∫<sub>0</sub><sup>T<sub>tot</sub></sup> ||q⃛(t)||<sup>2</sup> dt &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 10)",
        styles["EqBox"]
    ))

    # --- MỤC 2.2: KHUNG QUY HOẠCH HAI GIAI ĐOẠN ---
    story.append(Paragraph("2.2 Quy hoạch đường đi kết hợp Bi-RRT và tối ưu hóa siêu phỏng sinh học (Bi-RRT and metaheuristic optimization)", styles["H2"]))
    story.append(Paragraph(
        "Khung quy hoạch đề xuất hoạt động theo hai giai đoạn nối tiếp: <b>Giai đoạn 1</b> sử dụng Bi-RRT có dẫn hướng bởi đường cong B-spline kèm rút gọn đường đi để tìm kiếm nhanh một đường đi khả thi không va chạm toàn cục; <b>Giai đoạn 2</b> xây dựng các hành lang an toàn xung quanh các điểm nút và sử dụng WGA hoặc GWO để tối ưu hóa vị trí các điểm nút nhằm cực tiểu hóa hàm mục tiêu tổng hợp (L, E, J).",
        styles["Body"]
    ))

    story.append(Paragraph("Giai đoạn 1: Thuật toán Bi-RRT dẫn hướng bởi Spline kèm rút gọn đường đi (Stage 1: Spline-guided Bi-RRT with short-cutting)", styles["H3"]))
    story.append(Paragraph(
        "Không gian cấu hình C = [q<sub>min,1</sub>, q<sub>max,1</sub>] × ... × [q<sub>min,6</sub>, q<sub>max,6</sub>] ⊂ ℝ<sup>6</sup> có biên độ dải khớp khác nhau giữa các trục. Để tránh việc các khớp có dải quay lớn chi phối khoảng cách Euclid, bài báo sử dụng <b>metric khoảng cách có trọng số chuẩn hóa theo dải khớp</b>:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "r<sub>i</sub> = q<sub>max, i</sub> − q<sub>min, i</sub>, &nbsp;&nbsp; W = diag(r<sub>1</sub><sup>−1</sup>, ..., r<sub>6</sub><sup>−1</sup>), &nbsp;&nbsp; d(q<sub>start</sub>, q<sub>goal</sub>) = ||W (q<sub>start</sub> − q<sub>goal</sub>)||<sub>2</sub> &nbsp;&nbsp;&nbsp; (Phương trình 11)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Hai cây tìm kiếm T<sub>a</sub> (gốc tại q<sub>s</sub>) và T<sub>b</sub> (gốc tại q<sub>g</sub>) được phát triển luân phiên. Thay vì chỉ lấy mẫu đồng đều trên toàn bộ không gian C, một <b>chính sách lấy mẫu hỗn hợp (mixture sampling policy)</b> được áp dụng: với xác suất p<sub>tube</sub> ∈ [0.5, 0.8], bộ lập kế hoạch chọn ngẫu nhiên một điểm trên đường cong B-spline danh định (từ Mục 2.1), giải DLS IK để lấy cấu hình khớp tương ứng rồi cộng thêm nhiễu Gauss trung bình bằng 0 trong không gian khớp; với xác suất 1 − p<sub>tube</sub>, bộ lập kế hoạch lấy mẫu đồng đều trong C (có xác suất thiên kiến đích p<sub>g</sub> ∈ [0.05, 0.2]). "
        "Khi mở rộng cây từ nút gần nhất q<sub>near</sub> về phía mẫu q<sub>rand</sub> với bước tiến tối đa η ∈ [0.05, 0.2], cấu hình mới q<sub>new</sub> được xác định bởi:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "q<sub>new</sub> = q<sub>near</sub> + γ · (q<sub>rand</sub> − q<sub>near</sub>), &nbsp;&nbsp;&nbsp;&nbsp; γ = min( 1, &nbsp; η / d(q<sub>near</sub>, q<sub>rand</sub>) ) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 12)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Sau khi hai cây T<sub>a</sub> và T<sub>b</sub> kết nối thành công tạo ra đường đi thô P = {q<sub>0</sub>, q<sub>1</sub>, ..., q<sub>M</sub>}, một bước <b>rút gọn đường đi bảo toàn không va chạm (collision-preserving short-cutting)</b> được thực hiện: chọn ngẫu nhiên hai chỉ số không liền kề i &lt; j, kiểm tra va chạm dọc theo đoạn nội suy tuyến tính giữa q<sub>i</sub> và q<sub>j</sub>; nếu hoàn toàn tự do va chạm, toàn bộ các nút trung gian {q<sub>i+1</sub>, ..., q<sub>j−1</sub>} được loại bỏ.",
        styles["Body"]
    ))

    story.append(Paragraph(
        "<b>THUẬT TOÁN 1: Bi-RRT (RRT-Connect) dẫn hướng bởi Spline kèm rút gọn đường đi (Dịch nguyên bản Algorithm 1 trang 8)</b><br/>"
        "• <b>Đầu vào (Input):</b> Cấu hình đầu q<sub>s</sub>, cấu hình đích q<sub>g</sub>, giới hạn khớp [q<sub>min</sub>, q<sub>max</sub>], tập vật cản {(C<sub>k</sub>, r<sub>k</sub>)}, biên an toàn δ, đường cong B-spline danh định C<sub>nom</sub>(u), xác suất lấy mẫu ống spline p<sub>tube</sub>, xác suất hướng đích p<sub>g</sub>, bước tiến η, số vòng lặp tối đa N<sub>max</sub>.<br/>"
        "• <b>Đầu ra (Output):</b> Đường đi khớp không va chạm đã rút gọn P<sub>short</sub> = {q<sub>0</sub>, ..., q<sub>M'</sub>}.<br/>"
        "1: Khởi tạo hai cây T<sub>a</sub> ← {q<sub>s</sub>}, T<sub>b</sub> ← {q<sub>g</sub>}.<br/>"
        "2: <b>for</b> k = 1 <b>to</b> N<sub>max</sub> <b>do</b><br/>"
        "3: &nbsp;&nbsp;&nbsp; <b>if</b> rand() &lt; p<sub>g</sub> <b>then</b> q<sub>rand</sub> ← q<sub>g</sub> (hoặc gốc của cây đối diện)<br/>"
        "4: &nbsp;&nbsp;&nbsp; <b>else if</b> rand() &lt; p<sub>tube</sub> <b>then</b> Lấy mẫu u ~ U[0, 1] trên C<sub>nom</sub>(u), giải q<sub>nom</sub> = DLS_IK(C<sub>nom</sub>(u)), gán q<sub>rand</sub> ← Π<sub>[q<sub>min</sub>, q<sub>max</sub>]</sub>(q<sub>nom</sub> + N(0, Σ<sub>tube</sub>))<br/>"
        "5: &nbsp;&nbsp;&nbsp; <b>else</b> Lấy mẫu đồng đều q<sub>rand</sub> ~ U[q<sub>min</sub>, q<sub>max</sub>]<br/>"
        "6: &nbsp;&nbsp;&nbsp; Tìm nút gần nhất q<sub>near</sub> ∈ T<sub>a</sub> theo metric khoảng cách có trọng số d(·, ·) ở Phương trình (11).<br/>"
        "7: &nbsp;&nbsp;&nbsp; Tính q<sub>new</sub> bằng phép tiến bước giới hạn η theo Phương trình (12).<br/>"
        "8: &nbsp;&nbsp;&nbsp; <b>if</b> đoạn nối (q<sub>near</sub>, q<sub>new</sub>) không va chạm <b>then</b><br/>"
        "9: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Thêm q<sub>new</sub> vào T<sub>a</sub> với nút cha là q<sub>near</sub>.<br/>"
        "10: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Mở rộng liên tục cây T<sub>b</sub> về phía q<sub>new</sub> (Connect(T<sub>b</sub>, q<sub>new</sub>)) bằng Phương trình (12) cho đến khi kết nối hoặc gặp vật cản.<br/>"
        "11: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>if</b> T<sub>a</sub> và T<sub>b</sub> kết nối thành công <b>then</b> Trích xuất đường đi P = {q<sub>0</sub>, ..., q<sub>M</sub>} và thoát vòng lặp.<br/>"
        "12: &nbsp;&nbsp; Hoán đổi vai trò hai cây: Swap(T<sub>a</sub>, T<sub>b</sub>).<br/>"
        "13: <b>end for</b><br/>"
        "14: <b>Bước rút gọn (Short-cutting):</b> Lặp N<sub>sc</sub> lần: chọn hai chỉ số i + 1 &lt; j; nếu đoạn thẳng nối q<sub>i</sub> và q<sub>j</sub> không va chạm thì thay thế {q<sub>i</sub>, ..., q<sub>j</sub>} bằng {q<sub>i</sub>, q<sub>j</sub>}. Trả về P<sub>short</sub>.",
        styles["AlgoBox"]
    ))

    story.append(Paragraph("Giai đoạn 2: Xây dựng hành lang an toàn và Hàm mục tiêu tổng hợp (Stage 2: Collision-free corridors &amp; Composite objective)", styles["H3"]))
    story.append(Paragraph(
        "Mặc dù đường đi sau Giai đoạn 1 đã không còn va chạm, việc thực thi trực tiếp các đoạn nối gấp khúc gây ra gia tốc và độ giật (jerk) rất lớn. Để làm trơn quỹ đạo mà vẫn tuyệt đối không xâm phạm vật cản, Giai đoạn 2 định nghĩa các <b>hành lang hình cầu không va chạm (collision-free spherical corridors)</b> xung quanh từng điểm nút tránh né trung gian w<sub>i</sub> (i = 1, ..., m). Với bán kính vật cản mở rộng R<sub>k</sub> = r<sub>k</sub> + δ, khoảng cách tự do nhỏ nhất d<sub>i</sub> từ điểm nút w<sub>i</sub> đến các vật cản và bán kính hành lang an toàn ρ<sub>i</sub> được tính theo:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "d<sub>i</sub> = min<sub>k</sub> ( ||w<sub>i</sub> − c<sub>k</sub>|| − R<sub>k</sub> ), &nbsp;&nbsp;&nbsp;&nbsp; ρ<sub>i</sub> = α · d<sub>i</sub> &nbsp;&nbsp; (với α ∈ [0.5, 0.8]) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 13)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Mỗi cá thể ứng viên trong thuật toán tối ưu hóa được biểu diễn bởi vector biến quyết định θ = [Δw<sub>1</sub><sup>T</sup>, ..., Δw<sub>m</sub><sup>T</sup>]<sup>T</sup> ∈ ℝ<sup>3m</sup>, dịch chuyển các điểm nút trung gian thành w̃<sub>k</sub> = w<sub>k</sub> + Δw<sub>k</sub> sao cho ||Δw<sub>k</sub>|| ≤ ρ<sub>k</sub>. Từ tập điểm nút đã dịch chuyển W(θ) = {X<sub>s</sub>, w̃<sub>1</sub>, ..., w̃<sub>m</sub>, X<sub>g</sub>}, đường cong B-spline bậc ba C(u; θ) được tái tạo và tái tham số hóa theo độ dài cung với vận tốc dài không đổi trên khoảng thời gian [0, T]:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "s(u) = ∫<sub>0</sub><sup>u</sup> ||C'(ξ)|| dξ, &nbsp;&nbsp;&nbsp; S<sub>tot</sub> = s(1), &nbsp;&nbsp;&nbsp; x(t) = C(u(t)), &nbsp;&nbsp;&nbsp; s(u(t)) = (S<sub>tot</sub> / T) · t &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 14)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "||ẋ(t)|| = S<sub>tot</sub> / T, &nbsp;&nbsp;&nbsp;&nbsp; ∀ t ∈ [0, T] &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 15)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Gọi (L<sub>0</sub>, E<sub>0</sub>, J<sub>0</sub>) là các giá trị chiều dài, năng lượng và độ giật của quỹ đạo cơ sở ban đầu (baseline trajectory). Bởi vì ba đại lượng L [m], E [J] và J [rad<sup>2</sup>/s<sup>5</sup>] có thứ nguyên và bậc độ lớn chênh lệch nhau hàng triệu lần, bài báo định nghĩa <b>Hàm mục tiêu tổng hợp chuẩn hóa (Normalized Composite Objective Function)</b>:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Obj. Func. = w<sub>L</sub> · (L / L<sub>0</sub>) + w<sub>E</sub> · (E / E<sub>0</sub>) + w<sub>J</sub> · (J / J<sub>0</sub>), &nbsp;&nbsp;&nbsp;&nbsp; w<sub>L</sub> + w<sub>E</sub> + w<sub>J</sub> = 1 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 16)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "Bài toán tối ưu hóa phải thỏa mãn đồng thời các ràng buộc biên tại t = 0 và t = T, ràng buộc giới hạn khớp và ràng buộc khoảng cách an toàn có dấu σ(x(t)) ≥ 0 tại mọi thời điểm:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "x(0) = X<sub>s</sub>, &nbsp;&nbsp; x(T) = X<sub>g</sub>, &nbsp;&nbsp; q<sub>min</sub> ≤ q(t) ≤ q<sub>max</sub>, &nbsp;&nbsp; σ(x(t)) ≥ 0 &nbsp;&nbsp; ∀ t ∈ [0, T] &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 17)",
        styles["EqBox"]
    ))

    # --- WGA (ALGORITHM 2 & EQS 18-21) ---
    story.append(Paragraph("Bộ tối ưu hóa siêu phỏng sinh học 1: Thuật toán Di truyền Cá voi (Whale Genetic Algorithm - WGA)", styles["H3"]))
    story.append(Paragraph(
        "Thuật toán WGA lai ghép cơ chế săn mồi bằng lưới bong bóng của cá voi lưng gù (Whale Optimization Algorithm - WOA) với các toán tử tiến hóa của Thuật toán Di truyền (GA) gồm chọn lọc giải đấu (tournament selection), lai ghép theo bộ ba tọa độ điểm nút (waypoint-wise crossover) và đột biến Gauss co giãn theo bán kính hành lang (corridor-scaled Gaussian mutation). "
        "Gọi Pop<sub>j</sub><sup>(itr)</sup> là cá thể thứ j tại vòng lặp itr và Pop<sup>*(itr)</sup> là cá thể tốt nhất hiện tại (leader). Trong pha bao vây con mồi (encircling prey, khi p &lt; 0.5 và |A| &lt; 1), vị trí cá thể được cập nhật theo:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "D = | C ⊙ Pop<sup>*(itr)</sup> − Pop<sub>j</sub><sup>(itr)</sup> |, &nbsp;&nbsp;&nbsp;&nbsp; Pop<sub>j</sub><sup>(itr+1)</sup> = Pop<sup>*(itr)</sup> − A ⊙ D &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 18)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "A = 2 · a<sup>(itr)</sup> · r<sub>1</sub> − a<sup>(itr)</sup>, &nbsp;&nbsp;&nbsp;&nbsp; C = 2 · r<sub>2</sub>, &nbsp;&nbsp;&nbsp;&nbsp; a<sup>(itr)</sup> = 2 · (1 − itr / MaxItr) &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 19)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "trong đó r<sub>1</sub>, r<sub>2</sub> ~ U(0, 1)<sup>3m</sup> là các vector ngẫu nhiên, ⊙ là tích từng phần tử (Hadamard product), và hệ số a<sup>(itr)</sup> giảm tuyến tính từ 2 về 0. Nếu p &lt; 0.5 nhưng |A| ≥ 1, cá voi chuyển sang pha tìm kiếm con mồi toàn cục bằng cách thay Pop<sup>*(itr)</sup> bằng một cá thể chọn ngẫu nhiên trong quần thể. "
        "Ngược lại, khi p ≥ 0.5, cá voi cập nhật vị trí theo đường xoắn ốc logarit (spiral bubble-net attacking):",
        styles["Body"]
    ))
    story.append(Paragraph(
        "D' = | Pop<sup>*(itr)</sup> − Pop<sub>j</sub><sup>(itr)</sup> |, &nbsp;&nbsp;&nbsp;&nbsp; Pop<sub>j</sub><sup>(itr+1)</sup> = D' ⊙ e<sup>b·l</sup> ⊙ cos(2πl) + Pop<sup>*(itr)</sup> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 20)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "với b là hằng số xác định hình dạng xoắn ốc (b = 0.8) và l ~ U[−1, 1]. Sau bước cập nhật WOA, để ngăn chặn sự mất đa dạng quần thể và hội tụ sớm, WGA áp dụng pha tiến hóa di truyền: chọn các cá thể cha mẹ qua giải đấu, thực hiện lai ghép từng khối tọa độ điểm nút (Δw<sub>k,x</sub>, Δw<sub>k,y</sub>, Δw<sub>k,z</sub>) với xác suất p<sub>c</sub> = 0.8, và cộng đột biến Gauss co giãn theo bán kính hành lang với xác suất p<sub>m</sub> = 0.2:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "Δw<sub>k</sub><sup>mut</sup> = Π<sub>||·|| ≤ ρ<sub>k</sub></sub>( Δw<sub>k</sub> + N(0, σ<sub>k</sub><sup>2</sup> I<sub>3</sub>) ), &nbsp;&nbsp;&nbsp;&nbsp; σ<sub>k</sub> = β<sub>mut</sub> · ρ<sub>k</sub> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 21)",
        styles["EqBox"]
    ))

    story.append(Paragraph(
        "<b>THUẬT TOÁN 2: Thuật toán Di truyền Cá voi (Whale Genetic Algorithm - WGA) cho tối ưu hóa điểm nút B-spline (Dịch nguyên bản Algorithm 2 trang 11)</b><br/>"
        "• <b>Đầu vào (Input):</b> Các điểm nút cơ sở {w<sub>1</sub>, ..., w<sub>m</sub>}, bán kính hành lang {ρ<sub>1</sub>, ..., ρ<sub>m</sub>}, các giá trị chuẩn hóa (L<sub>0</sub>, E<sub>0</sub>, J<sub>0</sub>), trọng số (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>), kích thước quần thể N<sub>pop</sub>, số vòng lặp tối đa MaxItr, hằng số xoắn ốc b, xác suất lai ghép p<sub>c</sub>, xác suất đột biến p<sub>m</sub>.<br/>"
        "• <b>Đầu ra (Output):</b> Vector dịch chuyển điểm nút tối ưu θ<sup>*</sup> = Pop<sup>*</sup> và quỹ đạo B-spline tối ưu.<br/>"
        "1: Khởi tạo quần thể {Pop<sub>j</sub><sup>(0)</sup>}<sub>j=1</sub><sup>N<sub>pop</sub></sup> ngẫu nhiên bên trong các hình cầu hành lang ||Δw<sub>k</sub>|| ≤ ρ<sub>k</sub> (bao gồm nghiệm cơ sở Δw = 0).<br/>"
        "2: Đánh giá hàm mục tiêu (Phương trình 16) cho từng cá thể và xác định cá thể tốt nhất Pop<sup>*(0)</sup>.<br/>"
        "3: <b>for</b> itr = 1 <b>to</b> MaxItr <b>do</b><br/>"
        "4: &nbsp;&nbsp;&nbsp; Cập nhật hệ số giảm tuyến tính a<sup>(itr)</sup> = 2 · (1 − itr / MaxItr).<br/>"
        "5: &nbsp;&nbsp;&nbsp; <b>for</b> mỗi cá thể j = 1, ..., N<sub>pop</sub> <b>do</b><br/>"
        "6: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Sinh các số ngẫu nhiên r<sub>1</sub>, r<sub>2</sub> ∈ [0, 1], l ∈ [−1, 1], p ∈ [0, 1]; tính A và C theo Phương trình (19).<br/>"
        "7: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>if</b> p &lt; 0.5 <b>then</b><br/>"
        "8: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>if</b> |A| &lt; 1 <b>then</b> Cập nhật bao vây con mồi theo Pop<sup>*(itr)</sup> bằng Phương trình (18).<br/>"
        "9: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>else</b> Chọn cá thể ngẫu nhiên Pop<sub>rand</sub> và cập nhật tìm kiếm toàn cục.<br/>"
        "10: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <b>else</b> Cập nhật xoắn ốc bong bóng theo Phương trình (20).<br/>"
        "11: &nbsp;&nbsp; <b>end for</b><br/>"
        "12: &nbsp;&nbsp; <b>Pha Di truyền (GA Phase):</b> Chọn lọc giải đấu, lai ghép theo từng bộ ba tọa độ điểm nút với xác suất p<sub>c</sub>, đột biến Gauss theo hành lang (Phương trình 21) với xác suất p<sub>m</sub>, và chiếu kẹp mọi điểm nút về hành lang ||Δw<sub>k</sub>|| ≤ ρ<sub>k</sub>.<br/>"
        "13: &nbsp;&nbsp; Đánh giá lại quần thể, giữ lại các cá thể ưu tú (Elitism) và cập nhật Pop<sup>*(itr)</sup>.<br/>"
        "14: <b>end for</b><br/>"
        "15: Trả về Pop<sup>*</sup> và tái tạo quỹ đạo B-spline cuối cùng.",
        styles["AlgoBox"]
    ))

    # --- GWO (ALGORITHM 3 & EQS 22-24) ---
    story.append(Paragraph("Bộ tối ưu hóa siêu phỏng sinh học 2: Thuật toán Tối ưu hóa Bầy sói xám (Grey Wolf Optimizer - GWO)", styles["H3"]))
    story.append(Paragraph(
        "Thuật toán Tối ưu hóa Bầy sói xám (GWO) mô phỏng cấu trúc phân cấp xã hội và hành vi săn mồi hợp tác của bầy sói xám. Trong mỗi vòng lặp itr, ba cá thể có giá trị hàm mục tiêu tốt nhất được xếp hạng lần lượt là sói đầu đàn <b>Alpha (GW<sub>α</sub>)</b>, sói phó thứ nhất <b>Beta (GW<sub>β</sub>)</b> và sói phó thứ hai <b>Delta (GW<sub>δ</sub>)</b>; các cá thể còn lại là sói <b>Omega (GW)</b>. Quá trình bao vây và săn mồi được dẫn dắt đồng thời bởi cả ba cá thể dẫn đầu thông qua các phương trình:",
        styles["Body"]
    ))
    story.append(Paragraph(
        "D<sub>α</sub> = | C<sub>1</sub> ⊙ GW<sub>α</sub><sup>(itr)</sup> − GW<sup>(itr)</sup> |, &nbsp;&nbsp; D<sub>β</sub> = | C<sub>2</sub> ⊙ GW<sub>β</sub><sup>(itr)</sup> − GW<sup>(itr)</sup> |, &nbsp;&nbsp; D<sub>δ</sub> = | C<sub>3</sub> ⊙ GW<sub>δ</sub><sup>(itr)</sup> − GW<sup>(itr)</sup> | &nbsp;&nbsp; (Phương trình 22)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "GW<sub>1</sub> = GW<sub>α</sub><sup>(itr)</sup> − A<sub>1</sub> ⊙ D<sub>α</sub>, &nbsp;&nbsp;&nbsp; GW<sub>2</sub> = GW<sub>β</sub><sup>(itr)</sup> − A<sub>2</sub> ⊙ D<sub>β</sub>, &nbsp;&nbsp;&nbsp; GW<sub>3</sub> = GW<sub>δ</sub><sup>(itr)</sup> − A<sub>3</sub> ⊙ D<sub>δ</sub> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 23)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "GW<sup>(itr+1)</sup> = ( GW<sub>1</sub> + GW<sub>2</sub> + GW<sub>3</sub> ) / 3 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; (Phương trình 24)",
        styles["EqBox"]
    ))
    story.append(Paragraph(
        "trong đó A<sub>m</sub> = 2·a·r<sub>1,m</sub> − a và C<sub>m</sub> = 2·r<sub>2,m</sub> (m ∈ {1, 2, 3}). Nhờ cơ chế lấy trung bình cộng đồng thuận từ ba nghiệm dẫn đầu (α, β, δ), GWO làm giảm phương sai bước nhảy khi a → 0, giúp hội tụ cực kỳ êm mượt vào vùng tâm của hành lang tối ưu mà không cần thêm toán tử đột biến phức tạp.",
        styles["Body"]
    ))

    story.append(Paragraph(
        "<b>THUẬT TOÁN 3: Thuật toán Tối ưu hóa Bầy sói xám (Grey Wolf Optimizer - GWO) cho tối ưu hóa đường đi B-spline (Dịch nguyên bản Algorithm 3 trang 13)</b><br/>"
        "• <b>Đầu vào (Input):</b> Điểm nút cơ sở {w<sub>1</sub>, ..., w<sub>m</sub>}, bán kính hành lang {ρ<sub>1</sub>, ..., ρ<sub>m</sub>}, (L<sub>0</sub>, E<sub>0</sub>, J<sub>0</sub>), trọng số (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>), kích thước bầy sói N<sub>pop</sub>, số vòng lặp tối đa MaxItr.<br/>"
        "• <b>Đầu ra (Output):</b> Vị trí sói đầu đàn tối ưu GW<sub>α</sub> và quỹ đạo B-spline tối ưu.<br/>"
        "1: Khởi tạo quần thể bầy sói {GW<sub>j</sub><sup>(0)</sup>}<sub>j=1</sub><sup>N<sub>pop</sub></sup> trong miền hành lang ||Δw<sub>k</sub>|| ≤ ρ<sub>k</sub>.<br/>"
        "2: Đánh giá hàm mục tiêu (Phương trình 16) cho toàn bộ quần thể; gán 3 cá thể tốt nhất lần lượt cho GW<sub>α</sub>, GW<sub>β</sub>, GW<sub>δ</sub>.<br/>"
        "3: <b>for</b> itr = 1 <b>to</b> MaxItr <b>do</b><br/>"
        "4: &nbsp;&nbsp;&nbsp; Cập nhật a = 2 · (1 − itr / MaxItr).<br/>"
        "5: &nbsp;&nbsp;&nbsp; <b>for</b> mỗi cá thể sói j = 1, ..., N<sub>pop</sub> <b>do</b><br/>"
        "6: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Tính khoảng cách D<sub>α</sub>, D<sub>β</sub>, D<sub>δ</sub> tới 3 con đầu đàn theo Phương trình (22).<br/>"
        "7: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Tính 3 vector vị trí hướng dẫn GW<sub>1</sub>, GW<sub>2</sub>, GW<sub>3</sub> theo Phương trình (23).<br/>"
        "8: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Cập nhật vị trí mới GW<sub>j</sub><sup>(itr+1)</sup> = (GW<sub>1</sub> + GW<sub>2</sub> + GW<sub>3</sub>) / 3 theo Phương trình (24).<br/>"
        "9: &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Chiếu kẹp từng điểm nút thứ k vào trong hình cầu hành lang bán kính ρ<sub>k</sub>.<br/>"
        "10: &nbsp;&nbsp; <b>end for</b><br/>"
        "11: &nbsp;&nbsp; Đánh giá lại hàm mục tiêu của tất cả các cá thể và cập nhật bộ ba dẫn đầu GW<sub>α</sub>, GW<sub>β</sub>, GW<sub>δ</sub>.<br/>"
        "12: <b>end for</b><br/>"
        "13: Trả về nghiệm tốt nhất GW<sub>α</sub>.",
        styles["AlgoBox"]
    ))

    # --- MỤC 3: KẾT QUẢ VÀ THẢO LUẬN ---
    story.append(Paragraph("3. KẾT QUẢ VÀ THẢO LUẬN (RESULTS AND DISCUSSION)", styles["H1"]))
    story.append(Paragraph(
        "Để kiểm chứng hiệu quả của khung quy hoạch hai giai đoạn đề xuất, các thí nghiệm mô phỏng được thực hiện trên cánh tay robot công nghiệp 6 bậc tự do (KUKA KR 4 R600 trong bài báo gốc) hoạt động trong không gian làm việc có ba chướng ngại vật hình cầu đặt giữa điểm đầu và điểm đích. Thời gian thực thi quỹ đạo được cố định ở T<sub>tot</sub> = 2.0 s. Bảng 2 và Bảng 3 dưới đây trình bày thông số động học Denavit–Hartenberg (DH), giới hạn góc khớp và hằng số năng lượng cơ cấu chấp hành e<sub>i</sub> [J/rad] của 6 khớp.",
        styles["Body"]
    ))

    # Bảng 2 & Bảng 3
    story.append(Paragraph("Bảng 2 & Bảng 3. Thông số động học Denavit–Hartenberg (Table 2) và Giới hạn góc khớp cùng hệ số năng lượng động cơ e<sub>i</sub> [J/rad] (Table 3 trang 14).", styles["Caption"]))
    t23_data = [
        [
            Paragraph("<b>Khớp (Joint)</b>", styles["TableHeader"]),
            Paragraph("<b>θ<sub>i</sub> [rad]</b>", styles["TableHeader"]),
            Paragraph("<b>d<sub>i</sub> [m]</b>", styles["TableHeader"]),
            Paragraph("<b>a<sub>i</sub> [m]</b>", styles["TableHeader"]),
            Paragraph("<b>α<sub>i</sub> [rad]</b>", styles["TableHeader"]),
            Paragraph("<b>Giới hạn dưới q<sub>min</sub> [°]</b>", styles["TableHeader"]),
            Paragraph("<b>Giới hạn trên q<sub>max</sub> [°]</b>", styles["TableHeader"]),
            Paragraph("<b>Hệ số năng lượng e<sub>i</sub> [J/rad] (Eq. 9)</b>", styles["TableHeader"]),
        ],
        [Paragraph("Khớp 1 (J1)", styles["TableCellCenter"]), Paragraph("q<sub>1</sub>", styles["TableCellCenter"]), Paragraph("0.330", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("+π/2", styles["TableCellCenter"]), Paragraph("−170°", styles["TableCellCenter"]), Paragraph("+170°", styles["TableCellCenter"]), Paragraph("85.0 J/rad", styles["TableCellCenter"])],
        [Paragraph("Khớp 2 (J2)", styles["TableCellCenter"]), Paragraph("q<sub>2</sub>", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("0.290", styles["TableCellCenter"]), Paragraph("0", styles["TableCellCenter"]), Paragraph("−195°", styles["TableCellCenter"]), Paragraph("+40°", styles["TableCellCenter"]), Paragraph("185.0 J/rad", styles["TableCellCenter"])],
        [Paragraph("Khớp 3 (J3)", styles["TableCellCenter"]), Paragraph("q<sub>3</sub>", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("0.020", styles["TableCellCenter"]), Paragraph("+π/2", styles["TableCellCenter"]), Paragraph("−115°", styles["TableCellCenter"]), Paragraph("+150°", styles["TableCellCenter"]), Paragraph("160.0 J/rad", styles["TableCellCenter"])],
        [Paragraph("Khớp 4 (J4)", styles["TableCellCenter"]), Paragraph("q<sub>4</sub>", styles["TableCellCenter"]), Paragraph("0.310", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("−π/2", styles["TableCellCenter"]), Paragraph("−185°", styles["TableCellCenter"]), Paragraph("+185°", styles["TableCellCenter"]), Paragraph("8.5 J/rad", styles["TableCellCenter"])],
        [Paragraph("Khớp 5 (J5)", styles["TableCellCenter"]), Paragraph("q<sub>5</sub>", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("+π/2", styles["TableCellCenter"]), Paragraph("−120°", styles["TableCellCenter"]), Paragraph("+120°", styles["TableCellCenter"]), Paragraph("5.2 J/rad", styles["TableCellCenter"])],
        [Paragraph("Khớp 6 (J6)", styles["TableCellCenter"]), Paragraph("q<sub>6</sub>", styles["TableCellCenter"]), Paragraph("0.075", styles["TableCellCenter"]), Paragraph("0.000", styles["TableCellCenter"]), Paragraph("0", styles["TableCellCenter"]), Paragraph("−350°", styles["TableCellCenter"]), Paragraph("+350°", styles["TableCellCenter"]), Paragraph("3.8 J/rad", styles["TableCellCenter"])],
    ]
    t23 = Table(t23_data, colWidths=[23*mm, 16*mm, 18*mm, 18*mm, 18*mm, 27*mm, 27*mm, 33*mm], repeatRows=1)
    t23.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ]))
    story.append(t23)
    story.append(Spacer(1, 6))

    story.append(Paragraph("Phân tích 4 phương pháp quy hoạch được đánh giá trong Mục 3 của bài báo:", styles["H2"]))
    methods_desc = [
        "<b>Phương pháp 1 – Đường đi B-spline cơ sở (B-spline path baseline - Hình 2 &amp; Hình 6):</b> Xây dựng trực tiếp đường cong B-spline bậc 3 đi qua các điểm nút tránh né hình học ban đầu (Phương trình 1–5). Quỹ đạo có chiều dài ngắn L = 0.348 m, năng lượng E = 282.7 J và độ giật J = 1.29×10<sup>5</sup>, nhưng các điểm nút cố định theo hình học thuần túy chưa được tối ưu hóa về mặt động lực học và không có bước tìm kiếm toàn cục trong không gian cấu hình.",
        "<b>Phương pháp 2 – Bi-RRT kết hợp rút gọn đường đi (Bi-RRT + short-cutting - Hình 3):</b> Tìm kiếm đường đi không va chạm trong không gian khớp và cắt bỏ các nút dư thừa. Chiều dài đường đi đạt L = 0.3484 m và năng lượng theo tích phân vận tốc đạt E = 196.11 J, nhưng do bản chất nối đoạn thẳng gấp khúc tại các điểm chuyển hướng nên gây ra <b>độ giật cực đại J = 2.23×10<sup>6</sup></b> (gấp hơn 17 lần so với B-spline), gây nguy cơ rung lắc mạnh và quá tải cơ cấu truyền động.",
        "<b>Phương pháp 3 – Khung đề xuất Bi-RRT + WGA (Hình 4 &amp; Hình 7):</b> Sử dụng kết quả khả thi của Bi-RRT làm hạt giống và tối ưu hóa các điểm nút B-spline trong hành lang an toàn bằng Thuật toán Di truyền Cá voi (WGA). Độ giật giảm mạnh từ 2.23×10<sup>6</sup> xuống còn <b>J = 1.25×10<sup>5</sup> (giảm 94.4%)</b>, với chiều dài L = 0.402 m, năng lượng E = 398.2 J và giá trị hàm mục tiêu tổng hợp đạt <b>Fitness = 0.67751</b>.",
        "<b>Phương pháp 4 – Khung đề xuất Bi-RRT + GWO (Hình 5 &amp; Hình 8):</b> Tối ưu hóa các điểm nút B-spline trong hành lang an toàn bằng Thuật toán Bầy sói xám (GWO). Nhờ cơ chế hội tụ đồng thuận ba cấp lãnh đạo (α, β, δ), GWO đạt được độ giật thấp nhất <b>J = 1.00×10<sup>5</sup> (giảm 95.6% so với Bi-RRT)</b>, chiều dài L = 0.407 m, năng lượng E = 398.3 J và giá trị hàm mục tiêu tốt nhất <b>Fitness = 0.67481</b>.",
    ]
    for md in methods_desc:
        story.append(Paragraph("• " + md, styles["Bullet"]))

    # Bảng 4
    story.append(Paragraph("Bảng 4. Bảng tổng hợp kết quả so sánh định lượng giữa 4 phương pháp (Dịch nguyên bản Table 4 trang 21).", styles["Caption"]))
    t4_data = [
        [
            Paragraph("<b>Phương pháp quy hoạch (Method)</b>", styles["TableHeader"]),
            Paragraph("<b>Chiều dài L [m] (Eq. 8)</b>", styles["TableHeader"]),
            Paragraph("<b>Năng lượng E [J] (Eq. 9)</b>", styles["TableHeader"]),
            Paragraph("<b>Độ giật Jerk J (Eq. 10)</b>", styles["TableHeader"]),
            Paragraph("<b>Mức giảm Jerk so với Bi-RRT</b>", styles["TableHeader"]),
            Paragraph("<b>Giá trị Fitness (Eq. 16)</b>", styles["TableHeader"]),
        ],
        [
            Paragraph("1. Đường đi B-spline cơ sở (B-spline path)", styles["TableCell"]),
            Paragraph("0.348 m", styles["TableCellCenter"]),
            Paragraph("282.7 J", styles["TableCellCenter"]),
            Paragraph("1.29 × 10<sup>5</sup>", styles["TableCellCenter"]),
            Paragraph("Cơ sở tham chiếu", styles["TableCellCenter"]),
            Paragraph("—", styles["TableCellCenter"]),
        ],
        [
            Paragraph("2. Bi-RRT + rút gọn (Bi-RRT + short-cutting)", styles["TableCell"]),
            Paragraph("0.3484 m", styles["TableCellCenter"]),
            Paragraph("196.11 J", styles["TableCellCenter"]),
            Paragraph("<b>2.23 × 10<sup>6</sup></b>", styles["TableCellCenter"]),
            Paragraph("0% (Gấp khúc C<sup>0</sup>)", styles["TableCellCenter"]),
            Paragraph("1.00000 (Seed)", styles["TableCellCenter"]),
        ],
        [
            Paragraph("<b>3. Đề xuất: Bi-RRT + WGA</b>", styles["TableCell"]),
            Paragraph("0.402 m", styles["TableCellCenter"]),
            Paragraph("398.2 J", styles["TableCellCenter"]),
            Paragraph("<b>1.25 × 10<sup>5</sup></b>", styles["TableCellCenter"]),
            Paragraph("<b>−94.4%</b>", styles["TableCellCenter"]),
            Paragraph("<b>0.67751</b>", styles["TableCellCenter"]),
        ],
        [
            Paragraph("<b>4. Đề xuất: Bi-RRT + GWO (Tốt nhất)</b>", styles["TableCell"]),
            Paragraph("0.407 m", styles["TableCellCenter"]),
            Paragraph("398.3 J", styles["TableCellCenter"]),
            Paragraph("<b>1.00 × 10<sup>5</sup></b>", styles["TableCellCenter"]),
            Paragraph("<b>−95.6%</b>", styles["TableCellCenter"]),
            Paragraph("<b>0.67481</b>", styles["TableCellCenter"]),
        ],
    ]
    t4 = Table(t4_data, colWidths=[48*mm, 25*mm, 25*mm, 28*mm, 28*mm, 26*mm], repeatRows=1)
    t4.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#fff1f2"), colors.HexColor("#fffbeb"), colors.HexColor("#ecfdf5")]),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.append(t4)
    story.append(Spacer(1, 5))

    story.append(Image(fig1_path, width=176 * mm, height=52 * mm))
    story.append(Paragraph("Hình minh họa 1 (Tương ứng Hình 9 & Bảng 4 trong bài báo): Đối sánh trực quan Chiều dài L, Năng lượng E và Độ giật J giữa 4 phương pháp.", styles["Caption"]))
    story.append(Image(fig2_path, width=165 * mm, height=66 * mm))
    story.append(Paragraph("Hình minh họa 2 (Tương ứng Hình 7e & Hình 8e trong bài báo): Đường cong hội tụ giá trị Hàm mục tiêu tổng hợp của Bi-RRT + WGA và Bi-RRT + GWO.", styles["Caption"]))

    # Bảng 5: Thời gian tính toán
    story.append(Paragraph("Bảng 5. So sánh thời gian chạy trung bình (Runtime comparison) qua 20 lần chạy độc lập (Dịch nguyên bản Table 5 trang 21).", styles["Caption"]))
    t5_data = [
        [
            Paragraph("<b>Phương pháp (Method)</b>", styles["TableHeader"]),
            Paragraph("<b>Thời gian chạy TB ± Độ lệch chuẩn [s]</b>", styles["TableHeader"]),
            Paragraph("<b>Nhận xét đặc trưng tính toán</b>", styles["TableHeader"]),
        ],
        [
            Paragraph("1. Đường đi B-spline cơ sở", styles["TableCell"]),
            Paragraph("0.03 ± 0.01 s", styles["TableCellCenter"]),
            Paragraph("Tính toán hình học trực tiếp rất nhanh, nhưng thiếu khám phá toàn cục và chưa tối ưu hóa động lực học.", styles["TableCell"]),
        ],
        [
            Paragraph("2. Bi-RRT + rút gọn đường đi", styles["TableCell"]),
            Paragraph("1.12 ± 0.18 s", styles["TableCellCenter"]),
            Paragraph("Khám phá không gian cấu hình nhanh, tạo hạt giống không va chạm nhưng độ giật J quá cao.", styles["TableCell"]),
        ],
        [
            Paragraph("3. Đề xuất: Bi-RRT + WGA", styles["TableCell"]),
            Paragraph("18.6 ± 2.4 s", styles["TableCellCenter"]),
            Paragraph("Bao gồm đánh giá lặp DLS IK trên lưới thời gian và các toán tử lai ghép/đột biến GA.", styles["TableCell"]),
        ],
        [
            Paragraph("<b>4. Đề xuất: Bi-RRT + GWO</b>", styles["TableCell"]),
            Paragraph("<b>14.9 ± 1.9 s (Nhanh hơn WGA 19.9%)</b>", styles["TableCellCenter"]),
            Paragraph("Cập nhật vector đơn giản hơn (không có bước chọn lọc giải đấu/lai ghép GA) và hội tụ nhanh hơn.", styles["TableCell"]),
        ],
    ]
    t5 = Table(t5_data, colWidths=[48*mm, 52*mm, 80*mm], repeatRows=1)
    t5.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
    ]))
    story.append(t5)
    story.append(Spacer(1, 5))

    # Bảng 6: Phân tích độ nhạy (Sensitivity Analysis)
    story.append(Paragraph("3.1 Phân tích độ nhạy của các tham số điều khiển (Sensitivity analysis - Mục 3 trang 21–22 &amp; Bảng 6)", styles["H2"]))
    story.append(Paragraph(
        "Để đánh giá tính bền vững của khung quy hoạch đối với sự thay đổi của các trọng số ưu tiên (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>) trong Phương trình (16) cũng như kích thước quần thể N<sub>pop</sub> và số vòng lặp MaxItr, bài báo thực hiện phân tích độ nhạy trên 5 cấu hình trọng số khác nhau. Kết quả trong Bảng 6 chứng minh rằng GWO luôn đạt độ giật J và giá trị Fitness tốt hơn hoặc tương đương WGA trên mọi cấu hình.",
        styles["Body"]
    ))
    story.append(Paragraph("Bảng 6. Phân tích độ nhạy theo bộ trọng số (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>) cho WGA và GWO (Dịch nguyên bản Table 6 trang 22).", styles["Caption"]))
    t6_data = [
        [
            Paragraph("<b>Kịch bản trọng số (w<sub>L</sub>, w<sub>E</sub>, w<sub>J</sub>)</b>", styles["TableHeader"]),
            Paragraph("<b>Thuật toán</b>", styles["TableHeader"]),
            Paragraph("<b>L [m]</b>", styles["TableHeader"]),
            Paragraph("<b>E [J]</b>", styles["TableHeader"]),
            Paragraph("<b>Độ giật J [rad<sup>2</sup>/s<sup>5</sup>]</b>", styles["TableHeader"]),
            Paragraph("<b>Fitness (Eq. 16)</b>", styles["TableHeader"]),
        ],
        [
            Paragraph("Cân bằng (Balanced): (0.33, 0.33, 0.34)", styles["TableCell"]),
            Paragraph("WGA / GWO", styles["TableCellCenter"]),
            Paragraph("0.402 / 0.407", styles["TableCellCenter"]),
            Paragraph("398.2 / 398.3", styles["TableCellCenter"]),
            Paragraph("1.25×10<sup>5</sup> / <b>1.00×10<sup>5</sup></b>", styles["TableCellCenter"]),
            Paragraph("0.67751 / <b>0.67481</b>", styles["TableCellCenter"]),
        ],
        [
            Paragraph("Ưu tiên chiều dài (Length-priority): (0.60, 0.20, 0.20)", styles["TableCell"]),
            Paragraph("WGA / GWO", styles["TableCellCenter"]),
            Paragraph("<b>0.379 / 0.381</b>", styles["TableCellCenter"]),
            Paragraph("365.4 / 368.1", styles["TableCellCenter"]),
            Paragraph("1.68×10<sup>5</sup> / 1.52×10<sup>5</sup>", styles["TableCellCenter"]),
            Paragraph("0.74120 / <b>0.73850</b>", styles["TableCellCenter"]),
        ],
        [
            Paragraph("Ưu tiên năng lượng (Energy-priority): (0.20, 0.60, 0.20)", styles["TableCell"]),
            Paragraph("WGA / GWO", styles["TableCellCenter"]),
            Paragraph("0.388 / 0.391", styles["TableCellCenter"]),
            Paragraph("<b>352.8 / 351.2</b>", styles["TableCellCenter"]),
            Paragraph("1.59×10<sup>5</sup> / 1.44×10<sup>5</sup>", styles["TableCellCenter"]),
            Paragraph("0.78910 / <b>0.78540</b>", styles["TableCellCenter"]),
        ],
        [
            Paragraph("Ưu tiên độ trơn/Jerk (Smoothness-priority): (0.20, 0.20, 0.60)", styles["TableCell"]),
            Paragraph("WGA / GWO", styles["TableCellCenter"]),
            Paragraph("0.412 / 0.415", styles["TableCellCenter"]),
            Paragraph("405.1 / 406.0", styles["TableCellCenter"]),
            Paragraph("1.12×10<sup>5</sup> / <b>0.94×10<sup>5</sup></b>", styles["TableCellCenter"]),
            Paragraph("0.55820 / <b>0.55390</b>", styles["TableCellCenter"]),
        ],
        [
            Paragraph("Cấu hình chuẩn thực thi ROS 2: (0.20, 0.35, 0.45)", styles["TableCell"]),
            Paragraph("WGA / GWO", styles["TableCellCenter"]),
            Paragraph("0.400 / 0.404", styles["TableCellCenter"]),
            Paragraph("389.5 / 390.1", styles["TableCellCenter"]),
            Paragraph("1.18×10<sup>5</sup> / <b>0.98×10<sup>5</sup></b>", styles["TableCellCenter"]),
            Paragraph("0.64210 / <b>0.63890</b>", styles["TableCellCenter"]),
        ],
    ]
    t6 = Table(t6_data, colWidths=[56*mm, 24*mm, 24*mm, 24*mm, 28*mm, 24*mm], repeatRows=1)
    t6.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1e3a8a")),
        ("GRID", (0, 0), (-1, -1), 0.5, colors.HexColor("#cbd5e1")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
        ("TOPPADDING", (0, 0), (-1, -1), 3.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3.5),
    ]))
    story.append(t6)
    story.append(Spacer(1, 6))

    # --- MỤC 4: KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN ---
    story.append(Paragraph("4. KẾT LUẬN VÀ HƯỚNG PHÁT TRIỂN (CONCLUSION AND FUTURE WORKS)", styles["H1"]))
    story.append(Paragraph(
        "Bài báo này đã trình bày một khung quy hoạch đường đi hai giai đoạn nhận biết chất lượng động lực học cho cánh tay robot 6 bậc tự do hoạt động trong môi trường có nhiều chướng ngại vật. Bằng cách kết hợp khả năng khám phá toàn cục nhanh của thuật toán Bi-RRT có dẫn hướng bởi đường cong B-spline ở Giai đoạn 1 với việc tối ưu hóa vị trí các điểm nút B-spline bên trong các hành lang an toàn bằng WGA và GWO ở Giai đoạn 2, phương pháp đề xuất đã khắc phục triệt để nhược điểm gấp khúc và độ giật cao của các bộ lập kế hoạch lấy mẫu truyền thống. "
        "Các kết quả thực nghiệm mô phỏng xác nhận rằng việc tinh chỉnh B-spline bằng WGA và GWO giúp <b>giảm tích phân bình phương độ giật không gian khớp lần lượt 94.4% và 95.6%</b> so với Bi-RRT có rút gọn đường đi, đồng thời đảm bảo tuyệt đối ràng buộc giới hạn khớp thông qua DLS IK và giữ an toàn va chạm trên toàn bộ hành trình. Giữa hai thuật toán siêu phỏng sinh học, GWO đạt độ trơn quỹ đạo cao hơn, giá trị Fitness tốt hơn (0.67481 so với 0.67751) và thời gian tính toán nhanh hơn (14.9 s so với 18.6 s).",
        styles["Body"]
    ))
    story.append(Paragraph(
        "<b>Hướng nghiên cứu trong tương lai (Future Works):</b> (1) Mở rộng khung quy hoạch sang môi trường có chướng ngại vật động (dynamic obstacles) và cập nhật đường cong B-spline theo thời gian thực; (2) Cho phép tối ưu hóa đồng thời cả định hướng đầu công tác R(t) ∈ SO(3) và tổng thời gian chuyển động T<sub>tot</sub>; (3) Tích hợp mô hình động lực học ngược đầy đủ (mô-men quán tính, lực Coriolis, trọng trường và ma sát khớp) cùng kiểm tra va chạm toàn thân bằng lưới hình học phức tạp trên phần cứng robot thực tế.",
        styles["Body"]
    ))

    # --- TÀI LIỆU THAM KHẢO CHÍNH ---
    story.append(Paragraph("TÀI LIỆU THAM KHẢO CHÍNH TRONG BÀI BÁO (SELECTED REFERENCES [1]–[40])", styles["H2"]))
    refs = [
        "[1] Siciliano, B., Sciavicco, L., Villani, L. & Oriolo, G. Robotics: Modelling, Planning and Control (Springer, 2009).",
        "[2] Gasparetto, A., Boscariol, P., Lanzutti, A. & Vidoni, R. Path planning and trajectory planning algorithms: A general overview. Motion and Operation Planning of Robotic Systems 3–27 (2015).",
        "[7] Kavraki, L. E., Svestka, P., Latombe, J.-C. & Overmars, M. H. Probabilistic roadmaps for path planning in high-dimensional configuration spaces. IEEE Trans. Robot. Autom. 12, 566–580 (1996).",
        "[8] LaValle, S. M. Rapidly-exploring random trees: A new tool for path planning. Tech. Rep. 98-11, Computer Science Dept., Iowa State University (1998).",
        "[9] Kuffner, J. J. & LaValle, S. M. RRT-Connect: An efficient approach to single-query path planning. Proc. IEEE Int. Conf. Robot. Autom. (ICRA) 995–1001 (2000).",
        "[11] Piegl, L. & Tiller, W. The NURBS Book (Springer, 1997).",
        "[12] Mirjalili, S. & Lewis, A. The whale optimization algorithm. Adv. Eng. Softw. 95, 51–67 (2016).",
        "[13] Mirjalili, S., Mirjalili, S. M. & Lewis, A. Grey wolf optimizer. Adv. Eng. Softw. 69, 46–61 (2014).",
        "[14]–[40] Các công trình nghiên cứu về Bi-RRT, B-spline, WGA và GWO cho cánh tay robot 6-DoF được trích dẫn đầy đủ trong nguyên bản Scientific Reports (2026) 16:7487, DOI: 10.1038/s41598-026-37676-8.",
    ]
    for r in refs:
        story.append(Paragraph(r, styles["Bullet"]))

    import shutil
    primary_out = output_paths[0]
    os.makedirs(os.path.dirname(primary_out), exist_ok=True)
    doc = SimpleDocTemplate(
        primary_out, pagesize=A4,
        leftMargin=15 * mm, rightMargin=15 * mm,
        topMargin=14 * mm, bottomMargin=16 * mm,
        title="Ban Dich Tieng Viet Bai Bao s41598-026-37676-8",
        author="Abdelrahman T. Elgohr et al. (Nature Scientific Reports 2026)"
    )
    doc.build(story, onFirstPage=add_page_number, onLaterPages=add_page_number)
    print(f"[OK] Exported Vietnamese translation PDF to: {primary_out}")
    for extra_path in output_paths[1:]:
        os.makedirs(os.path.dirname(extra_path), exist_ok=True)
        shutil.copy2(primary_out, extra_path)
        print(f"[OK] Copied Vietnamese translation PDF to: {extra_path}")


if __name__ == "__main__":
    outputs = [
        "/mnt/c/Users/thanh/OneDrive/Máy tính/Ban_Dich_Tieng_Viet_s41598-026-37676-8.pdf",
        "/mnt/c/Users/thanh/Desktop/Ban_Dich_Tieng_Viet_s41598-026-37676-8.pdf",
    ]
    build_pdf(outputs)
