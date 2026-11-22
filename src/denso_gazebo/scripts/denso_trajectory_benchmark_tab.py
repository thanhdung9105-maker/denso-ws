#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DENSO VS-6556 G-CODE TRAJECTORY MULTI-CONFIGURATION ENERGY OPTIMIZER
Tối ưu hóa năng lượng vẽ G-code đa cấu hình dựa trên mô-men quay RNEA, khối lượng khâu
và công suất định mức 6 động cơ [750W, 400W, 200W, 80W, 50W, 50W].
"""

from denso_gcode_energy_optimizer_tab import DensoGCodeEnergyOptimizerTab

# Alias tương thích
DensoTrajectoryBenchmarkTab = DensoGCodeEnergyOptimizerTab

if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
    app = QApplication(sys.argv)
    window = QWidget()
    layout = QVBoxLayout(window)
    tab = DensoGCodeEnergyOptimizerTab()
    layout.addWidget(tab)
    window.resize(1150, 780)
    window.setWindowTitle("Denso VS-6556 G-Code Trajectory Energy Optimizer")
    window.show()
    sys.exit(app.exec_())
