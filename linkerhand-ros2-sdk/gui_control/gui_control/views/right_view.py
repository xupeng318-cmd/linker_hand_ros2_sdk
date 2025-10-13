import sys,os,time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QLineEdit, QPushButton, QWidget, QGridLayout, QScrollArea
)
from PyQt5.QtCore import Qt, pyqtSignal


class RightView(QMainWindow):
    add_button_handle = pyqtSignal(str)  # 定义一个信号
    handle_button_click = pyqtSignal(str)  # 定义一个信号
    def __init__(self,hand_joint="L20", hand_type="left",load_yaml=None):
        super().__init__()
        self.hand_joint = hand_joint
        self.hand_type = hand_type
        self.buttons = []
        self.yaml = load_yaml
        self.all_action = None
        self.all_action = self.yaml.load_action_yaml(hand_type=self.hand_type,hand_joint=self.hand_joint)
        self.setWindowTitle("按钮网格布局")
        self.setGeometry(100, 100, 600, 400)
        self.init_ui()
        self.init_buttons()

    def init_ui(self):
        # 主窗口容器
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        # 主布局
        self.main_layout = QVBoxLayout(main_widget)
        self.main_layout.setContentsMargins(10, 10, 10, 10)
        self.main_layout.setSpacing(10)

        # 输入框和添加按钮
        self.input_field = QLineEdit()
        self.add_button = QPushButton("添加")
        self.add_button.clicked.connect(self.add_button_to_list)

        # 输入框和按钮布局
        self.top_layout = QVBoxLayout()
        self.top_layout.setSpacing(10)
        self.top_layout.addWidget(self.input_field)
        self.top_layout.addWidget(self.add_button)

        # 添加到主布局
        self.main_layout.addLayout(self.top_layout)
        # 创建一个滚动区域用于按钮列表
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_widget = QWidget()
        self.scroll_layout = QGridLayout(self.scroll_widget)
        self.scroll_layout.setContentsMargins(10, 10, 10, 10)
        self.scroll_layout.setSpacing(10)
        self.scroll_layout.setAlignment(Qt.AlignTop | Qt.AlignLeft)  # 左对齐和上对齐
        self.scroll_area.setWidget(self.scroll_widget)

        # 添加滚动区域到主布局
        self.main_layout.addWidget(self.scroll_area)

        # 网格布局参数
        self.row = 0  # 当前行
        self.column = 0  # 当前列
        self.BUTTONS_PER_ROW = 2  # 每行按钮数量限制
    def init_buttons(self):
        if self.all_action == None:
            return
        for item in self.all_action:
            button = QPushButton(item["ACTION_NAME"])
            button.setFixedWidth(100)  # 可根据需要设置按钮宽度
            button.setFixedHeight(30)  # 可根据需要设置按钮高度
            #button.clicked.connect(lambda checked, action_pos=item["ACTION_POS"]: self.button_clicked.emit(action_pos))
            button.clicked.connect(lambda checked, text=item["ACTION_NAME"]: self.handle_button_click.emit(text))
            # 添加按钮到网格布局
            self.scroll_layout.addWidget(button, self.row, self.column, alignment=Qt.AlignLeft | Qt.AlignTop)

            # 更新行列位置
            self.column += 1
            if self.column >= self.BUTTONS_PER_ROW:  # 超过每行按钮数量时换行
                self.column = 0
                self.row += 1

    def add_button_to_list(self):
        text = self.input_field.text().strip()
        if text:
            
            button = QPushButton(text)
            button.setFixedWidth(100)  # 可根据需要设置按钮宽度
            button.setFixedHeight(30)  # 可根据需要设置按钮高度
            button.clicked.connect(lambda checked, text=text: self.handle_button_click.emit(text))
            # 添加按钮到网格布局
            self.scroll_layout.addWidget(button, self.row, self.column, alignment=Qt.AlignLeft | Qt.AlignTop)

            # 更新行列位置
            self.column += 1
            if self.column >= self.BUTTONS_PER_ROW:  # 超过每行按钮数量时换行
                self.column = 0
            self.input_field.clear()  # 清空输入框
            self.buttons.append(button)
            self.add_button_handle.emit(text)  # 发出信号
            

    def clear_scroll_layout(self):
        """清空 scroll_layout 中的所有小部件"""
        while self.scroll_layout.count():
            item = self.scroll_layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()
