import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel, QSpinBox, QPushButton, QDialog, QDialogButtonBox
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import Qt

class ConfirmationDialog(QDialog):
    def __init__(self, menu_items, parent=None):
        super().__init__(parent)
        self.setWindowTitle("선택 확인")
        self.setGeometry(150, 150, 300, 200)

        layout = QVBoxLayout()

        # 선택된 메뉴와 수량을 표시하는 레이블
        self.selection_label = QLabel(self)
        selection_text = "선택한 메뉴와 수량:\n"
        for menu, data in menu_items.items():
            if data["quantity"] > 0:
                selection_text += f"{menu}: {data['quantity']}개\n"
        self.selection_label.setText(selection_text)
        
        layout.addWidget(self.selection_label)

        # 확인 및 취소 버튼
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout.addWidget(buttons)

        self.setLayout(layout)

class MenuApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("메뉴 선택")
        self.setGeometry(100, 100, 400, 400)

        # 메인 레이아웃 (수직 방향)
        main_layout = QVBoxLayout()

        # 3x3 그리드 레이아웃
        self.grid_layout = QGridLayout()

        # 메뉴 목록과 수량을 저장할 딕셔너리
        self.menu_items = {
            "메뉴 1": {"quantity": 0, "image": "menu1.png"},
            "메뉴 2": {"quantity": 0, "image": "menu2.png"},
            "메뉴 3": {"quantity": 0, "image": "menu3.png"},
            "메뉴 4": {"quantity": 0, "image": "menu4.png"},
            "메뉴 5": {"quantity": 0, "image": "menu5.png"},
            "메뉴 6": {"quantity": 0, "image": "menu6.png"},
            "메뉴 7": {"quantity": 0, "image": "menu7.png"},
            "메뉴 8": {"quantity": 0, "image": "menu8.png"},
            "메뉴 9": {"quantity": 0, "image": "menu9.png"}
        }

        row = 0
        col = 0
        self.menu_labels = {}
        self.menu_spinboxes = {}

        for menu, data in self.menu_items.items():
            # 이미지 라벨 생성
            pixmap = QPixmap(data["image"])
            label = QLabel()
            label.setPixmap(pixmap.scaled(100, 100, Qt.KeepAspectRatio))  # 이미지 크기 조정
            label.setAlignment(Qt.AlignCenter)

            # 수량 스핀박스 생성
            spinbox = QSpinBox()
            spinbox.setRange(0, 10)  # 수량 범위 설정 (0부터 10까지)
            spinbox.setValue(0)  # 초기값 0
            spinbox.valueChanged.connect(self.update_quantity(menu, spinbox))

            self.menu_spinboxes[menu] = spinbox

            # 수량 레이블 만들기
            quantity_label = QLabel(f"수량: {data['quantity']}")
            self.menu_labels[menu] = quantity_label

            # 그리드에 이미지, 스핀박스 및 수량 레이블 추가
            self.grid_layout.addWidget(label, row, col)
            self.grid_layout.addWidget(spinbox, row, col + 1)
            self.grid_layout.addWidget(quantity_label, row, col + 2)

            # 행과 열 인덱스 업데이트
            col += 3
            if col >= 9:  # 3개씩 배치 후, 다음 행으로 이동
                col = 0
                row += 1

        # 확인 버튼
        self.button = QPushButton("확인")
        self.button.clicked.connect(self.show_confirmation_dialog)

        # 레이아웃에 위젯 추가
        main_layout.addLayout(self.grid_layout)
        main_layout.addWidget(self.button)

        self.setLayout(main_layout)
        
    def update_quantity(self, menu, spinbox):
        # 스핀박스 값이 변경될 때마다 해당 메뉴의 수량을 업데이트합니다.
        def _update():
            self.menu_items[menu]["quantity"] = spinbox.value()
            self.menu_labels[menu].setText(f"수량: {self.menu_items[menu]['quantity']}")
        return _update

    def show_confirmation_dialog(self):
        # 확인 버튼을 누르면 확인 창을 띄운다6
        dialog = ConfirmationDialog(self.menu_items, self)
        result = dialog.exec_()

        if result == QDialog.Accepted:
            print('선택을 확인했습니다.')
            print(self.menu_items)
        else:
            print("선택을 취소했습니다.")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MenuApp()
    window.show()
    sys.exit(app.exec_())

    
