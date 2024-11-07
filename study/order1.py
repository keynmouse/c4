import sqlite3
from datetime import datetime
from PyQt5 import QtWidgets, QtGui
import sys

# 데이터베이스 연결(파일이 없으면 자동으로 생성됨)
conn = sqlite3.connect('orders.db')
cursor = conn.cursor()

# 테이블 생성 (존재하지 않을 경우)
cursor.execute('''
CREATE TABLE IF NOT EXISTS orders (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    product_name TEXT NOT NULL,
    quantity INTEGER NOT NULL,
    price REAL NOT NULL,
    order_date TEXT NOT NULL
)
''')

# 주문 내역 저장 함수
def save_order(product_name, quantity, price):
    order_date = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    cursor.execute('''
    INSERT INTO orders (product_name, quantity, price, order_date)
    VALUES (?, ?, ?, ?)
    ''', (product_name, quantity, price, order_date))
    conn.commit()
    print(f"Order saved: {product_name}, Quantity: {quantity}, Price: {price}")

# PyQt5 주문 애플리케이션 클래스 정의
class OrderApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 레이아웃 설정
        layout = QtWidgets.QVBoxLayout()

        # 메뉴 레이블
        menu_label = QtWidgets.QLabel("Menu")
        menu_label.setFont(QtGui.QFont("Arial", 16))
        layout.addWidget(menu_label)

        # 메뉴 항목 버튼 생성
        self.menu_buttons = []
        self.menu_items = [
            ("Pizza", 10.0),
            ("Burger", 8.0),
            ("Pasta", 12.0),
            ("Salad", 7.0),
            ("Sushi", 15.0),
            ("Steak", 20.0)
        ]
        for item, price in self.menu_items:
            button = QtWidgets.QPushButton(f"{item} - ${price}")
            button.clicked.connect(lambda checked, i=item, p=price: self.select_menu_item(i, p))
            layout.addWidget(button)
            self.menu_buttons.append(button)

        # 선택된 제품 이름 표시
        self.selected_product_label = QtWidgets.QLabel("Selected Item: None")
        layout.addWidget(self.selected_product_label)

        # 수량 입력
        self.quantity_input = QtWidgets.QSpinBox(self)
        self.quantity_input.setRange(1, 1000)
        layout.addWidget(self.quantity_input)

        # 저장 버튼
        save_button = QtWidgets.QPushButton("Save Order", self)
        save_button.clicked.connect(self.confirm_order)
        layout.addWidget(save_button)

        # 레이아웃 설정
        self.setLayout(layout)
        self.setWindowTitle('Food Delivery Order System')
        self.show()

    def select_menu_item(self, product_name, price):
        self.selected_product = product_name
        self.selected_price = price
        self.selected_product_label.setText(f"Selected Item: {product_name}")

    def confirm_order(self):
        # 입력된 값 가져오기
        if hasattr(self, 'selected_product'):
            product_name = self.selected_product
            quantity = self.quantity_input.value()
            price = self.selected_price * quantity

            # 주문 확인 메시지 박스
            reply = QtWidgets.QMessageBox.question(self, 'Confirm Order',
                                                   f"You have selected {quantity} x {product_name} for a total of ${price:.2f}.\nDo you want to proceed?",
                                                   QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)

            if reply == QtWidgets.QMessageBox.Yes:
                # 주문 내역 저장 함수 호출
                save_order(product_name, quantity, price)
                QtWidgets.QMessageBox.information(self, "Order Saved", "Your order has been saved successfully.")
        else:
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please select a product.")

# PyQt5 애플리케이션 실행
def main():
    app = QtWidgets.QApplication(sys.argv)
    order_app = OrderApp()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

# 연결 종료
conn.close()
