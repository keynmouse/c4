import sqlite3
from datetime import datetime
from PyQt5 import QtWidgets
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

# PyQt5 애플리케이션 클래스 정의
class OrderApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 레이아웃 설정
        layout = QtWidgets.QVBoxLayout()

        # 제품 이름 입력
        self.product_name_input = QtWidgets.QLineEdit(self)
        self.product_name_input.setPlaceholderText("Product Name")
        layout.addWidget(self.product_name_input)

        # 수량 입력
        self.quantity_input = QtWidgets.QSpinBox(self)
        self.quantity_input.setRange(1, 1000)
        layout.addWidget(self.quantity_input)

        # 가격 입력
        self.price_input = QtWidgets.QDoubleSpinBox(self)
        self.price_input.setRange(0.01, 10000.00)
        self.price_input.setDecimals(2)
        layout.addWidget(self.price_input)

        # 저장 버튼
        save_button = QtWidgets.QPushButton("Save Order", self)
        save_button.clicked.connect(self.save_order)
        layout.addWidget(save_button)

        # 레이아웃 설정
        self.setLayout(layout)
        self.setWindowTitle('Order Entry')
        self.show()

    def save_order(self):
        # 입력된 값 가져오기
        product_name = self.product_name_input.text()
        quantity = self.quantity_input.value()
        price = self.price_input.value()

        # 주문 내역 저장 함수 호출
        if product_name:
            save_order(product_name, quantity, price)
        else:
            QtWidgets.QMessageBox.warning(self, "Input Error", "Please enter a product name.")

# PyQt5 애플리케이션 실행
def main():
    app = QtWidgets.QApplication(sys.argv)
    order_app = OrderApp()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()

# 연결 종료
conn.close()
