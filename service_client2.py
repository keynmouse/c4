### 클라이언트 파일 (client.py) ###
import socket
import tkinter as tk
from tkinter import messagebox, simpledialog

# 클라이언트 UI 클래스 정의
class ClientApp:
    def __init__(self, root):
        self.root = root
        self.root.title('Client Order System')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('localhost', 12345))
        self.initUI()

    def initUI(self):
        # 메뉴 항목 버튼 생성
        self.menu_items = ["Pizza", "Burger", "Pasta"]
        for item in self.menu_items:
            button = tk.Button(self.root, text=item, command=lambda i=item: self.select_menu_item(i))
            button.pack(pady=5)

    def select_menu_item(self, product_name):
        # 수량 입력 다이얼로그
        quantity = simpledialog.askinteger('Select Quantity', f'How many {product_name}s would you like?', minvalue=1, maxvalue=100)
        if quantity:
            # 확인 메시지 박스
            if messagebox.askyesno('Confirm Order', f"You have selected {quantity} x {product_name}.\nDo you want to proceed?"):
                self.send_order_to_server(product_name, quantity)

    def send_order_to_server(self, product_name, quantity):
        order_message = f"ORDER:{product_name}:{quantity}"
        self.client_socket.send(order_message.encode())
        response = self.client_socket.recv(1024).decode()
        messagebox.showinfo("Order Response", response)

# 메인 실행 부분
if __name__ == "__main__":
    root = tk.Tk()
    client_app = ClientApp(root)
    root.mainloop()
