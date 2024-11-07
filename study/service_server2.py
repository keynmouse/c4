### 서버 파일 (server.py) ###
import socket
import tkinter as tk
import threading

# 서버 스레드 함수
def server_thread():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 12345))
    server_socket.listen(5)
    print("Server is listening on port 12345...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr}")

        order_data = client_socket.recv(1024).decode()
        if order_data.startswith("ORDER"):
            _, product_name, quantity = order_data.split(":")
            server_app = ServerApp(client_socket, product_name, int(quantity))
            server_app.run()

# 서버 UI 클래스 정의
class ServerApp:
    def __init__(self, client_socket, product_name, quantity):
        self.client_socket = client_socket
        self.product_name = product_name
        self.quantity = quantity
        self.root = tk.Tk()
        self.root.title('Server Order Response')
        self.initUI()

    def initUI(self):
        # 주문 정보 레이블
        order_label = tk.Label(self.root, text=f"Order Received: {self.quantity} x {self.product_name}", font=("Arial", 16))
        order_label.pack(pady=10)

        # 수락 버튼
        accept_button = tk.Button(self.root, text="Accept Order", command=self.accept_order)
        accept_button.pack(pady=5)

        # 거절 버튼
        reject_button = tk.Button(self.root, text="Reject Order", command=self.reject_order)
        reject_button.pack(pady=5)

    def accept_order(self):
        self.client_socket.send(b"Order Accepted")
        self.root.destroy()

    def reject_order(self):
        self.client_socket.send(b"Order Rejected")
        self.root.destroy()

    def run(self):
        self.root.mainloop()

# 메인 실행 부분
if __name__ == "__main__":
    server = threading.Thread(target=server_thread)
    server.daemon = True
    server.start()
    server.join()
