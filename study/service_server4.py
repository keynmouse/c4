### 서버 파일 (server.py) ###
import rclpy
from rclpy.node import Node
import tkinter as tk
from std_srvs.srv import Trigger

# 서버 노드 클래스 정의
class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')  # ROS2 노드 이름을 'server_node'로 설정
        self.service = self.create_service(Trigger, 'process_order', self.order_callback)  # 서비스 서버 생성 (Trigger 서비스 사용)

    def order_callback(self, request, response):
        # 클라이언트로부터의 주문 메시지를 처리
        order_data = request.data
        if order_data.startswith("ORDER"):
            _, product_name, quantity = order_data.split(":")  # 주문 데이터 파싱
            server_app = ServerApp(self, product_name, int(quantity))  # 서버 UI 생성
            server_app.run()  # 서버 UI 실행
            response.message = self.server_response  # 서버 응답 메시지 설정
            response.success = True  # 응답 성공 여부 설정
        else:
            response.message = "Invalid order format."
            response.success = False
        return response

    def set_response(self, response):
        self.server_response = response  # 서버 응답 메시지 설정

# 서버 UI 클래스 정의
class ServerApp:
    def __init__(self, node, product_name, quantity):
        self.node = node  # ROS2 노드 설정
        self.product_name = product_name  # 주문한 제품 이름 설정
        self.quantity = quantity  # 주문 수량 설정
        self.root = tk.Tk()  # Tkinter 루트 생성
        self.root.title('Server Order Response')  # 창 제목 설정
        self.initUI()  # UI 초기화

    def initUI(self):
        # 주문 정보 레이블
        order_label = tk.Label(self.root, text=f"Order Received: {self.quantity} x {self.product_name}", font=("Arial", 16))  # 주문 정보 레이블 생성
        order_label.pack(pady=10)  # 레이블을 레이아웃에 추가

        # 수락 버튼
        accept_button = tk.Button(self.root, text="Accept Order", command=self.accept_order)  # 수락 버튼 생성 및 클릭 시 함수 호출
        accept_button.pack(pady=5)  # 버튼을 레이아웃에 추가

        # 거절 버튼
        reject_button = tk.Button(self.root, text="Reject Order", command=self.reject_order)  # 거절 버튼 생성 및 클릭 시 함수 호출
        reject_button.pack(pady=5)  # 버튼을 레이아웃에 추가

    def accept_order(self):
        # 주문 수락 시 응답 전송
        self.node.set_response("Order Accepted")  # 응답 메시지를 설정
        self.root.destroy()  # UI 종료

    def reject_order(self):
        # 주문 거절 시 응답 전송
        self.node.set_response("Order Rejected")  # 응답 메시지를 설정
        self.root.destroy()  # UI 종료

    def run(self):
        # Tkinter 메인 루프 실행
        self.root.mainloop()

# 메인 실행 부분
def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    server_node = ServerNode()  # 서버 노드 생성

    # 스레드로 ROS2 스피너 실행
    def ros_spin():
        rclpy.spin(server_node)  # ROS2 노드 스피너 실행

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)  # 스레드를 이용하여 스피너 실행
    spinner_thread.start()

    spinner_thread.join()  # 스레드 종료 대기
    server_node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
