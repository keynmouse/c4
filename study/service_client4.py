### 클라이언트 파일 (client.py) ###
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox, simpledialog
from std_srvs.srv import Trigger

# 클라이언트 노드 클래스 정의
class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')  # ROS2 노드 이름을 'client_node'로 설정
        self.client = self.create_client(Trigger, 'process_order')  # 서비스 클라이언트 생성 (Trigger 서비스 사용)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')  # 서비스가 준비될 때까지 대기

    def send_order(self, product_name, quantity):
        # 주문 메시지를 생성하여 서비스 호출
        order_message = f"ORDER:{product_name}:{quantity}"
        request = Trigger.Request()
        request.data = order_message
        future = self.client.call_async(request)  # 비동기 서비스 호출
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        # 서버로부터의 응답 메시지를 처리
        try:
            response = future.result().message
            messagebox.showinfo("Order Response", response)  # 응답을 팝업 창으로 표시
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

# 클라이언트 UI 클래스 정의
class ClientApp:
    def __init__(self, root, node):
        self.root = root  # Tkinter 루트 설정
        self.node = node  # ROS2 노드 설정
        self.root.title('Client Order System')  # 창 제목 설정
        self.initUI()  # UI 초기화

    def initUI(self):
        # 메뉴 항목 버튼 생성
        self.menu_items = ["Pizza", "Burger", "Pasta"]  # 메뉴 항목 리스트
        for item in self.menu_items:
            button = tk.Button(self.root, text=item, command=lambda i=item: self.select_menu_item(i))  # 버튼 생성 및 클릭 시 주문 선택 함수 호출
            button.pack(pady=5)  # 버튼을 레이아웃에 추가

    def select_menu_item(self, product_name):
        # 수량 입력 다이얼로그
        quantity = simpledialog.askinteger('Select Quantity', f'How many {product_name}s would you like?', minvalue=1, maxvalue=100)  # 수량 입력 창 생성
        if quantity:
            # 확인 메시지 박스
            if messagebox.askyesno('Confirm Order', f"You have selected {quantity} x {product_name}.\nDo you want to proceed?"):  # 주문 확인 창 표시
                self.node.send_order(product_name, quantity)  # 주문 전송

# 메인 실행 부분
def main(args=None):
    rclpy.init(args=args)  # ROS2 초기화
    client_node = ClientNode()  # 클라이언트 노드 생성

    root = tk.Tk()  # Tkinter 루트 생성
    client_app = ClientApp(root, client_node)  # 클라이언트 UI 생성

    # 스레드로 ROS2 스피너 실행
    def ros_spin():
        rclpy.spin(client_node)  # ROS2 노드 스피너 실행

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)  # 스레드를 이용하여 스피너 실행
    spinner_thread.start()

    root.mainloop()  # Tkinter 메인 루프 실행
    client_node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
