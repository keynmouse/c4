### 서버 파일 (server.py) ###
import rclpy
from rclpy.node import Node
import tkinter as tk
from std_msgs.msg import String

# 서버 노드 클래스 정의
class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.subscription = self.create_subscription(
            String,
            'order_topic',
            self.order_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, 'order_response_topic', 10)
        self.subscription  # prevent unused variable warning

    def order_callback(self, msg):
        order_data = msg.data
        if order_data.startswith("ORDER"):
            _, product_name, quantity = order_data.split(":")
            server_app = ServerApp(self, product_name, int(quantity))
            server_app.run()

    def send_response(self, response):
        msg = String()
        msg.data = response
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent response: {response}')

# 서버 UI 클래스 정의
class ServerApp:
    def __init__(self, node, product_name, quantity):
        self.node = node
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
        self.node.send_response("Order Accepted")
        self.root.destroy()

    def reject_order(self):
        self.node.send_response("Order Rejected")
        self.root.destroy()

    def run(self):
        self.root.mainloop()

# 메인 실행 부분
def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()

    # 스레드로 ROS2 스피너 실행
    def ros_spin():
        rclpy.spin(server_node)

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)
    spinner_thread.start()

    spinner_thread.join()
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
