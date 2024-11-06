### 클라이언트 파일 (client.py) ###
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox, simpledialog
from std_msgs.msg import String

# 클라이언트 노드 클래스 정의
class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.publisher_ = self.create_publisher(String, 'order_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'order_response_topic',
            self.order_response_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def send_order(self, product_name, quantity):
        order_message = f"ORDER:{product_name}:{quantity}"
        msg = String()
        msg.data = order_message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent order: {order_message}')

    def order_response_callback(self, msg):
        response = msg.data
        messagebox.showinfo("Order Response", response)

# 클라이언트 UI 클래스 정의
class ClientApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title('Client Order System')
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
                self.node.send_order(product_name, quantity)

# 메인 실행 부분
def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()

    root = tk.Tk()
    client_app = ClientApp(root, client_node)

    # 스레드로 ROS2 스피너 실행
    def ros_spin():
        rclpy.spin(client_node)

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)
    spinner_thread.start()

    root.mainloop()
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()