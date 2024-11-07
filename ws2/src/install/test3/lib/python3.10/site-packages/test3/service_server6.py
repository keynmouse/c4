import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
from test3.srv import Order

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.service = self.create_service(Order, 'process_order', self.order_callback)
        self.server_response = None

    def order_callback(self, request, response):
        product_name = request.product_name
        quantity = request.quantity
        self.get_logger().info(f"Received order: {quantity} x {product_name}")
        self.create_server_ui(product_name, quantity)
        response.message = self.server_response
        response.success = self.server_response == "Order Accepted"
        return response

    def create_server_ui(self, product_name, quantity):
        self.server_app = ServerApp(self, product_name, quantity)
        self.server_app.run()

    def set_response(self, response):
        self.server_response = response

class ServerApp:
    def __init__(self, node, product_name, quantity):
        self.node = node
        self.product_name = product_name
        self.quantity = quantity
        self.root = tk.Tk()
        self.root.title('Server Order Response')
        self.root.geometry("450x300")
        self.root.resizable(False, False)
        self.initUI()

    def initUI(self):
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 12), padding=10)
        style.configure('TLabel', font=('Arial', 14))

        frame = ttk.Frame(self.root, padding="20")
        frame.pack(fill=tk.BOTH, expand=True)

        order_label = ttk.Label(frame, text=f"Order Received: {self.quantity} x {self.product_name}")
        order_label.pack(pady=20)

        accept_button = ttk.Button(frame, text="Accept Order", command=self.accept_order)
        accept_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=10, pady=10)

        reject_button = ttk.Button(frame, text="Reject Order", command=self.reject_order)
        reject_button.pack(side=tk.RIGHT, expand=True, fill=tk.X, padx=10, pady=10)

    def accept_order(self):
        self.node.set_response("Order Accepted")
        self.root.quit()

    def reject_order(self):
        self.node.set_response("Order Rejected")
        self.root.quit()

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    server_node = ServerNode()

    def ros_spin():
        rclpy.spin(server_node)

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)
    spinner_thread.start()

    rclpy.spin(server_node)
    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()