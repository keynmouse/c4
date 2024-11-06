import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk
from test3.srv import Order

class ClientNode(Node):
    def __init__(self):
        super().__init__('client_node')
        self.client = self.create_client(Order, 'process_order')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def send_order(self, product_name, quantity):
        request = Order.Request()
        request.product_name = product_name
        request.quantity = quantity
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            messagebox.showinfo("Order Response", response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

class ClientApp:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title('Client Order System')
        self.root.geometry("450x350")
        self.root.resizable(False, False)
        self.initUI()

    def initUI(self):
        style = ttk.Style()
        style.configure('TButton', font=('Arial', 12), padding=10)
        style.configure('TLabel', font=('Arial', 14))

        frame = ttk.Frame(self.root, padding="20")
        frame.pack(fill=tk.BOTH, expand=True)

        menu_label = ttk.Label(frame, text="Select an item to order")
        menu_label.pack(pady=20)

        self.menu_items = ["Pizza", "Burger", "Pasta"]
        for item in self.menu_items:
            button = ttk.Button(frame, text=item, command=lambda i=item: self.select_menu_item(i))
            button.pack(fill=tk.X, pady=5)

    def select_menu_item(self, product_name):
        quantity = simpledialog.askinteger('Select Quantity', f'How many {product_name}s would you like?', minvalue=1, maxvalue=100)
        if quantity:
            if messagebox.askyesno('Confirm Order', f"You have selected {quantity} x {product_name}.\nDo you want to proceed?"):
                self.node.send_order(product_name, quantity)

def main(args=None):
    rclpy.init(args=args)
    client_node = ClientNode()

    root = tk.Tk()
    client_app = ClientApp(root, client_node)

    def ros_spin():
        rclpy.spin(client_node)

    import threading
    spinner_thread = threading.Thread(target=ros_spin, daemon=True)
    spinner_thread.start()

    root.mainloop()
    spinner_thread.join()
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()