#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from calculator_interfaces.srv import Calculate
import tkinter as tk
from tkinter import ttk, messagebox

class CalculatorClient(Node):
    def __init__(self):
        super().__init__('calculator_client')
        self.client = self.create_client(Calculate, 'calculator')
        self.create_gui()

    def create_gui(self):
        self.window = tk.Tk()
        self.window.title("ROS2 Calculator")
        self.window.geometry("300x400")

        # Main frame
        main_frame = ttk.Frame(self.window, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))

        # Input fields
        ttk.Label(main_frame, text="First Number:").grid(row=0, column=0, pady=5)
        self.num1 = ttk.Entry(main_frame)
        self.num1.grid(row=0, column=1, pady=5)

        ttk.Label(main_frame, text="Second Number:").grid(row=1, column=0, pady=5)
        self.num2 = ttk.Entry(main_frame)
        self.num2.grid(row=1, column=1, pady=5)

        # Operation selection
        ttk.Label(main_frame, text="Operation:").grid(row=2, column=0, pady=5)
        self.operation = ttk.Combobox(main_frame, 
                                    values=['add', 'subtract', 'multiply', 'divide'],
                                    state='readonly')
        self.operation.set('add')
        self.operation.grid(row=2, column=1, pady=5)

        # Calculate button
        ttk.Button(main_frame, text="Calculate", 
                  command=self.send_request).grid(row=3, column=0, columnspan=2, pady=20)

        # Result display
        self.result_var = tk.StringVar(value="Result will appear here")
        result_label = ttk.Label(main_frame, textvariable=self.result_var,
                               wraplength=250, justify="center")
        result_label.grid(row=4, column=0, columnspan=2, pady=10)

        # History display
        self.history = tk.Text(main_frame, height=8, width=30)
        self.history.grid(row=5, column=0, columnspan=2, pady=10)
        
        # Clear history button
        ttk.Button(main_frame, text="Clear History", 
                  command=self.clear_history).grid(row=6, column=0, columnspan=2, pady=5)

    def clear_history(self):
        self.history.delete(1.0, tk.END)

    def send_request(self):
        try:
            num1 = float(self.num1.get())
            num2 = float(self.num2.get())
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
            return

        if not self.client.wait_for_service(timeout_sec=1.0):
            messagebox.showerror("Error", "Service not available")
            return

        request = Calculate.Request()
        request.a = num1
        request.b = num2
        request.operation = self.operation.get()

        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                result_text = f"Result: {response.result}"
                self.result_var.set(result_text)
                # Add to history
                self.history.insert(tk.END, f"{response.message}\n")
                self.history.see(tk.END)
            else:
                messagebox.showerror("Error", response.message)
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            self.window.update()

def main():
    rclpy.init()
    client = CalculatorClient()
    
    try:
        client.run()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()