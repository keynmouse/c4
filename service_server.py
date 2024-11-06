import rclpy
from rclpy.node import Node
from calculator_interfaces.srv import Calculate

class CalculatorServer(Node):
    def __init__(self):
        super().__init__('calculator_server')
        self.srv = self.create_service(
            Calculate,
            'calculator',
            self.calculate_callback
        )
        self.get_logger().info('Calculator Server is ready')

    def calculate_callback(self, request, response):
        a, b = request.a, request.b
        op = request.operation.lower()

        try:
            if op == 'add':
                response.result = a + b
            elif op == 'subtract':
                response.result = a - b
            elif op == 'multiply':
                response.result = a * b
            elif op == 'divide':
                if b == 0:
                    raise ZeroDivisionError("Division by zero is not allowed")
                response.result = a / b
            else:
                raise ValueError(f"Unknown operation: {op}")

            response.success = True
            response.message = f"Successfully calculated {a} {op} {b} = {response.result}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)

        return response

def main():
    rclpy.init()
    calculator_server = CalculatorServer()
    
    try:
        rclpy.spin(calculator_server)
    except KeyboardInterrupt:
        calculator_server.get_logger().info('Server stopped cleanly')
    except Exception as e:
        calculator_server.get_logger().error(f'Server stopped unexpectedly: {e}')
    finally:
        calculator_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
