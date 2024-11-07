import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class KitchenDisplay(Node):
    def __init__(self):
        super().__init__('kitchen_display')
        self.subscription = self.create_subscription(
            String,
            'order_received',
            self.display_order,
            10
        )
        self.alarm_publisher = self.create_publisher(Bool, 'alarm', 10)
        self.get_logger().info('Kitchen Display Node has been started.')

    def display_order(self, msg):
        order = msg.data
        self.get_logger().info(f'New order received: {order}')
        # UI 업데이트 로직 추가 가능
        self.trigger_alarm(True)

    def trigger_alarm(self, state):
        alarm_msg = Bool()
        alarm_msg.data = state
        self.alarm_publisher.publish(alarm_msg)
        self.get_logger().info(f'Alarm state: {state}')

def main(args=None):
    rclpy.init(args=args)
    node = KitchenDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
