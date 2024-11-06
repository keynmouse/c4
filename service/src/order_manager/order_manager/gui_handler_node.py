#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from order_interfaces.msg import NewOrder, CancelOrder
from order_interfaces.srv import OrderService
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem
from PyQt5.QtCore import QTimer
import sys

class RestaurantServer(Node):
    def __init__(self, qt_display):
        super().__init__('restaurant_server')
        self.qt_display = qt_display
        
        # 서비스 서버 생성
        self.order_service = self.create_service(
            OrderService,
            'order_service',
            self.handle_order)
            
        # 취소 주문 구독자 생성
        self.cancel_subscription = self.create_subscription(
            CancelOrder,
            'cancel_order',
            self.handle_cancel_order,
            10)
            
        self.orders = {}  # 주문 저장용 딕셔너리
        self.get_logger().info('Restaurant server is ready to take orders.')

    def handle_order(self, request, response):
        """주문 서비스 처리"""
        order = request.order
        self.get_logger().info(f'Received order: {order.order_id} from table {order.table_id}')
        
        # 주문 처리 로직
        self.orders[order.order_id] = order
        
        # PyQt 디스플레이 업데이트
        self.qt_display.update_order_display(order)
        
        # 응답 설정
        response.success = True
        response.order_id = order.order_id
        response.message = f'주문이 성공적으로 접수되었습니다: 주문번호 {order.order_id}'
        return response

    def handle_cancel_order(self, msg):
        """주문 취소 토픽 처리"""
        order_id = msg.order_id
        if order_id in self.orders:
            self.get_logger().info(f'Cancelling order {order_id} from table {msg.table_id}')
            # 주문 취소 처리
            del self.orders[order_id]
            # PyQt 디스플레이 업데이트
            self.qt_display.remove_order(order_id)
        else:
            self.get_logger().warn(f'Order {order_id} not found for cancellation')

class RestaurantDisplay(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('주방 디스플레이')
        self.setGeometry(100, 100, 800, 600)

        # 메인 위젯과 레이아웃
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # 주문 현황 테이블
        self.order_table = QTableWidget()
        self.order_table.setColumnCount(5)
        self.order_table.setHorizontalHeaderLabels(['주문번호', '테이블', '메뉴', '수량', '주문시간'])
        layout.addWidget(self.order_table)

    def update_order_display(self, order):
        """새 주문 추가 또는 기존 주문 업데이트"""
        for food_order in order.orders:
            row_position = self.order_table.rowCount()
            self.order_table.insertRow(row_position)
            
            self.order_table.setItem(row_position, 0, QTableWidgetItem(str(order.order_id)))
            self.order_table.setItem(row_position, 1, QTableWidgetItem(str(order.table_id)))
            self.order_table.setItem(row_position, 2, QTableWidgetItem(str(food_order.food_id)))
            self.order_table.setItem(row_position, 3, QTableWidgetItem(str(food_order.quantity)))
            self.order_table.setItem(row_position, 4, QTableWidgetItem(str(order.order_time)))

    def remove_order(self, order_id):
        """주문 취소 시 테이블에서 해당 주문 제거"""
        for row in range(self.order_table.rowCount()):
            if self.order_table.item(row, 0).text() == str(order_id):
                self.order_table.removeRow(row)
                break

def main(args=None):
    rclpy.init(args=args)
    
    app = QApplication(sys.argv)
    qt_display = RestaurantDisplay()
    qt_display.show()
    
    restaurant_server = RestaurantServer(qt_display)
    
    # Timer for processing ROS2 callbacks
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(restaurant_server, timeout_sec=0))
    timer.start(100)  # 100ms 간격으로 ROS2 콜백 처리
    
    try:
        sys.exit(app.exec_())
    finally:
        restaurant_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()