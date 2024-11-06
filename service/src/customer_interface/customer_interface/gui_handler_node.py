# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from order_interfaces.msg import NewOrder, CancelOrder, FoodOrder
# from order_interfaces.srv import OrderService
# from rclpy.callback_groups import ReentrantCallbackGroup
# from builtin_interfaces.msg import Time
# import datetime
# import time

# class RestaurantClient(Node):
#     def __init__(self):
#         super().__init__('restaurant_client')
        
#         callback_group = ReentrantCallbackGroup()
        
#         self.order_client = self.create_client(
#             OrderService,
#             'order_service',
#             callback_group=callback_group)
            
#         self.cancel_publisher = self.create_publisher(
#             CancelOrder,
#             'cancel_order',
#             10)
            
#         while not self.order_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('서비스를 기다리는 중...')
            
#         self.get_logger().info('레스토랑 클라이언트가 준비되었습니다.')
        
#         # 주문 카운터 (order_id 생성용)
#         self._order_counter = 1

#     def send_order(self, table_id, food_orders):
#         """
#         주문을 보내는 메서드
        
#         Args:
#             table_id (int): 테이블 번호
#             food_orders (list): [(food_id, quantity), ...] 형태의 주문 목록
#                 food_id (int): 음식 ID
#                 quantity (int): 수량
#         """
#         # 새 주문 메시지 생성
#         order = NewOrder()
#         order.order_id = self._order_counter  # 순차적인 주문 번호 할당
#         self._order_counter += 1
#         order.table_id = table_id
        
#         # ROS2 Time 메시지 생성
#         now = self.get_clock().now()
#         order.order_time = Time(sec=int(now.seconds_nanoseconds()[0]),
#                               nanosec=int(now.seconds_nanoseconds()[1]))
        
#         # 음식 주문 목록 생성
#         for food_id, quantity in food_orders:
#             food_order = FoodOrder()
#             food_order.food_id = food_id
#             food_order.quantity = quantity
#             order.orders.append(food_order)
            
#         # 서비스 요청 생성
#         request = OrderService.Request()
#         request.order = order
        
#         # 비동기로 서비스 호출
#         future = self.order_client.call_async(request)
#         return future

#     def cancel_order(self, order_id, table_id, reason="고객 요청"):
#         """
#         주문을 취소하는 메서드
        
#         Args:
#             order_id (int): 취소할 주문 번호
#             table_id (int): 테이블 번호
#             reason (str): 취소 사유
#         """
#         cancel_msg = CancelOrder()
#         cancel_msg.order_id = order_id
#         cancel_msg.table_id = table_id
#         cancel_msg.reason = reason  # 취소 사유 추가
        
#         self.cancel_publisher.publish(cancel_msg)
#         self.get_logger().info(
#             f'주문 취소 요청을 보냈습니다: 주문번호 {order_id}, 테이블 {table_id}, 사유: {reason}'
#         )

# def main(args=None):
#     rclpy.init(args=args)
#     client = RestaurantClient()
    
#     try:
#         food_orders = [
#             (1, 2),  # food_id 1: 2개
#             (2, 1)   # food_id 2: 1개
#         ]
#         future = client.send_order(1, food_orders)
        
#         rclpy.spin_until_future_complete(client, future)
        
#         if future.result() is not None:
#             response = future.result()
#             client.get_logger().info(
#                 f'주문 결과: {response.success}, '
#                 f'주문번호: {response.order_id}, '
#                 f'메시지: {response.message}'
#             )
            
#             # 5초 후 주문 취소
#             time.sleep(5)
#             client.cancel_order(
#                 response.order_id, 
#                 1, 
#                 "고객 변심"
#             )
            
#             # 취소 메시지가 전송될 수 있도록 잠시 대기
#             time.sleep(1)
#         else:
#             client.get_logger().error('서비스 호출 실패')
            
#     except KeyboardInterrupt:
#         pass
#     finally:
#         client.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from order_interfaces.msg import NewOrder, CancelOrder, FoodOrder
from order_interfaces.srv import OrderService
from rclpy.callback_groups import ReentrantCallbackGroup
from builtin_interfaces.msg import Time
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QPushButton, QSpinBox, 
                           QTableWidget, QTableWidgetItem, QMessageBox)
from PyQt5.QtCore import QTimer
import sys
import datetime
import time

class RestaurantClientGUI(QMainWindow):
    def __init__(self, client_node):
        super().__init__()
        self.client_node = client_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('주문 시스템')
        self.setGeometry(100, 100, 800, 600)

        # 메인 위젯과 레이아웃
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # 메뉴 선택 영역
        menu_widget = QWidget()
        menu_layout = QHBoxLayout(menu_widget)

        # 메뉴 항목들
        self.menu_items = {
            1: {"name": "피자", "price": 15000},
            2: {"name": "파스타", "price": 12000},
            3: {"name": "샐러드", "price": 8000},
            4: {"name": "음료", "price": 2000}
        }

        # 각 메뉴에 대한 스피너 생성
        self.spinners = {}
        for food_id, item in self.menu_items.items():
            menu_item = QWidget()
            item_layout = QVBoxLayout(menu_item)
            
            # 메뉴 이름과 가격
            item_label = QLabel(f"{item['name']}\n{item['price']}원")
            item_layout.addWidget(item_label)
            
            # 수량 선택 스피너
            spinner = QSpinBox()
            spinner.setRange(0, 10)
            self.spinners[food_id] = spinner
            item_layout.addWidget(spinner)
            
            menu_layout.addWidget(menu_item)

        layout.addWidget(menu_widget)

        # 테이블 번호 선택
        table_widget = QWidget()
        table_layout = QHBoxLayout(table_widget)
        table_label = QLabel('테이블 번호:')
        self.table_spinner = QSpinBox()
        self.table_spinner.setRange(1, 10)
        table_layout.addWidget(table_label)
        table_layout.addWidget(self.table_spinner)
        table_layout.addStretch()
        layout.addWidget(table_widget)

        # 주문 버튼
        order_btn = QPushButton('주문하기')
        order_btn.clicked.connect(self.place_order)
        layout.addWidget(order_btn)

        # 주문 현황 테이블
        self.order_table = QTableWidget()
        self.order_table.setColumnCount(5)
        self.order_table.setHorizontalHeaderLabels(['주문번호', '테이블', '메뉴', '수량', '상태'])
        layout.addWidget(self.order_table)

        # 선택된 주문 취소 버튼
        cancel_btn = QPushButton('선택 주문 취소')
        cancel_btn.clicked.connect(self.cancel_selected_order)
        layout.addWidget(cancel_btn)

    def place_order(self):
        # 주문할 메뉴 수집
        food_orders = []
        for food_id, spinner in self.spinners.items():
            quantity = spinner.value()
            if quantity > 0:
                food_orders.append((food_id, quantity))

        if not food_orders:
            QMessageBox.warning(self, '경고', '최소 한 개 이상의 메뉴를 선택해주세요.')
            return

        # 주문 전송
        table_id = self.table_spinner.value()
        future = self.client_node.send_order(table_id, food_orders)
        
        # 응답 처리를 위한 콜백 설정
        future.add_done_callback(self.handle_order_response)

    def handle_order_response(self, future):
        try:
            response = future.result()
            if response.success:
                # 주문 테이블에 추가
                row = self.order_table.rowCount()
                self.order_table.insertRow(row)
                
                self.order_table.setItem(row, 0, QTableWidgetItem(str(response.order_id)))
                self.order_table.setItem(row, 1, QTableWidgetItem(str(self.table_spinner.value())))
                
                # 주문한 메뉴들을 문자열로 결합
                menu_str = ', '.join([
                    f"{self.menu_items[food_id]['name']} x{quantity}"
                    for food_id, quantity in [
                        (food_id, self.spinners[food_id].value())
                        for food_id in self.spinners
                        if self.spinners[food_id].value() > 0
                    ]
                ])
                
                self.order_table.setItem(row, 2, QTableWidgetItem(menu_str))
                self.order_table.setItem(row, 3, QTableWidgetItem(str(sum(
                    self.spinners[food_id].value() 
                    for food_id in self.spinners
                    if self.spinners[food_id].value() > 0
                ))))
                self.order_table.setItem(row, 4, QTableWidgetItem('처리중'))
                
                # 스피너 초기화
                for spinner in self.spinners.values():
                    spinner.setValue(0)
                    
                QMessageBox.information(self, '알림', f'주문이 완료되었습니다.\n주문번호: {response.order_id}')
            else:
                QMessageBox.warning(self, '경고', f'주문 실패: {response.message}')
        except Exception as e:
            QMessageBox.critical(self, '오류', f'주문 처리 중 오류 발생: {str(e)}')

    def cancel_selected_order(self):
        current_row = self.order_table.currentRow()
        if current_row < 0:
            QMessageBox.warning(self, '경고', '취소할 주문을 선택해주세요.')
            return

        order_id = int(self.order_table.item(current_row, 0).text())
        table_id = int(self.order_table.item(current_row, 1).text())
        
        reply = QMessageBox.question(self, '확인', 
                                   f'주문번호 {order_id}을 취소하시겠습니까?',
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.client_node.cancel_order(order_id, table_id, "고객 취소")
            self.order_table.item(current_row, 4).setText('취소됨')

class RestaurantClient(Node):
    def __init__(self):
        super().__init__('restaurant_client')
        
        callback_group = ReentrantCallbackGroup()
        
        self.order_client = self.create_client(
            OrderService,
            'order_service',
            callback_group=callback_group)
            
        self.cancel_publisher = self.create_publisher(
            CancelOrder,
            'cancel_order',
            10)
            
        while not self.order_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')
            
        self.get_logger().info('레스토랑 클라이언트가 준비되었습니다.')
        
        self._order_counter = 1

    def send_order(self, table_id, food_orders):
        """
        주문을 보내는 메서드
        
        Args:
            table_id (int): 테이블 번호
            food_orders (list): [(food_id, quantity), ...] 형태의 주문 목록
                food_id (int): 음식 ID
                quantity (int): 수량
        """
        order = NewOrder()
        order.order_id = self._order_counter
        self._order_counter += 1
        order.table_id = table_id
        
        now = self.get_clock().now()
        order.order_time = Time(sec=int(now.seconds_nanoseconds()[0]),
                              nanosec=int(now.seconds_nanoseconds()[1]))
        
        for food_id, quantity in food_orders:
            food_order = FoodOrder()
            food_order.food_id = food_id
            food_order.quantity = quantity
            order.orders.append(food_order)
            
        request = OrderService.Request()
        request.order = order
        
        return self.order_client.call_async(request)

    def cancel_order(self, order_id, table_id, reason="고객 요청"):
        """
        주문을 취소하는 메서드
        
        Args:
            order_id (int): 취소할 주문 번호
            table_id (int): 테이블 번호
            reason (str): 취소 사유
        """
        cancel_msg = CancelOrder()
        cancel_msg.order_id = order_id
        cancel_msg.table_id = table_id
        cancel_msg.reason = reason
        
        self.cancel_publisher.publish(cancel_msg)
        self.get_logger().info(
            f'주문 취소 요청을 보냈습니다: 주문번호 {order_id}, 테이블 {table_id}, 사유: {reason}'
        )

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    
    client_node = RestaurantClient()
    gui = RestaurantClientGUI(client_node)
    gui.show()
    
    # ROS2 스핀을 위한 타이머 설정
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(client_node, timeout_sec=0))
    timer.start(100)  # 100ms 간격으로 ROS2 콜백 처리
    
    try:
        sys.exit(app.exec_())
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()