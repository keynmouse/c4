#!/usr/bin/env python3
import rclpy
import sys
import sqlite3
import threading
import time
import pygame
from rclpy.node import Node
from order_interfaces.msg import NewOrder, CancelOrder
from order_interfaces.srv import OrderService
from PyQt5.QtWidgets import QPushButton, QLabel, QApplication, QMainWindow, QWidget, QVBoxLayout, QTableWidget, QTableWidgetItem, QMessageBox
from PyQt5.QtCore import QTimer
from datetime import datetime
from collections import deque
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import GoalStatus
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import Point, Quaternion


class RestaurantServer(Node):
    def __init__(self, qt_display, restaurant_DB):
        super().__init__('restaurant_server')
        self.qt_display = qt_display
        self.restaurant_DB = restaurant_DB
        self.timestamp = datetime.fromtimestamp(0)
        
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
        order.order_id = self.restaurant_DB.get_table_order()
        self.get_logger().info(f'Received order: {order.order_id} from table {order.table_id}')
        
        seconds = order.order_time.sec
        self.timestamp = datetime.fromtimestamp(seconds)
        # 주문 처리 로직
        self.orders[order.order_id] = order
        
        # 주문 수락 로직
        response.success = self.qt_display.accept_order(order)
        
        # 응답 설정
        if response.success:
            # PyQt 디스플레이 업데이트
            self.qt_display.update_order_display(order, self.timestamp)
            # 주문 DB 등록
            self.restaurant_DB.create_order(order, self.timestamp)
            message = f'주문이 성공적으로 접수되었습니다: 주문번호 {order.order_id}'
            self.get_logger().info(message)
            response.order_id = order.order_id
            response.message = message
            return response
        else:
            message = f'주문이 취소되었습니다: 주문번호 {order.order_id}'
            self.get_logger().info(message)
            response.order_id = order.order_id
            response.message = message
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
    def __init__(self, gui_node, restaurant_DB):
        super().__init__()
        self.setWindowTitle('주방 디스플레이')
        self.setGeometry(100, 100, 800, 600)
        self.gui_node = gui_node
        self.restaurant_DB = restaurant_DB
        self.sales_volume = 0

        self.menu_name = {
            1: "피자",
            2: "파스타",
            3: "샐러드",
            4: "음료",
        }


        # 메인 위젯과 레이아웃
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout(main_widget)

        # 주문 현황 테이블
        self.order_table = QTableWidget()
        self.order_table.setColumnCount(7)
        self.order_table.setHorizontalHeaderLabels(['주문번호', '테이블', '메뉴', '수량', '주문시간', '서빙', '조리중/서빙완료'])
        layout.addWidget(self.order_table)

        button = QPushButton("매출 확인")
        button.clicked.connect(lambda checked: self.on_sales_button_click())
        layout.addWidget(button)
        self.sales_layout = QLabel(f" 총 매출: {self.sales_volume}")
        layout.addWidget(self.sales_layout)

    def update_order_display(self, order, timestamp):
        """새 주문 추가 또는 기존 주문 업데이트"""
        for food_order in order.orders:
            row_position = self.order_table.rowCount()
            self.order_table.insertRow(row_position)
            self.order_table.setItem(row_position, 0, QTableWidgetItem(str(order.order_id)))
            self.order_table.setItem(row_position, 1, QTableWidgetItem(str(order.table_id)))
            self.order_table.setItem(row_position, 2, QTableWidgetItem(str(self.menu_name[food_order.food_id])))
            self.order_table.setItem(row_position, 3, QTableWidgetItem(str(food_order.quantity)))
            self.order_table.setItem(row_position, 4, QTableWidgetItem(str(timestamp)))
            # 버튼 추가
            button = QPushButton("서빙")
            button.clicked.connect(lambda checked, r=row_position: self.button_clicked(int(self.order_table.item(r, 1).text()), 
                                                                                       r, 
                                                                                       int(order.order_id)))
            self.order_table.setCellWidget(row_position, 5, button)
            self.order_table.setItem(row_position, 6, QTableWidgetItem(str("조리중")))

    # 서빙 버튼 클릭시 실행
    def button_clicked(self, table_id:int, row_position:int, order_id:int):
        self.gui_node.position = self.gui_node.pose[table_id-1]
        self.gui_node.navigate_to_pose_send_goal()
        self.order_table.setItem(row_position, 6, QTableWidgetItem(str("서빙중")))
        self.gui_node.order_table = self.order_table
        self.gui_node.row_position = row_position

        self.restaurant_DB.update_order_status(order_id, "CLOSED")
        
    
    def on_sales_button_click(self):
        self.sales_volume = self.restaurant_DB.get_total_sales_volume()
        self.sales_layout.setText(f" 총 매출: {self.sales_volume}")
    
    
    def remove_order(self, order_id):
        """주문 취소 시 테이블에서 해당 주문 제거"""
        for row in range(self.order_table.rowCount()):
            if self.order_table.item(row, 0).text() == str(order_id):
                self.order_table.removeRow(row)
                break

    def accept_order(self, order):
        file_path = './resource/baedalyi-minjog.mp3'

        try:
            # pygame 초기화
            pygame.mixer.init()
            pygame.mixer.music.load(file_path)
            
            # 소리 재생을 비동기적으로 실행
            thread = threading.Thread(target=pygame.mixer.music.play, daemon=True)
            thread.start()
        except Exception as e:
            print(f"재생 오류: {e}")

        reply = QMessageBox.question(self, '주문 수락', 
                            f'주문번호 {order.order_id}을/를 수락하시겠습니까?',
                            QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            return True
        
        elif reply == QMessageBox.No:
            return False
        

class RestaurantDatabase():
    def __init__(self):
        self.connection = self.create_connection()
        self.create_tables(self.connection)
        self.table_data_initialization(self.connection)
        self.menu_data_initialization(self.connection)

            
        
    def create_connection(self):
        try:
            # DB 연결
            conn = sqlite3.connect('restaurant.db')
            conn.row_factory = sqlite3.Row
            return conn
        except sqlite3.Error as e:
            print(f"Error connecting to database: {e}")
            return None
        
    # 테이블 생성
    def create_tables(self, conn):
        try:
            cursor = conn.cursor()
            cursor.executescript('''
            CREATE TABLE IF NOT EXISTS "TABLE" (
                table_number INTEGER PRIMARY KEY,
                capacity INTEGER,
                is_occupied INTEGER
            );
                           
            CREATE TABLE IF NOT EXISTS "ORDER" (
                order_id INTEGER PRIMARY KEY,
                table_number INTEGER,
                order_time TEXT,
                status TEXT,
                FOREIGN KEY (table_number) REFERENCES "TABLE" (table_number)
            );
                           
            CREATE TABLE IF NOT EXISTS "MENU_ITEM" (
                menu_item_id INTEGER PRIMARY KEY,
                name TEXT,
                description TEXT,
                price INTEGER,
                is_available INTEGER
            );

            CREATE TABLE IF NOT EXISTS "ORDER_ITEM" (
                order_item_id INTEGER PRIMARY KEY,
                order_id INTEGER,
                menu_item_id INTEGER,
                quantity INTEGER,
                FOREIGN KEY (order_id) REFERENCES "ORDER" (order_id),
                FOREIGN KEY (menu_item_id) REFERENCES "MENU_ITEM" (menu_item_id)
            );


            CREATE INDEX IF NOT EXISTS idx_order_table ON "ORDER" (table_number);
            CREATE INDEX IF NOT EXISTS idx_orderitem_menuitem ON "ORDER_ITEM" (menu_item_id);         
            CREATE INDEX IF NOT EXISTS idx_orderitem_order ON "ORDER_ITEM" (order_id);
            ''')

            conn.commit()
        except sqlite3.Error as e:
            print(f"Error creating tables: {e}")
            conn.rollback()

    def table_data_initialization(self, conn):
        try:
            cursor = self.connection.cursor()
            cursor.execute(f'SELECT COUNT(*) FROM "TABLE"')
            if int(cursor.fetchone()[0]) == 9:
                return None
        except sqlite3.Error as e:
                print(f"데이터 초기화 실패: {e}")
                self.connection.rollback()
                return None
    
        try:
            cursor = conn.cursor()
            for _ in range(9):
                cursor.execute('''
                INSERT INTO "TABLE" (capacity, is_occupied)
                VALUES (?, ?)
                ''', (4, 1))
                conn.commit()
        except sqlite3.Error as e:
            print(f"Error initialization table data: {e}")
            conn.rollback()


    def menu_data_initialization(self, conn):
        menu_items = {
            1: {"name": "피자", 
                "price": 15000, 
                "description": "이탈리안 정통 방식으로 화덕에서 구운 나폴리 스타일 피자"},
                
            2: {"name": "파스타", 
                "price": 12000, 
                "description": "알덴테로 삶은 면과 특제 소스로 맛을 낸 수제 파스타"},
                
            3: {"name": "샐러드", 
                "price": 8000, 
                "description": "제철 채소와 호두, 크랜베리를 곁들인 건강한 샐러드"},
                
            4: {"name": "음료", 
                "price": 2000, 
                "description": "매장에서 직접 만드는 수제 에이드와 다양한 음료 선택 가능"}
        }

        try:
            cursor = self.connection.cursor()
            cursor.execute(f'SELECT COUNT(*) FROM "ORDER"')
            if int(cursor.fetchone()[0]) == 4:
                return None
        except sqlite3.Error as e:
                print(f"데이터 초기화 실패: {e}")
                self.connection.rollback()
                return None
        
    
        try:
            cursor = conn.cursor()
            for food_id, item in menu_items.items():
                cursor.execute('''
                INSERT INTO "MENU_ITEM" (menu_item_id, name, description, price, is_available)
                VALUES (?, ?, ?, ?, ?)
                ''', (food_id, item["name"], item["description"], item["price"], 1))
                conn.commit()
        except sqlite3.Error as e:
            print(f"Error initialization menu data: {e}")
            conn.rollback()

    # 수락한 주문 생성
    def create_order(self, request, timestamp):
            try:
                cursor = self.connection.cursor()
                cursor.execute('''
                INSERT INTO "ORDER" (order_id, table_number, order_time, status)
                VALUES (?, ?, ?, ?)
                ''', (request.order_id, request.table_id, timestamp.strftime('%Y-%m-%d %H:%M:%S'), "NEW"))
                self.connection.commit()

                for order in request.orders:
                    cursor.execute('''
                    INSERT INTO "ORDER_ITEM" (order_id, menu_item_id, quantity)
                    VALUES (?, ?, ?)
                    ''', (request.order_id, order.food_id, order.quantity))
                    self.connection.commit()
                return cursor.lastrowid     # 가장 최근에 INSERT한 행의 ID(row id)를 반환
            except sqlite3.Error as e:
                print(f"등록 실패: {e}")
                self.connection.rollback()
                return None
            

    def get_table_order(self):
        try:
            cursor = self.connection.cursor()
            cursor.execute(f'SELECT COUNT(*) FROM "ORDER"')
            return cursor.fetchone()[0] + 1
        except sqlite3.Error as e:
            print(f"테이블 번호 불러오기 실패: {e}")
            self.connection.rollback()
            return None
        

    def update_order_status(self, order_id, status):
        try:
            cursor = self.connection.cursor()
            cursor.execute('''
            UPDATE "ORDER"
            SET status=?
            WHERE order_id=?
            ''', (status, order_id))
            return None
        except sqlite3.Error as e:
            print(f"주문 상태 바꾸기 실패: {e}")
            self.connection.rollback()
            return None


    def sync_with_DB(self):
        pass

    def get_today_sales_volume(self):

        pass

    def get_sales_volume(self, order_id):
        try:
            cursor = self.connection.cursor()
            cursor.execute(f'''
            SELECT SUM(oi.quantity * mi.price) as total_amount
            FROM ORDER_ITEM oi
            JOIN MENU_ITEM mi ON oi.menu_item_id = mi.menu_item_id
            WHERE oi.order_id = {order_id};
            ''')
            return cursor.fetchone()[0]
        except sqlite3.Error as e:
            print(f"주문번호 {order_id}의 총액 구하기 실패: {e}")
            self.connection.rollback()
            return None

    def get_total_sales_volume(self):
        try:
            cursor = self.connection.cursor()
            cursor.execute(f'''
            SELECT SUM(oi.quantity * mi.price) as total_amount
            FROM ORDER_ITEM oi
            JOIN MENU_ITEM mi ON oi.menu_item_id = mi.menu_item_id;
            ''')
            return cursor.fetchone()[0]
        except sqlite3.Error as e:
            print(f"총 매출 구하기 실패: {e}")
            self.connection.rollback()
            return None



class GuiNode(Node):
    """ROS2 노드 클래스"""
    def __init__(self):
        super().__init__("gui_node")

        # Action Client 생성
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Service Cline 생성
        self.set_yaw_goal_tolerance_client = self.create_client(SetParameters, "/controller_server/set_parameters")

        # Init
        self.set_yaw_goal_tolerance("general_goal_checker.yaw_goal_tolerance", 90.0)
        self.position = [0.0, 0.0]
        
        # create Service Clinet
        self.init_pose = [0.0, 0.0, 0.0, 1.0] # pose:x,y orient:z,w
        self.set_initial_pose_service_client = self.create_client(SetInitialPose, '/set_initial_pose')
        
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')

        self.set_initial_pose(*self.init_pose)
        self.pose = [(2.6, 1.6),    # 0
                    (2.6, 0.5),     # 1
                    (2.6, -0.6),    # 2
                    (1.5, 1.6),     # 3
                    (1.5, 0.5),     # 4
                    (1.5, -0.6),    # 5
                    (0.4, 1.6),     # 6
                    (0.4, 0.5),     # 7
                    (0.4, -0.6)]    # 8
        
        self.order_table = None
        self.row_position = None


    # Service client SET INIT POSE ESTIMATE
    def set_initial_pose(self, x,y,z,w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        future = self.set_initial_pose_service_client.call_async(req)
        
        return future.result()
        
    ## ACTION CLIENT NAVIGATE      
    def navigate_to_pose_send_goal(self):
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                print(message)
                return False
            wait_count += 1


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.position[0]
        goal_msg.pose.pose.position.y = self.position[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)

        self.send_goal_future.add_done_callback(self.navigate_to_pose_action_goal)
        return True
    
    def navigate_to_pose_go_init(self):
        time.sleep(4)
        wait_count = 1
        while not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=0.1):
            if wait_count > 3:
                message = "[WARN] Navigate action server is not available."
                print(message)
                return False
            wait_count += 1
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.init_pose[0]
        goal_msg.pose.pose.position.y = self.init_pose[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.send_goal_future = self.navigate_to_pose_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigate_to_pose_action_feedback)
        self.action_result_future.add_done_callback(lambda _ : self.update_state())
        return True
    
    # 서빙 버튼 정보 업데이트
    def update_state(self):
        self.order_table.setItem(self.row_position, 6, QTableWidgetItem(str("서빙완료")))


    def navigate_to_pose_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            message = "[WARN] Action goal rejected."
            print(message)
            return

        message = "[INFO] Action goal accepted."
        print(message)

        self.action_result_future = goal_handle.get_result_async()
        self.action_result_future.add_done_callback(self.navigate_to_pose_action_result)
        self.action_result_future.add_done_callback(lambda _ : self.navigate_to_pose_go_init())



    def navigate_to_pose_action_feedback(self, feedback_msg):
        action_feedback = feedback_msg.feedback
        # self.get_logger().info("Action feedback: {0}".format(action_feedback))

    def navigate_to_pose_action_result(self, future):
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            message = "[INFO] Action succeeded!."
            print(message)

        else:
            message = f"[WARN] Action failed with status: {action_status}"
            print(message)


    # SERVICE CLIENT SET PARAMETER
    def set_yaw_goal_tolerance(self, parameter_name, value):
        request = SetParameters.Request()
        parameter = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        request.parameters = [Parameter(name=parameter_name, value=parameter)]
        service_client = self.set_yaw_goal_tolerance_client
        return self.call_service(service_client, request, "yaw_goal_tolerance parameter")


    def call_service(self, service_client, request, service_name):
        wait_count=1
        while not service_client.wait_for_service(timeout_sec=0.1):
            if wait_count > 10:
                message = f"[WARN] {service_name} service is not available"
                print(message)
                return False
            wait_count += 1

        message = f"[INIT] Set to have no yaw goal"
        print(message)
        service_client.call_async(request)

        return True


def main(args=None):
    rclpy.init(args=args)

    # DB
    restaurant_DB = RestaurantDatabase()

    # Robot Control 
    gui_node = GuiNode()
    
    # pyqt5
    app = QApplication(sys.argv)
    qt_display = RestaurantDisplay(gui_node, restaurant_DB)
    qt_display.show()

   
    
    
    restaurant_server = RestaurantServer(qt_display, restaurant_DB)

    
    # 노드 리스트 생성
    nodes = [restaurant_server, gui_node] # 노드 객체들

    # 단일 타이머로 모든 노드 처리
    timer = QTimer()
    timer.timeout.connect(lambda: [rclpy.spin_once(node, timeout_sec=0) for node in nodes])
    timer.start(100)

    
    try:
        sys.exit(app.exec_())
    finally:
        restaurant_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()