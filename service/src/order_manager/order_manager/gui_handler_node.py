#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from order_interfaces.msg import NewOrder, CancelOrder
from order_interfaces.srv import OrderService
from PyQt5.QtWidgets import QPushButton, QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QTableWidget, QTableWidgetItem, QMessageBox
from PyQt5.QtCore import QTimer
import sys
import sqlite3
from datetime import datetime
from playsound import playsound
import threading


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
        
        seconds = order.order_time.sec + (order.order_time.nanosec / 1e9)
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
        self.order_table.setColumnCount(7)
        self.order_table.setHorizontalHeaderLabels(['주문번호', '테이블', '메뉴', '수량', '주문시간', '서빙', '조리중/서빙완료'])
        layout.addWidget(self.order_table)

    def update_order_display(self, order, timestamp):
        """새 주문 추가 또는 기존 주문 업데이트"""
        for food_order in order.orders:
            row_position = self.order_table.rowCount()
            self.order_table.insertRow(row_position)
            self.order_table.setItem(row_position, 0, QTableWidgetItem(str(order.order_id)))
            self.order_table.setItem(row_position, 1, QTableWidgetItem(str(order.table_id)))
            self.order_table.setItem(row_position, 2, QTableWidgetItem(str(food_order.food_id)))
            self.order_table.setItem(row_position, 3, QTableWidgetItem(str(food_order.quantity)))
            self.order_table.setItem(row_position, 4, QTableWidgetItem(str(timestamp)))
            # 버튼 추가
            button = QPushButton("서빙")
            button.clicked.connect(lambda checked, r=row_position: self.button_clicked(int(self.order_table.item(r, 1).text())))
            self.order_table.setCellWidget(row_position, 5, button)
            self.order_table.setItem(row_position, 6, QTableWidgetItem(str("조리중")))

    # 서빙 버튼 클릭시 실행 메소드
    def button_clicked(self, table_id:int):
        print(table_id)
        pass

    def remove_order(self, order_id):
        """주문 취소 시 테이블에서 해당 주문 제거"""
        for row in range(self.order_table.rowCount()):
            if self.order_table.item(row, 0).text() == str(order_id):
                self.order_table.removeRow(row)
                break

    def accept_order(self, order):
        file_path = './resource/baedalyi-minjog.mp3'

        try:
            thread = threading.Thread(
                target=playsound, 
                args=(file_path), 
                daemon=True
            )
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
        

    def update_order_status(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    
    # pyqt5
    app = QApplication(sys.argv)
    qt_display = RestaurantDisplay()
    qt_display.show()

    # DB
    restaurant_DB = RestaurantDatabase()
    
    restaurant_server = RestaurantServer(qt_display, restaurant_DB)

    
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