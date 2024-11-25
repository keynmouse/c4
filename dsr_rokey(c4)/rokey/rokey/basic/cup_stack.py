import rclpy
import DR_init
from collections import deque
import math

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 160, 160

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0



def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_control", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            set_tool,
            set_tcp,
            set_digital_output,
            get_digital_input,
            movej,
            movel,
            wait,
            trans,
            get_current_posx,
            parallel_axis,
            DR_TOOL,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    # grip motion
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            print("Wait for digital input")
    
    def grip():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait_digital_input(1)

    def ungrip():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait_digital_input(2)

    def pick_and_place3(pick_pos, place_pos,count):
        # Move to hover position above pick location
        delta = [0, 0, 30, 0, 0, 0]
        start_hover = trans(pick_pos, delta, DR_BASE, DR_BASE)
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # Start gripping
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        
        movel([0,0,-13*count,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)

        # Force control to lower
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=6) == 0:
                release_force()
                break
            wait(0.1)  # 추가적인 대기 시간

        release_compliance_ctrl()

        # before go to pick up place (status = ungrip)
        movel([0, 0, 30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        ungrip()
        wait(1)

        # go to pick up place (status = grip)
        movel([0, 0, -45, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()
        wait(1)

        # Move up after placing
        movel([0, 0, 120, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        delta = [0, 0, 120, 0, 0, 0]
        hover_pos = trans(place_pos, [0, 0, 350-place_pos[2], 0, 0, 0], DR_BASE, DR_BASE)
        movel(hover_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # place_pos로 수직 하강 (70mm 띄움)
        place_pos_hover = trans(place_pos, [0, 0, 70, 0, 0, 0], DR_BASE, DR_BASE)
        movel(place_pos_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 힘제어로 내려가기
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        
        while True:
            if check_force_condition(DR_AXIS_Z, min=4) == 0:
                release_force()
                wait(1)
                ungrip()
                wait(1)
                break
            wait(0.1)
            
        release_compliance_ctrl()
    
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")

    JReady = posj([0, 0, 90, 0, 90, 0])  # 시작위치로
    movej(JReady, vel=VELOCITY, acc=ACC)

    class Test():
        def __init__(self):
            self.deque = deque()
            self.pos = []
            self.cent:posx = None
            self.calculate_circle_centers(0, 0, 95*2) # 3층
            
            

        def set_pose(self, pose):
            self.cent = pose.copy()
            for po in self.pos:
                temp = self.cent.copy()
                temp[0] += po[0]
                temp[1] += po[1]
                temp[2] += po[2]
                self.deque.append(temp)
             

        def get_end_pos(self):
            return self.deque.pop()

        def get__all_schedule(self):
            return self.deque

        def append_pos(self, pos):
            self.deque.append(pos.copy())
        
        def calculate_circle_centers(self, a, b, height):
            r = 45
            height -= 95
            if height < 0:
                return

            # 첫 번째 위치
            x1 = a + r
            y1 = b
            if [x1, y1, height] not in self.pos:
                self.pos.append([x1, y1, height])
            
            # 두 번째 위치
            x2 = a - r/2
            y2 = b + r * math.sqrt(3)/2
            if [x2, y2, height] not in self.pos:
                self.pos.append([x2, y2, height]) 
            
            # 세 번째 위치
            x3 = a - r/2
            y3 = b - r * math.sqrt(3)/2
            if [x3, y3, height] not in self.pos:
                self.pos.append([x3, y3, height])

            self.calculate_circle_centers(x1, y1, height)
            self.calculate_circle_centers(x2, y2, height)
            self.calculate_circle_centers(x3, y3, height)
    

    def turn_cup_1(posx1):
        movel(posx1,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        parallel_axis([0,1,0],DR_AXIS_Z,DR_BASE)
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        wait(1)

        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)

        while True:
            if check_force_condition(DR_AXIS_Z, min=6) == 0:
                release_force()
                break
        release_compliance_ctrl()
        
        movel([0,0,10,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(1)
        
        movel([0,8,-30,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        wait(1)
        
        movel([0,0,150,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        movel([0,0,0,0,0,180],vel=VELOCITY,acc=ACC,ref=DR_TOOL,mod=DR_MV_MOD_REL)
        movel([130,0,0,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        movel([0,0,-240,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=5) == 0:
                release_force()
                break
        release_compliance_ctrl()
        movel([0,0,4,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        posx2, _ =get_current_posx(0)
        print(posx2)
        wait(1)

        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(1)
        
        movel([0,-70,0,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        movej([0,0,90,0,90,0],vel=VELOCITY,acc=ACC)
        return posx2
    
    

    def turn_cup_2(posx2):
        delta1=[0,0,130,0,0,0]
        list1 = posx2[0:3]+[0,180,0]
        print(list1)
        movel([0,0,50,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        
        posx3=trans(list1,delta1,DR_BASE,DR_BASE)
        print(posx3)
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        print(movel(posx3,vel=VELOCITY,acc=ACC,ref=DR_BASE))
        movel([0,0,-100,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=5) == 0:
                release_force()
                break
        release_compliance_ctrl()
        movel([0,-3,1,0,0,0],vel=80,acc=80,ref=DR_BASE,mod=DR_MV_MOD_REL)
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(1)
        movel([406.72, -33.94, 350, 173.27, 180, 173.83],vel=80,acc=80,ref=DR_BASE)
        movel([0,0,-30,0,0,0],vel=80,acc=80,ref=DR_BASE,mod=DR_MV_MOD_REL)
        
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=5) == 0:
                release_force()
                break
        release_compliance_ctrl()
        movel([0,0,4,0,0,0],vel=80,acc=80,ref=DR_BASE,mod=DR_MV_MOD_REL)
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        movel([0,0,105,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        movel([0,100,0,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
        
        wait(1)
        
    test_move = Test()

    # pos = posx([548.296, -134.828, 83.853, 179.033, 178.610, 179.279])
    place_pos = posx([406.72, -33.94, 29.63, 173.27, 180, 173.83])  # 컵 플레이스 위치
    pos_top = place_pos.copy()
    pos_top[2] += 95*2 # 3층
    test_move.append_pos(pos_top)
    test_move.set_pose(place_pos)
    pick_pos =posx([424.75, 280.38, 204.14, 0, 180.00, 0]) 
    #[424.75, 280.38, 204.14, 0, 180.00, 0]
    posx1=[424.75, 280.38, 264.14, 0, 180.00, 0]
    #[429.80,258.87,264.23,28.26,179.99,29.96]
    while rclpy.ok():
        a=turn_cup_1(posx1)
        count=0
        while(test_move.get__all_schedule()):
            end_pos = test_move.get_end_pos()
            print(end_pos)
            # movel(end_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
            pick_and_place3(pick_pos, end_pos,count)
            count+=1
            if end_pos == None:
                break
        turn_cup_2(a)
        break

if __name__ == "__main__":
    main()

