import rclpy
import DR_init
from collections import deque

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0


class DominoScheduler():
    def __init__(self):
        self.deque = deque()

    def get__all_schedule(self):
        return self.deque
        
    def set_schedule(self, pos):
        # 가운데 블록의 위치 가정 x축 좌우 2.5mm
        print(pos)
        pos[3], pos[4], pos[5] = 90, 90, 90
        pos[1] -= 40    # y
        pos[2] -= 10    # z
        offset = 28
        pos[0] -= offset
        self.deque.append(pos.copy())
        pos[0] += offset
        self.deque.append(pos.copy())
        pos[0] += offset
        self.deque.append(pos.copy())
        print(pos)


    def pop_schedule(self):
        print(self.deque)
        return self.deque.popleft()


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
    
    # 도미노 위치 순서 클래스 인스턴스
    DominoSchedule = DominoScheduler()

    JReady = posj([0, 0, 90, 0, 90, 0])  # 시작위치로

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

    
    def pick_and_place(start_pos, end_pos):

        push_block_pos = trans(start_pos, [0, 0, 20, 0 ,0, 0], DR_BASE, DR_BASE)
        print(start_pos)
        push_block_pos[3], push_block_pos[4], push_block_pos[5] = 0, 180, 0
        print("push_block1")
        print(push_block_pos)
        movel(push_block_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        print(push_block_pos)

        # temp = posx(push_block_pos.tolist())
        push_block_pos = trans(push_block_pos.tolist(), [0, 110, 0, 0, 0, 0], DR_BASE, DR_BASE)
        print("push_block2")
        print(push_block_pos)
        movel(push_block_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        push_block_pos = trans(push_block_pos.tolist(), [0, 0, -20, 0, 0, 0], DR_BASE, DR_BASE)
        print("push_block2")
        print(push_block_pos)
        movel(push_block_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        
        push_block_pos = trans(push_block_pos.tolist(), [0, -40, 0, 0 ,0, 0], DR_BASE, DR_BASE)
        print("push_block3")
        print(push_block_pos)
        movel(push_block_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        push_block_pos = trans(push_block_pos.tolist(), [0, 40, 0, 0 ,0, 0], DR_BASE, DR_BASE)
        print("push_block3")
        print(push_block_pos)
        movel(push_block_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        ungrip()
        movel([0, 0, 100, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel(start_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        wait(1)
        movel([0, -100, 100, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        wait(1)
        movel(end_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        ungrip()
        wait(1)
        movel([0, 0, 250, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
                

    def set_start_pos():
        # grip
        # 도미노 중심 위치 추가 필요
        center = [333.763, 201.292, 277.584, 113.677, 178.065, 114.606]
        delta = [0, 0, 30, 0, 0, 0]
        center_hover = trans(center, delta, DR_BASE, DR_BASE)
        movel(center_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                release_force()
                break
        release_compliance_ctrl()


        # 현재 위치 (z pos)
        wait(1)
        current_posx, _ = get_current_posx(0)
        DominoSchedule.set_schedule(current_posx)

    
    def get_end_pos():
        global counter
        schedule = [[507.346, 18.309, 60.952, 70.474, 178.916, 67.069],
            [517.346, 13.309, 60.952, 70.474, 178.916, 67.069],
            [527.346, 8.309, 60.952, 70.474, 178.916, 67.069]]
        counter += 1
        return schedule[counter]
    
    def rotate():
        center = [333.763, 201.292, 277.584, 113.677, 178.065, 114.606]
        delta = [0, 0, 30, 0, 0, 0]
        center_hover = trans(center, delta, DR_BASE, DR_BASE)
        movel(center_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                release_force()
                break
        release_compliance_ctrl()

        ungrip()
        movel([0, 0, -20, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()
        movel([0, 0, 10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        movel([0, 0, 0, 0, 0, 90], vel=VELOCITY, acc=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        movel([0, 0, -10, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)


    
    global counter
    counter = -1
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")

    movej(JReady, vel=VELOCITY, acc=ACC)


    # 도미노 위치

    while rclpy.ok():
        set_start_pos()
        while(DominoSchedule.get__all_schedule()):
            start_pos = DominoSchedule.pop_schedule()
            if start_pos == None:
                break
            pick_and_place(start_pos, get_end_pos())
        rotate()


    # 마지막 위치로 이동
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)

if __name__ == "__main__":
    main()
