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
            move_periodic,
            check_position_condition,
            DR_TOOL,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS
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


    def pick_and_place(start_pos, end_pos):
        # grip
        delta = [0, 0, 30, 0, 0, 0]
        start_hover = trans(start_pos, delta, DR_BASE, DR_BASE)
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        # 힘 제어 내려가기 
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                release_force()
                break
        release_compliance_ctrl()

        # 올라가기
        movel([0, 0, 30, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        ungrip()
        wait(1)
        # 내려가기

        movel([0, 0, -40, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()
        wait(1)

        # 올라가기
        movel([0, 0, 80, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        delta = [0, 0, 80, 0, 0, 0]
        end_hover = trans(end_pos, delta, DR_BASE, DR_BASE)
        movel(end_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                release_force()
                ungrip()
                wait(1)
                break
        release_compliance_ctrl()
        movel([0, 0, 30, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)


    def pick_and_place_with_periodic(start_pos, end_pos):
        # grip
        delta = [0, 0, 30, 0, 0, 0]
        start_hover = trans(start_pos, delta, DR_BASE, DR_BASE)
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

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

        # 올라가기
        movel([0, 0, 30, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        ungrip()
        wait(1)
        # 내려가기

        movel([0, 0, -40, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        grip()
        wait(1)

        # 올라가기
        movel([0, 0, 80, 0, 0, 0],vel=VELOCITY, acc=ACC, ref=DR_BASE, mod=DR_MV_MOD_REL)
        delta = [0, 0, 80, 0, 0, 0]
        end_hover = trans(end_pos, delta, DR_BASE, DR_BASE)
        movel(end_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)


        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir= [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL) 
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                move_periodic(amp =[0,0,0,0,0,20], period=1.0, atime=0.2, repeat=2, ref=DR_TOOL)
                wait(3)
                if check_position_condition(DR_AXIS_Z, max=10, mod=DR_MV_MOD_REL, ref=DR_BASE, pos=end_pos):
                    print("check_position_condition_work")
                    continue
                print("release")
                release_force()
                wait(1)
                ungrip()
                wait(1)
                break

        release_compliance_ctrl()
    
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")

    JReady = posj([0, 0, 90, 0, 90, 0])  # 시작위치로
    movej(JReady, vel=VELOCITY, acc=ACC)

    
    # 기어 위치
    # 시작위치 1,2,3
    # posx([363.185, -159.041, 37.909, 155.464, 179.999, 154.806])
    # posx([456.404, -207.424, 38.278, 145.606, 179.996, 144.762])
    # posx([451.203, -102.970, 37.690, 162.614, -179.999, 161.599])
    # 도착위치 1,2,3
    # posx([367.774, 140.455, 37.509, 18.361, -179.998, 17.623])
    # posx([460.633, 92.011, 37.671, 4.761, 179.999, 4.198])
    # posx([456.302, 196.711, 38.190, 31.706, 179.995, 31.464])
    # 작은 기어 시작 도착위치
    # posx([423.194, -153.798, 43.123, 16.526, -179.447, 16.253])
    # posx([428.155, 146.129, 41.963, 13.999, 180.000, 13.695])


    gear_start_pos = [posx([363.185, -159.041, 37.909, 155.464, 179.999, 154.806]),
                        posx([456.404, -207.424, 38.278, 145.606, 179.996, 144.762]),
                        posx([451.203, -102.970, 37.690, 162.614, -179.999, 161.599])]
    gear_end_pos = [posx([367.774, 140.455, 37.509, 18.361, -179.998, 17.623]),
                    posx([460.633, 92.011, 37.671, 4.761, 179.999, 4.198]),
                    posx([456.302, 196.711, 38.190, 31.706, 179.995, 31.464])]
    
    small_start = posx([423.194, -153.798, 43.123, 16.526, -179.447, 16.253])
    small_end = posx([428.155, 146.129, 41.963, 13.999, 180.000, 13.695])

    while rclpy.ok():
        # for start_pos, end_pos in zip(gear_start_pos, gear_end_pos):
        #     pick_and_place(start_pos, end_pos)
        pick_and_place_with_periodic(small_start, small_end)
        


    # 마지막 위치로 이동
    movej(JReady, vel=VELOCITY, acc=ACC)
if __name__ == "__main__":
    main()
