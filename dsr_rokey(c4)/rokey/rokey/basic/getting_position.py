import rclpy
import DR_init

# Single robot configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 50, 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("cup", namespace=ROBOT_ID)

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
            movec,
            movesx,
            get_current_posx,
            move_periodic,
            check_position_condition,
            DR_TOOL,
            DR_BASE,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_MV_MOD_REL,
            DR_MV_MOD_ABS,
            parallel_axis  # 추가된 부분
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        rclpy.shutdown()
        return

    # Grip motion functions
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
            wait(0.5)
            node.get_logger().info("Wait for digital input")

    def grip():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait_digital_input(1)

    def ungrip():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait_digital_input(2)

    # Define positions
    place_pos = posx([406.72, -33.94, 299.63, 173.27, 180, 173.83])  # 컵 플레이스 위치
    pick_pos =posx([424.75, 280.38, 204.14, 0, 180.00, 0])  # 컵 피킹 위치
    # Pick and place function
    def pick_and_place(pick_pos, place_pos):
        # Move to hover position above pick location
        delta = [0, 0, 30, 0, 0, 0]
        start_hover = trans(pick_pos, delta, DR_BASE, DR_BASE)
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # Start gripping
        set_digital_output(2, OFF)
        set_digital_output(1, ON)

        # Force control to lower
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        fd = [0, 0, -10, 0, 0, 0]
        fctrl_dir = [0, 0, 1, 0, 0, 0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        while True:
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
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
            if check_force_condition(DR_AXIS_Z, min=3) == 0:
                release_force()
                wait(1)
                ungrip()
                wait(1)
                break
            wait(0.1)
            
        release_compliance_ctrl()

                
    # Set tool and TCP
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")

    # Move to ready position
    JReady = posj([0, 0, 90, 0, 90, 0])  # 시작 위치
    movej(JReady, vel=VELOCITY, acc=ACC)
    while rclpy.ok():
        # 피킹 위치에서 pick and place 실행
        pick_and_place(pick_pos, place_pos)
    
                    
        break  # 한 번 실행 후 종료

    movej(JReady, vel=VELOCITY, acc=ACC)

if __name__ == "__main__":
    main()
