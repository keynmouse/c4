import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            trans,
            DR_TOOL,
            DR_BASE,
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = posj([0, 0, 90, 0, 90, 0])  # 시작위치로

    # grip motion
    

    def grip():
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
       
    def ungrip():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
    
    def pick_and_place(start_pos, end_pos):
        delta = [0, 0, 100, 0, 0, 0]
        start_hover = trans(start_pos, delta, DR_BASE, DR_BASE)
        end_hover = trans(end_pos, delta, DR_BASE, DR_BASE)

        # Move above the start position
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(start_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        grip()
        wait(0.5)
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # Move to the end position
        movel(end_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(end_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        ungrip()
        wait(0.3)
        movel(end_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    def get_pattern_point(pos1, pos2, pos3, pos4, pallet_index, direction, row, column, stack, thickness, point_offset):
        # direction 0: Snake pattern 방식으로 팔레트 위치 계산
        if direction == 0:
            # 각 칸의 간격 계산 (양끝에서 중간까지의 길이 비례)
            x_spacing = (pos2[0] - pos1[0]) / (column - 1)
            y_spacing = (pos3[1] - pos1[1]) / (row - 1)

            x_offset = (pallet_index % column) * x_spacing
            y_offset = (pallet_index // column) * y_spacing
            z_offset = (pallet_index // (row * column)) * thickness

            x = pos1[0] + x_offset + point_offset[0]
            y = pos1[1] + y_offset + point_offset[1]
            z = pos1[2] + z_offset + point_offset[2]

            return [x, y, z, pos1[3], pos1[4], pos1[5]]
        else:
            raise ValueError("지원되지 않는 방향입니다. 현재는 direction=0 (Snake)만 지원됩니다.")

    # 첫 번째와 두 번째 포즈 그룹 정의
    Start_Pallet_Pose = []
    End_Pallet_Pose = []

    # 첫 번째 팔레트 포즈 설정
    start_pos1 = posx(401.11, -3.47, 33.49, 177.19, 179.99, 176.08)
    start_pos2 = posx(502.11, -3.32, 31.73, 175.47, 180, 175.06)
    start_pos3 = posx(502.33, -107.99, 31.24, 165.33, -179.99, 167.27)
    start_pos4 = posx(399.65, -106.12, 32.02, 159.72, -180, 159.09)

    direction = 0  # Normal Pallet -> 0: Snake, 1: Zigzag / Rhombus Pallet -> 2: Snake, 3: Zigzag
    row = 3
    column = 3
    stack = 1
    thickness = 0
    point_offset = [0, 0, 0]  # 오프셋 설정

    # 총 포인트 개수 계산
    if direction < 2:  # Normal Pallet
        total_count = row * column * stack
    else:  # Rhombus Pallet
        total_count = (row * column - int(row / 2)) * stack

    # 첫 번째 팔레트 포즈 계산
    for pallet_index in range(total_count):
        Start_Pallet_Pose.append(get_pattern_point(start_pos1, start_pos2, start_pos3, start_pos4, pallet_index, direction, row, column, stack, thickness, point_offset))

    # 두 번째 팔레트 포즈 설정
    end_pos1 = posx(402.39, 146.69, 32.49, 20.69, 180, 19.48)
    end_pos2 = posx(503.93, 146.41, 32.02, 15.01, 179.99, 16.21)
    end_pos3 = posx(503.99, 43.62, 33.54, 4.28, -179.97, 3.74)
    end_pos4 = posx(402.13, 42.56, 32.53, 30.95, -180, 30.7)

    # 두 번째 팔레트 포즈 계산
    for pallet_index in range(total_count):
        End_Pallet_Pose.append(get_pattern_point(end_pos1, end_pos2, end_pos3, end_pos4, pallet_index, direction, row, column, stack, thickness, point_offset))

    # 이동 반복 실행
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test_1")
    grip()
    movej(JReady, vel=VELOCITY, acc=ACC)
    Count = 0
    while rclpy.ok():
        grip()
        movej(JReady, vel=VELOCITY, acc=ACC)
        if (Count // 9) % 2 == 0:
            pick_and_place(Start_Pallet_Pose[Count % 9], End_Pallet_Pose[Count % 9])
            Count += 1
        else:
            pick_and_place(End_Pallet_Pose[8 - Count % 9], Start_Pallet_Pose[8 - Count % 9])
            Count += 1
            if Count % 18 == 0:
                break

    # 마지막 위치로 이동
    movej([0, 0, 90, 0, 90, 0], vel=VELOCITY, acc=ACC)

if __name__ == "__main__":
    main()
