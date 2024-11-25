import rclpy
import DR_init

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

pose = [[True for _ in range(3)] for _ in range(3)]

def get_level(z):
    pos_ranges = [[42.5, 48.2], # 45.2
                  [52.5, 58.2], # 55.2
                  [62.5, 68.2]]   #65.2
    for i in range(3):
        if pos_ranges[i][0] <= z and z <= pos_ranges[i][1]:
            return i


def get_sorted_pos(force_pos, End_Pallet_Pose):
    level = get_level(force_pos[2]) #z
    for i in range(3):
        if pose[level-1][i]:
            pose[level-1][i] = False
            return End_Pallet_Pose[(level)*3 + i]
    print("남은 자리 없음")
    return False

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
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

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
        delta = [0, 0, 60, 0, 0, 0]
        delta2 = [0, 0, 150, 0, 0, 0]
        start_hover = trans(start_pos, delta, DR_BASE, DR_BASE)
        start_hover2 = trans(start_pos, delta2, DR_BASE, DR_BASE)
        print(start_hover)
        # Move above the start position
        movel(start_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        print("4")
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        # movel(start_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        print("5")
        # Z축 힘 측정
        task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
        set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)
        print("6")
        
        # force_checked = check_force_condition(DR_AXIS_Z, max=10)
        while True:
            print('1')
            if check_force_condition(DR_AXIS_Z, min=3) == 0:

                print('2')
                release_force()
                
                # z 위치 측정
                break
        print('3')
        # if force_checked:
        release_compliance_ctrl()
        print("7")
        wait(1)
        current_posx, _ = get_current_posx(0)
        print(current_posx)
        # current_posx2 = get_current_posx()[0][:]
        # print(current_posx2)
        end_pos = get_sorted_pos(current_posx, End_Pallet_Pose)
        print(end_pos)
        end_hover = trans(end_pos, delta, DR_BASE, DR_BASE)
        # 블럭의 Z축으로 50mm만큼 물러남
        # retreat_pos = trans(start_pos, [0, 0, 50, 0, 0, 0], DR_BASE, DR_BASE)
        movel([0, 0, -30, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        ungrip()
        # 다시 내려옴
        movel([0, 0, 40, 0, 0, 0], vel=VELOCITY, acc=ACC, ref=DR_TOOL)
        grip()
        movel(start_hover2, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        # Move to the end position
        end_2=end_hover
        end_2[2]=start_hover2[2]
        movel(end_2, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        end_3=end_hover
        end_3[2]=current_posx[2]-10
        movel(end_3, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        ungrip()
        end_4=end_3
        end_4[2]=start_hover2[2]
        movel(end_4, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        movel(end_hover, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 포스 측정 실패 시 해당 블럭 건너뜀
        # release_compliance_ctrl()
        # print("Force not detected, skipping this block.")

    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")

    # 초기 위치로 이동하며 그립 상태로 대기
    # grip()

    movej(JReady, vel=VELOCITY, acc=ACC)

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
    # start_pos1 = posx(401.11, -3.47, 33.49, 177.19, 179.99, 176.08)
    # start_pos2 = posx(502.11, -3.32, 31.73, 175.47, 180, 175.06)
    # start_pos3 = posx(502.33, -107.99, 31.24, 165.33, -179.99, 167.27)
    # start_pos4 = posx(399.65, -106.12, 32.02, 159.72, -180, 159.09)


    start_pos1 = posx(400.0038146972656, -3.395210027694702, 34.57353973388672, 171.94117736816406, 179.9988250732422, 171.395751953125)

    start_pos2 = posx(500.6770935058594, -4.6781792640686035, 35.11067199707031, 179.20327758789062, 179.99581909179688, 178.54014587402344)

    start_pos3 = posx(501.7113037109375, -107.5072021484375, 35.23503112792969, 28.305377960205078, -179.99990844726562, 27.88985824584961)

    start_pos4 = posx(399.4409484863281, -106.31370544433594, 34.7200698852539, 163.5529327392578, 179.99754333496094, 162.99391174316406)


    end_pos1 = posx(401.62762451171875, 146.05162048339844, 29.781232833862305, 18.334856033325195, -179.99853515625, 19.242460250854492)
    end_pos2 = posx(503.81195068359375, 145.58648681640625, 29.871646881103516, 15.166122436523438, -179.99168395996094, 16.162363052368164)
    end_pos3 = posx(503.3501892089844, 43.731590270996094, 31.619985580444336, 2.6529629230499268, -179.99766540527344, 3.7084217071533203)
    end_pos4 = posx(401.0836486816406, 44.065673828125, 27.78842544555664, 5.642519474029541, -179.99728393554688, 6.708737373352051)


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
    # end_pos1 = posx(402.39, 146.69, 32.49, 20.69, 180, 19.48)
    # end_pos2 = posx(503.93, 146.41, 32.02, 15.01, 179.99, 16.21)
    # end_pos3 = posx(503.99, 43.62, 33.54, 4.28, -179.97, 3.74)
    # end_pos4 = posx(402.13, 42.56, 32.53, 30.95, -180, 30.7)

    # 두 번째 팔레트 포즈 계산
    for pallet_index in range(total_count):
        End_Pallet_Pose.append(get_pattern_point(end_pos1, end_pos2, end_pos3, end_pos4, pallet_index, direction, row, column, stack, thickness, point_offset))

    # 이동 반복 실행
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")
    Count = 0
    while rclpy.ok():
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
