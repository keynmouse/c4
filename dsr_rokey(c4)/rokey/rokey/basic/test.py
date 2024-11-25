# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import copy

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

        
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            check_force_condition,
            task_compliance_ctrl,
            get_current_posx,
            set_digital_output,
            set_desired_force,
            parallel_axis,
            release_force,
            set_tool,
            set_tcp,
            movel,
            movej,
            wait,
            DR_FC_MOD_REL,
            DR_MV_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            DR_TOOL,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    def g_d(pos_init,i):
        #그립
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        wait(0.5)
        #힘 제어 위치에서 1cm Z UP
        posx_force_back= copy.deepcopy(pos_init)
        posx_force_back[0]+= 26*i
        posx_force_back[2]+= 10   
        movel(posx_force_back,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #젠가 밀기위한 위치로 이동1
        posx_force_back2=posx_force_back
        posx_force_back2[1]+= 50
        posx_force_back2[0]-= 30
        movel(posx_force_back2,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #젠가 밀기위한 위치로 이동2
        posx_force_back3=posx_force_back2
        posx_force_back3[2]-= 17
        movel(posx_force_back3,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #젠가 밀기
        posx_force_back4=posx_force_back3
        posx_force_back4[1]-= 20
        movel(posx_force_back4,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #젠가 밀고난 후 잡는 위치로 이동1
        posx_force_back5=posx_force_back4
        posx_force_back5[2]+= 17
        posx_force_back5[0]+= 7
        movel(posx_force_back5,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #젠가 밀고난 후 잡는 위치로 이동2
        posx_force_back6=posx_force_back5
        posx_force_back6[1]-= 75
        movel(posx_force_back6,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #언그립
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(0.5)
        #젠가 밀고난 후 잡는 위치로 이동3
        posx_force_back7=posx_force_back6
        posx_force_back7[2]-= 17
        movel(posx_force_back7,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #그립 잡는 축으로 이동
        parallel_axis([0,1,0], DR_AXIS_Z, DR_BASE)
        #그립
        set_digital_output(2,OFF)
        set_digital_output(1,ON)
        wait(1.5)
        #그립후 상공
        posx_grip=posx_force_back7
        posx_grip[2]=posx_force_back7[2] + 100
        movel(posx_grip,vel=VELOCITY,acc=ACC,ref=DR_BASE)
        #놓는 축으로 이동
        parallel_axis([0,0,-1], DR_AXIS_Z, DR_BASE)
        
        #언그립 (임시용)
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait(1.5)

    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")
    print('1')
    #젠가 상공(중앙 젠가의 가운데)
    posx_start = posx([333.763, 201.292, 277.584, 113.677, 178.065, 114.606])
    #젠가 상공 이동
    movel(posx_start,vel=VELOCITY,acc=ACC,ref=DR_BASE)
    #힘 제어
    set_digital_output(2,OFF)
    set_digital_output(1,ON)
    task_compliance_ctrl(stx=[2000,2000,500,3000,3000,3000])
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0.1, mod=DR_FC_MOD_REL)
    while True:
        if check_force_condition(DR_AXIS_Z,min=7)==0:
            release_force()
            break
    release_compliance_ctrl()
    wait(1)
    posx_force,_ =get_current_posx(0)
    print(posx_force)
    x=copy.deepcopy(posx_force)
    x[2]+=10
    movel(x,vel=VELOCITY,acc=ACC,ref=DR_BASE)
    set_digital_output(1,OFF)
    set_digital_output(2,ON)
    wait(1)
    movej([0,0,0,0,0,90],vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
    x[2]-=25
    movel(x,vel=VELOCITY,acc=ACC,ref=DR_BASE)
    set_digital_output(2,OFF)
    set_digital_output(1,ON)
    wait(1.5)
    

    y=[0,0,-10,0,0,0]
    movel(y,vel=VELOCITY,acc=ACC,ref=DR_TOOL,mod=DR_MV_MOD_REL)
    z=[0,0,10,0,0,0]
    movej([0,0,0,0,0,-90],vel=VELOCITY,acc=ACC,mod=DR_MV_MOD_REL)
    movel(z,vel=VELOCITY,acc=ACC,ref=DR_TOOL,mod=DR_MV_MOD_REL)
    set_digital_output(1,OFF)
    set_digital_output(2,ON)
    wait(1.5)    
    
    '''
    for i in range(3):
        g_d(posx_force,i)
    '''

    rclpy.shutdown()


if __name__ == "__main__":
    main()
