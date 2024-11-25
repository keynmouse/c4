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
            check_position_condition,
            task_compliance_ctrl,
            get_current_posx,
            set_digital_output,
            set_desired_force,
            amove_periodic,
            parallel_axis,
            release_force,
            set_tool,
            set_tcp,
            movel,
            movej,
            trans,
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
    
    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")
    #작은기어 잡는 위치
    movej([0,0,90,0,90,0],vel=VELOCITY,acc=ACC)
    
    posx_small_gear_start = [423.194, -153.798, 93.123, 16.526, -179.447, 16.253]
    movel(posx_small_gear_start,vel=VELOCITY,acc=ACC,ref=DR_BASE)
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
    posx_small_gear_z,_=get_current_posx(0)
    delta1=[0,0,-15,0,0,0]
    posx_small_gear_pick=trans(posx_small_gear_z,delta1,DR_BASE,DR_BASE)
    set_digital_output(1,OFF)
    set_digital_output(2,ON)
    wait(1)
    movel(posx_small_gear_pick,vel=VELOCITY,acc=ACC,ref=DR_BASE)
    set_digital_output(2,OFF)
    set_digital_output(1,ON)
    wait(1)
    delta2=[0,0,115,0,0,0]
    posx_small_gear_up=trans(posx_small_gear_z,delta2,DR_BASE,DR_BASE)
    movel(posx_small_gear_up,vel=VELOCITY,acc=ACC,ref=DR_BASE)

    posx_small_gear_drop= [428.155, 146.129, 91.963, 13.999, 180.000, 13.695]
    movel(posx_small_gear_drop,vel=VELOCITY,acc=ACC,ref=DR_BASE)

    task_compliance_ctrl(stx=[2000,2000,500,3000,3000,3000])
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0.1, mod=DR_FC_MOD_REL)
    while True:
        if check_force_condition(DR_AXIS_Z,min=7)==0:
            print('1')
            if check_position_condition(DR_AXIS_Z,min=50,ref=DR_BASE)==0:
                print('2')
                amove_periodic(amp=[0, 0, 0, 0, 0, 30], period=1.0, atime=0.02, repeat=3, ref=DR_TOOL)
                print('3')
            else:
                print('4')
                release_force()
                break
    print('5')
    release_compliance_ctrl()
    
    set_digital_output(1,OFF)
    set_digital_output(2,ON)
    wait(1)
    movej([0,0,90,0,90,0],vel=VELOCITY,acc=ACC)
    
    
    
    
    
    
    
    
    
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()
