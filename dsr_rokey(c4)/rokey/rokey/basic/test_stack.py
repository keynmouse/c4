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
    node = rclpy.create_node("test_stack", namespace=ROBOT_ID)

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
            DR_MV_MOD_ABS,
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
    
    posx_drop_init=[810.13,-178.04,101.74,90,90,90]
    posx_take_init=[424.75, 280.38,224.14,90,90,90]
    posj_take_init=[-9.87,14.29,102.04,85.46,99.29,-26.72]
    n=3
    w=79
    h=95
    for i in range(n):
        for j in range(i+1):
            posx_cup=copy.deepcopy(posx_drop_init)
            posx_cup[1]+=-w*i+0.5*w*j
            posx_cup[2]+=h*j+10
            #__________
            movej(posj_take_init,vel=VELOCITY,acc=ACC)
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
            movel([0,0,-15,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
            set_digital_output(2,OFF)
            set_digital_output(1,ON)
            wait(1)
            movel([0,0,100,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
            movel([0,-100,0,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)
            
            movej([8.54,-3.26,97.39,55.16,-7.29,35],vel=VELOCITY,acc=ACC)

            #________________
            print(posx_cup)
            movel(posx_cup,vel=VELOCITY,acc=ACC,ref=DR_BASE)
            print("1")
            task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
            set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],time=0.1, mod=DR_FC_MOD_REL)
            while True:
                print('2')
                if check_force_condition(DR_AXIS_Z, min=3) == 0:
                    print('3')
                    release_force()
                    break
            release_compliance_ctrl()
            
            set_digital_output(1,OFF)
            set_digital_output(2,ON)
            wait(1)
            
            movel([-100,0,0,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_MV_MOD_REL)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
