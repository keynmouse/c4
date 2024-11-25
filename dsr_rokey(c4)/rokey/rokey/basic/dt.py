import rclpy
import DR_init
import copy

# For single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("dt", namespace=ROBOT_ID)

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
            wait,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
        )

        from DR_common2 import posx

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def g_d(pos_init, i):
        # Grip the object
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        wait(0.5)
        
        # Move to force control position (1cm up in Z)
        posx_force_back = copy.deepcopy(pos_init)
        posx_force_back[0] += 26 * i
        posx_force_back[2] += 10
        movel(posx_force_back, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # Move to drop axis position
        parallel_axis([0, 0, -1], DR_AXIS_Z, DR_BASE)
        
        # Ungrip (temporary)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.5)
        
        # Move to the specified end position after dropping the block
        end_pos1 = posx(
            [400.0038146972656, -3.395210027694702, 34.57353973388672,
             171.94117736816406, 179.9988250732422, 171.395751953125]
        )
        movel(end_pos1, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # Optional: Add any additional steps if needed after moving to end_pos1

    set_tool("Tool Weight_test_1")
    set_tcp("GripperSA_v1_test1")
    print('1')
    
    # Move to Jenga's upper position (center of the Jenga tower)
    posx_start = posx([333.763, 201.292, 277.584, 113.677, 178.065, 114.606])
    movel(posx_start, vel=VELOCITY, acc=ACC, ref=DR_BASE)
    
    # Enable force control
    task_compliance_ctrl(stx=[2000, 2000, 500, 3000, 3000, 3000])
    set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], time=0.1, mod=DR_FC_MOD_REL)
    
    # Wait until the force condition is met
    while True:
        if check_force_condition(DR_AXIS_Z, min=7) == 0:
            release_force()
            break
    release_compliance_ctrl()
    wait(1)
    
    # Get the current position
    posx_force, _ = get_current_posx(0)
    print(posx_force)
    
    # Perform the pick-and-place operation three times
    for i in range(3):
        g_d(posx_force, i)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
