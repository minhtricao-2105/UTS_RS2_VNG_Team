from ur3module import *

def gen_path(robot,q_goal,obstacle_list = None):

    tolerance = 0.1 # tolerance value (in rads) for maximum difference between a current joint state and the random joint state
    current_joint_state = robot.q
    
    iteration_joint = 0
    iteration_path = 0
    
    Kp = 0.1 # a proportional gain constant
    final_path = []

    print("--GENERATING PATH!")
    while iteration_path < 500:
        # path from current joint state to goal
        path = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = q_goal, t = 60)
        path_valid, all_valid = get_valid_path(robot,path,obstacle_list)
        final_path += path_valid

        if not all_valid:
            if bool(path_valid):
                current_joint_state = path_valid[-1]

            while iteration_joint < 50:
                iteration_joint += 1
                
                # generate a random joint state
                error = q_goal - current_joint_state # error between the current joint state and the goal joint state
                random_joint_state = current_joint_state + Kp * error+ np.random.normal(0, tolerance, robot.n)

                # clip joint angles to valid range
                random_joint_state = np.clip(random_joint_state, -np.pi, np.pi)
            
                if not is_joint_valid(robot,random_joint_state,obstacle_list)[0]: continue 
                else: # create a path to connect the current joint state with the random joint state
                    print(">Check subpath:")
                    subpath = rtb.mtraj(tfunc = rtb.trapezoidal, q0 = current_joint_state, qf = random_joint_state, t = 15)
                    subpath_valid, all_subpath_valid = get_valid_path(robot,subpath,obstacle_list)
                    
                    if all_subpath_valid:
                        final_path += subpath_valid
                        current_joint_state = random_joint_state
                        iteration_joint = 0    
                        break
                    else: continue
        else: 
            break

        iteration_path += 1

        print("STEP:",iteration_path)
        if np.linalg.norm(final_path[-1]- q_goal) <= 0.05: break
        
    if np.linalg.norm(final_path[-1]- q_goal) <= 0.05: 
        print("--GENERATE SUCCESSFULL!")
    else: 
        print("--GENERATE NOT SUCCESSFULL!")
        
    return final_path