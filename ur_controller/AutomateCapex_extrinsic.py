import time

import numpy as np
import spatialmath as sm

from config import T_W_BASE, T_W_DEFAULT, T_W_S1 #T_BASE_S1, T_W_t1, T_W_t2
from robot import Robot
from utils import make_tf, se3_to_pose, pose_to_se3

class AutomateCapex:
    def __init__(self, robot_ip: str, simulation: bool = False):
        self.robot = Robot(robot_ip, simulation)
        self.T_W_BASE = T_W_BASE
        self._init_pose = T_W_DEFAULT.t
        self.wrench_data = self.robot.recv.getActualTCPForce()
        print("TCP : \n", self.robot.T_base_tcp)
        self.wrench_thrs = 0
        self.contact_pos = 0

    @property
    def init_pose(self):
        return self._init_pose

    def get_tcp_pose(self):
        return self.robot.T_base_tcp

    def pick_tube(self, tube_poses):
        """Pick up the tube at the given pose."""
        tube_d_up = np.array([tube_poses[0], tube_poses[1], tube_poses[2]+0.120])
        self.move_to_target(tube_d_up)
        
        self.move_to_target(tube_poses)
        
        self.robot.hande.move(255, 1, 255)
        tube_d_up = np.array([tube_poses[0], tube_poses[1], tube_poses[2]+0.120])
        time.sleep(2.5)
        self.move_to_target(tube_d_up)
        #self.move_to_target(T_W_S1.t, T_W_S1)
        pass

    def wire_insert(self):
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_Tcp.t[2] = T_W_Tcp.t[2]
        self.move_to_target(T_W_Tcp)

    def push_it_in(self):
        print("push it in")
        depth_step = 0
        xy_check = 0
        xy_flag = 0
        spiral = 0
        dirx = [1, 1, -1, -1]
        diry = [1, -1, -1, 1]
        max_z_dist = 190/5
        while True:
            self.wrench_data = self.robot.recv.getActualTCPForce()
            Fx = self.wrench_data[0]
            Fy = self.wrench_data[1]
            Fz = self.wrench_data[2] 
            if Fz > 5.5:
                if abs(Fx) > 2.0 or abs(Fy) > 2.0:
                    for i in range(0, 4):
                        if xy_flag == 0:
                            goal =  sm.SE3.Tx(0.002*dirx[xy_check]) @ T_W_S1 @ sm.SE3.Tz(0.0005*depth_step-0.001)
                            self.move_to_target(goal.t, goal, 0.002)
                            goal =  sm.SE3.Tx(0.002*dirx[xy_check]) @ T_W_S1 @ sm.SE3.Tz(0.0005*depth_step)
                            self.move_to_target(goal.t, goal, 0.002)
                            xy_flag = 1
                            time.sleep(0.1)
                        else:
                            goal = sm.SE3.Ty(0.002*diry[xy_check]) @ T_W_S1 @ sm.SE3.Tz(0.0005*depth_step-0.001) 
                            self.move_to_target(goal.t, goal, 0.002)
                            goal =  sm.SE3.Ty(0.002*diry[xy_check]) @ T_W_S1 @ sm.SE3.Tz(0.0005*depth_step)
                            self.move_to_target(goal.t, goal, 0.002)
                            xy_flag = 0
                            time.sleep(0.1)
                    xy_check +=1
                    if xy_check > 3:
                        xy_check = 0
                else:
                    depth_step +=1
                    goal = T_W_S1 @ sm.SE3.Tz(0.0005*depth_step)
                    self.move_to_target(goal.t, goal, 0.0025)
                    time.sleep(0.1)
            else:
                depth_step +=1
                goal = T_W_S1 @ sm.SE3.Tz(0.0005*depth_step)
                self.move_to_target(goal.t, goal, 0.0025)
                time.sleep(0.1)
            if depth_step > max_z_dist:
                break
        self.robot.hande.move(0, 1, 10)
        time.sleep(2)


    def tilt_it_in(self,target_hole,  speed_scalar, acceleration,tilt_amount,Tool_length, syringe):
        print("tilt it in")
        
        delta_p = 0.005

        if syringe:
            self.robot.ctrl.setTcp([0, 0, 0.160, 0, 0, 0])
            above_T_B_TCP = target_hole * sm.SE3(0,0,-delta_p)
            # Compute new pose after tilting around the tip
            Tilted_above_pose_TCP = above_T_B_TCP
        else:
            self.robot.ctrl.setTcp(Tool_length)
            Tilt = sm.SE3.Rx(np.deg2rad(tilt_amount))
            above_T_B_TCP = target_hole * sm.SE3(0,0,-delta_p)
            # Compute new pose after tilting around the tip
            Tilted_above_pose_TCP = above_T_B_TCP * Tilt

        # Move robot
        self.robot.moveL(Tilted_above_pose_TCP)

        self.robot.ctrl.setTcp(Tool_length)
        direction = above_T_B_TCP.t - target_hole.t 
        direction_unit = -(direction / np.linalg.norm(direction))

        time.sleep(0.2)
        self.robot.ctrl.zeroFtSensor()
        time.sleep(0.2)

        stop_force = 1.5
        max_distance = 0.1

        self.speed_until_force(direction_unit,stop_force,max_distance,speed_scalar,acceleration)
        
        task_frame = se3_to_pose(target_hole).tolist()
        selecetion_vector = [1, 1, 0, 0, 0, 0]
        wrench = [0, 0, 0, 0, 0, 0 ]
        type = 2
        limits = [0.01, 0.01, 0.1, 0.1, 0.1, 0.1]
        if syringe !=1:
            self.robot.ctrl.forceMode(task_frame,selecetion_vector,wrench,type,limits)
        untilted_pose = self.robot.T_base_tcp  * sm.SE3.Rx(np.deg2rad(-tilt_amount)) * sm.SE3(0,0,0.0025)
        
        self.robot.moveL(pose = untilted_pose,speed=0.01,acc=0.1)

        if syringe:
            direction_tcp = self.robot.T_base_tcp * sm.SE3(0,0, 0.003)
            self.robot.moveL(pose = direction_tcp,speed=0.01,acc=0.1)
        else:
            self.robot.ctrl.forceMode(task_frame,selecetion_vector,wrench,type,limits)
            direction_tcp = self.robot.T_base_tcp * sm.SE3(0,0,1)
            self.speed_until_force(direction_tcp.t,4,max_distance,speed_scalar,acceleration)

        self.robot.hande.move(0, 1, 255)

        self.robot.ctrl.forceModeStop()

        time.sleep(3)

        untilted_pose_above = self.robot.T_base_tcp * sm.SE3(0,0,-0.1)

        self.robot.moveL(pose = untilted_pose_above,speed=0.1,acc=0.1)

        self.robot.ctrl.setTcp([0,0,0,0,0,0])
        

    def speed_until_force(self, direction_unit, stop_force, max_distance, speed_scalar, acceleration):

        start_pos = self.robot.T_base_tcp.t
        speed_vector = speed_scalar * np.concatenate([direction_unit, np.zeros(3)])

        while True:
            
            self.robot.speedL(speed_vector,acceleration)

            # Get reading from FT Sensor
            base_force = self.robot.recv.getActualTCPForce()  # should return a 3-element list or array
            base_force = np.array(base_force[:3])

            # Project the force vector onto the direction unit vector using dot product
            force_direction = -np.dot(base_force, direction_unit)

            if force_direction > stop_force:
                print("Force was greater than the the stop force")
                self.robot.ctrl.speedStop()
                break
            
            current_pos = self.robot.T_base_tcp.t
            
            
            distance_traveled = np.linalg.norm(abs(current_pos - start_pos))

            if distance_traveled > max_distance:
                print("The gripper exceded the d_max threshold")
                self.robot.ctrl.speedStop()
                break
        

    def move_to_target(self, target_pos_w: np.array, rot: sm.SO3 = None, vel:int = None):        # self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])

        # self.pick_tube(tube_s2)t: sm.SO3 = None, vel:int = None):
        """Move the robot to the target position in world coordinates."""
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()

        # print("Tcp T in world :\n", T_W_Tcp)
        if rot is None:
            rot = T_W_DEFAULT.R
        if vel is None:
            vel = 0.1

        T_W_Target = make_tf(pos=target_pos_w, ori=rot)
        # print("Target T in world :\n", T_W_Target)
        T_BASE_Target = self.T_W_BASE.inv() @ T_W_Target
        # print("Target T in base :\n", T_BASE_Target)
        self.robot.moveL(T_BASE_Target, vel)

    def run(self):
        # Always go to the initial position first.
        #self.robot.hande.move(0, 5, 10)
        #self.move_to_target(self.init_pose, T_W_DEFAULT.R)
        """Main execution function."""

        reset_point = [0.700, -0.0750, 0.32]

        tube_s1 = np.array([0.600, -0.075, 0.080])
        tube_s2 = np.array([0.600, -0.125, 0.080])
        tilt_amount = 30
        tool_lenght = 0.160
        speed_scalar = 0.01
        acceleration = 0.01

        self.robot.ctrl.setTcp([0, 0, 0, 0, 0, 0])
        self.move_to_target(reset_point)
        
        self.robot.hande.move(0, 1, 255)

        # ##Syringe:
        tube_s3 = np.array([0.645, -0.100, 0.075])
        position_3 = [0.35250, -0.43537, 0.27845]#[0.35730, -0.34022, 0.24680]#[0.2560, -0.60202, 0.09510]
        rotation_vector_3 = [0.074, 3.053, -0.720]#[0.033, -3.152, 0.296]

        self.move_to_target(reset_point)

        self.robot.hande.move(0, 1, 255)

        self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])

        self.pick_tube(tube_s3)

        # go side
        cable_position_in = [0.45168, -0.06122, 0.126]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        # go lower
        cable_position_in = [0.45168, -0.06122, 0.028]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        # go in
        cable_position_in = [0.4772, -0.081, 0.028]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        # go up
        cable_position_in = [0.4772, -0.081, 0.078]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.01,acc=0.1)
        
        # go down
        cable_position_in = [0.45168, -0.06122, 0.028]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.01,acc=0.1)

        # move up for hoel
        cable_position_in = [0.3483, -0.17892, 0.31060]
        cable_orientation = [2.926, 1.143, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)
        
        # reach the hole
        cable_position_in = [0.35248, -0.43029, 0.28369]
        cable_orientation = [0.074, 3.141, 0]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)
        Tool = [0, 0.065, 0.208, -0.872665, 0, 0]
        # Tool = [0, 0.072, 0.152, -1.5707, 0, 0]

        hole3_T_B_TCP = pose_to_se3(position=cable_position_in, rotation=cable_orientation)
        tilt_amount = -47
        self.tilt_it_in(hole3_T_B_TCP,speed_scalar,acceleration,tilt_amount,Tool, 1)

        self.move_to_target(reset_point)


        # Hole 1:
        tilt_amount = 30
        position = [0.35388, -0.34345, 0.24572 ] #[0.24798, -0.59093, 0.09410]
        rotation_vector = [2.912, 0.023, 0.027 ]

        hole_T_B_TCP = pose_to_se3(position=position, rotation=rotation_vector)
        

        self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])



        self.pick_tube(tube_s1)

        self.move_to_target(reset_point)

        Tool_length_with_tube = 0.160+0.075
        
        Tool = [0, 0, Tool_length_with_tube, 0, 0, 0]

        self.tilt_it_in(hole_T_B_TCP,speed_scalar,acceleration,-tilt_amount, Tool, 0)
        
        self.move_to_target(reset_point)


        # Hole 2:
        position_2 = [0.366, -0.35412, 0.25374]#[0.2560, -0.60202, 0.09510]
        rotation_vector_2 = [2.310, 2.40, -0.323]

        hole2_T_B_TCP = pose_to_se3(position=position_2, rotation=rotation_vector_2)

        self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])

        self.pick_tube(tube_s2)

        self.move_to_target(reset_point)

        self.tilt_it_in(hole2_T_B_TCP,speed_scalar,acceleration,tilt_amount, Tool, 0)
    
        self.move_to_target(reset_point)

        # cable

        cable_position = [0.42040, -0.14548, 0.46120]
        cable_orientation = [0.980, 2.485, -2.498]
        cable_pose = pose_to_se3(cable_position, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        cable_position_reach = [0.31838, -0.39390, 0.45925]
        cable_orientation = [1.047, 2.497, -2.499]
        cable_pose = pose_to_se3(cable_position_reach, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)
        self.robot.hande.move(255, 1, 255)
        time.sleep(3)

        cable_position_back = [0.45071, -0.22245, 0.43162]
        cable_orientation = [1.047, 2.498, -2.499]
        cable_pose = pose_to_se3(cable_position_back, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        cable_position_in = [0.46171, -0.21170, 0.382]
        cable_orientation = [1.047, 2.498, -2.499]
        cable_pose = pose_to_se3(cable_position_in, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)
        self.robot.hande.move(0, 1, 255)
        time.sleep(3)

        cable_position_up = [0.46171, -0.21170, 0.432]
        cable_orientation = [1.047, 2.498, -2.499]
        cable_pose = pose_to_se3(cable_position_up, cable_orientation)

        self.robot.moveL(cable_pose,speed=0.1,acc=0.1)

        self.move_to_target(reset_point)


if __name__ == "__main__":

    automate = AutomateCapex("192.168.0.12", True)
    automate.robot.recv.startFileRecording("robot_data.csv")
    try:
        automate.run()
    except KeyboardInterrupt:
        automate.robot.ctrl.speedStop()
        
        print("Program interrupted by user, shutting down.")

        time.sleep(2) 
        automate.robot.ctrl.forceModeStop()
        #automate.robot.ctrl.setTcp([0, 0, 0, 0, 0, 0])

    automate.robot.ctrl.stopScript()

    automate.robot.recv.stopFileRecording()