import time

import numpy as np
import spatialmath as sm

from config_DTU import T_W_DEFAULT # T_W_S1 #T_BASE_S1, T_W_t1, T_W_t2, T_W_BASE
from robot import Robot
from utils import make_tf, se3_to_pose, pose_to_se3

class AutomateCapex:
    def __init__(self, robot_ip: str, simulation: bool = False):
        self.robot = Robot(robot_ip, simulation)
        # self.T_W_BASE = T_W_BASE
        # self._init_pose = T_W_DEFAULT.t 
        self.wrench_data = self.robot.recv.getActualTCPForce()
        print("TCP : \n", self.robot.T_base_tcp)
        self.wrench_thrs = 0
        self.contact_pos = 0
        self.debug_tube_num = 2

    @property
    def init_pose(self):
        return self._init_pose

    def get_tcp_pose(self):
        return self.robot.T_base_tcp

    def pick_tube(self, tube_poses):
        """Pick up the tube at the given pose."""
        self.move_to_target(tube_poses)

        tube_d_up = np.array([tube_poses[0], tube_poses[1], tube_poses[2]-0.190])
        self.move_to_target(tube_d_up)
        
        self.robot.hande.move(255, 1, 255)
        tube_d_up = np.array([tube_poses[0], tube_poses[1], tube_poses[2]])
        time.sleep(2.5)
        self.move_to_target(tube_d_up)
        pass

    def wire_insert(self):
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_Tcp.t[2] = T_W_Tcp.t[2]
        self.move_to_target(T_W_Tcp)

    def tilt_it_in(self,target_hole,  speed_scalar, acceleration, tilt_rotvec, Tool, syringe):
        print("tilt it in")
        
        self.robot.ctrl.setTcp(np.array([0, 0, 0.160, 0, 0, 0]))

        delta_p = 0.005
 
        above_T_B_TCP = target_hole
        unload_pose = above_T_B_TCP
        # Move robot
        self.robot.moveL(above_T_B_TCP)

        self.robot.ctrl.setTcp(Tool)

        target_hole_temp = target_hole.t
        direction = above_T_B_TCP.t - np.array([target_hole_temp[0], target_hole_temp[1], target_hole_temp[2] -0.005])
        direction_unit = -(direction / np.linalg.norm(direction))

        time.sleep(2)
        self.robot.ctrl.zeroFtSensor()
        time.sleep(5)

        stop_force = 5
        max_distance = 0.1

        self.speed_until_force(direction_unit, stop_force, max_distance, speed_scalar, acceleration)
        
        task_frame = se3_to_pose(target_hole).tolist()
        selecetion_vector = [1, 1, 0, 0, 0, 0]
        wrench = [0, 0, 0, 0, 0, 0 ]
        type = 2
        limits = [0.01, 0.01, 0.1, 0.1, 0.1, 0.1]

        self.robot.ctrl.forceMode(task_frame,selecetion_vector,wrench,type,limits)
        pose_curr = self.robot.T_base_tcp.t
        
        rot_goal = tilt_rotvec #[2.757, -1.138, 0.195]
        untilted_pose = pose_to_se3(position=pose_curr, rotation=rot_goal)
        
        time.sleep(2)
        self.robot.moveL(pose = untilted_pose,speed=0.01,acc=0.1)

        if syringe == 1:
            untilted_pose_above = self.robot.T_base_tcp * sm.SE3(0,0, 0.05) # syringe
            insertion_force = 4
            open = 130
            direction_tcp = sm.SE3(0,-1,0) * self.robot.T_base_tcp  #FIXME
            # direction_tcp = sm.SE3(0,0,-1) * self.robot.T_base_tcp  #syringe
        else:
            untilted_pose_above = self.robot.T_base_tcp * sm.SE3(0,0,-0.05) # tube
            insertion_force = 6
            open = 0
            direction_tcp = sm.SE3(0,0,-1) * self.robot.T_base_tcp  #syringe
            

        self.speed_until_force(direction_tcp.t, insertion_force, max_distance, speed_scalar, acceleration)
        time.sleep(2)
        self.robot.hande.move(open, 1, 255)
        self.robot.ctrl.forceModeStop()
        time.sleep(3)

        self.robot.moveL(pose = untilted_pose_above,speed=0.01,acc=0.01)
        self.robot.ctrl.setTcp([0,0,0.160,0,0,0])
        untilted_pose_above = self.robot.T_base_tcp
        return untilted_pose_above
        

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
        # TODO : ADD THE REACHABILITY CHECKER FOR LARGET INPUT

        # self.pick_tube(tube_s2)t: sm.SO3 = None, vel:int = None):
        """Move the robot to the target position in world coordinates."""
        T_W_Tcp = self.get_tcp_pose() #self.T_W_BASE @ 

        # print("Tcp T in world :\n", T_W_Tcp)
        if rot is None:
            rot = T_W_DEFAULT.R
        else:
            rot = rot

        if vel is None:
            vel = 0.1

        T_W_Target = make_tf(pos=target_pos_w, ori=rot)
        # print("Target T in world :\n", T_W_Target)
        T_BASE_Target = T_W_Target #self.T_W_BASE.inv() @ 
        print("Target T in base :\n", T_BASE_Target)
        self.robot.moveL(T_BASE_Target, vel)

    def run(self):
        """Main execution function."""

        reset_point = [-0.53564, -0.33244, 0.269]
        reset_ori = [1.738, 2.619, 0]

        tube_s1 = np.array([-0.51687, -0.3789, 0.250])
        tube_s2 = np.array([-0.53710, -0.33435, 0.250])
        tube_s3 = np.array([-0.56587, -0.37390, 0.230])
        tool_lenght = 0.160
        speed_scalar = 0.01
        acceleration = 0.01

        # Tool_length_with_tube = 0.160+0.081
        # Tool = [0, 0, Tool_length_with_tube, 0, 0, 0] # 3d printed
        Tool_length_with_glass = 0.160+0.081
        Tool = [0, 0, Tool_length_with_glass, 0, 0, 0] # 3d printed

        q = self.robot.get_q()
        self.robot.ctrl.setTcp(np.array([0, 0, 0.160, 0, 0, 0]))
        home_q = np.deg2rad(np.array([22.09, -90, 90, -90, -90, 0]))
        
        if q[0] < -0.785398:
            raise RuntimeError("too large change of motion. Move it manually")
        
        self.robot.moveJ(home_q)
        self.robot.hande.move(0, 1, 255)

        # ## Hole 1:
    
        self.move_to_target(reset_point)
        self.robot.hande.move(0, 1, 255)
        
        position = [0.40105, 0.44490, 0.43982 ]
        rotation_vector = [2.595, -1.107, 0.560 ]

        hole_T_B_TCP = pose_to_se3(position=position, rotation=rotation_vector)

        self.pick_tube(tube_s1)
        intermid_position = [-0.26795, -0.19715, 0.40607]
        intermid_rotvec   = [1.738, 2.619, 0]
        intermid_position = pose_to_se3(position=intermid_position, rotation=intermid_rotvec)
        self.robot.moveL(intermid_position)
        intermid_q = np.deg2rad(np.array([-145.14, -110, 87.81, -75.04, -90, -10.15]))
        self.robot.moveJ(intermid_q)

        tilt_rotvec = [2.757, -1.138, 0.195]
        self.tilt_it_in(hole_T_B_TCP, speed_scalar, acceleration, tilt_rotvec, Tool, 0)
        intermid_q = np.deg2rad(np.array([-90.14, -73, 5.81, -66.04, -95, -10.15]))
        self.robot.moveJ(intermid_q)
        intermid_q = np.deg2rad(np.array([-40.14, -73, 5.81, -66.04, -95, -10.15]))
        self.robot.moveJ(intermid_q)
        intermid_q = np.deg2rad(np.array([18.14, -73, 56.81, -66.04, -95, -7.15]))
        self.robot.moveJ(intermid_q)

        ##Syringe:

        position_3 = [0.33226, -0.3195, 0.2455]#[0.35730, -0.34022, 0.24680]#[0.2560, -0.60202, 0.09510]
        rotation_vector_3 = [2.310,2.317,-0.253]#[0.033, -3.152, 0.296]

        self.move_to_target(reset_point)
        self.robot.hande.move(170, 1, 255)
        time.sleep(1)

        self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])

        self.pick_tube(tube_s3)
        intermid_position = [-0.26795, -0.19715, 0.40607]
        intermid_rotvec   = [1.738, 2.619, 0]
        intermid_position = pose_to_se3(position=intermid_position, rotation=intermid_rotvec)
        self.robot.moveL(intermid_position)
        intermid_q = np.deg2rad(np.array([-120.14, -110, 87.81, -75.04, -90, 110.15]))
        self.robot.moveJ(intermid_q)
        intermid_q = np.deg2rad(np.array([-134.5, -75, 79, -95, -90, 110]))
        self.robot.moveJ(intermid_q)

        move_away = [0.35407, 0.5264, 0.360]
        self.move_to_target(move_away, self.robot.T_base_tcp.R)

        position = [0.33924, 0.43880, 0.40270]
        rotation_vector = [1.600, -2.381, -1.070]
        hole3_T_B_TCP = pose_to_se3(position=position, rotation=rotation_vector)

        tilt_rotvec = [0.25,-0.232, -2.599]

        Tool_syringe = [0, -0.074, 0.152, -1.5707, 0, 0]
        self.tilt_it_in(hole3_T_B_TCP, speed_scalar ,acceleration, tilt_rotvec, Tool_syringe, 1)

        intermid_q = np.deg2rad(np.array([-90.14, -73, 5.81, -66.04, -95, -10.15]))
        self.robot.moveJ(intermid_q)
        intermid_q = np.deg2rad(np.array([-40.14, -73, 5.81, -66.04, -95, -10.15]))
        self.robot.moveJ(intermid_q)
        intermid_q = np.deg2rad(np.array([18.14, -73, 56.81, -66.04, -95, -7.15]))
        self.robot.moveJ(intermid_q)

        self.move_to_target(reset_point)

        ## Hole 2:
        self.move_to_target(reset_point)
        self.robot.ctrl.setTcp([0, 0, tool_lenght, 0, 0, 0])

        self.pick_tube(tube_s2)
        self.move_to_target(reset_point)

        intermid_position = [-0.26795, -0.19715, 0.40607]
        intermid_rotvec   = [1.738, 2.619, 0]
        intermid_position = pose_to_se3(position=intermid_position, rotation=intermid_rotvec)
        self.robot.moveL(intermid_position)
        intermid_q = np.deg2rad(np.array([-130.14, -110, 87.81, -75.04, -90, -10.15]))
        self.robot.moveJ(intermid_q)
        
        position = [0.38622, 0.36197, 0.43004 ]
        rotation_vector = [3.241, 1.049, 0.921]
        hole2_T_B_TCP = pose_to_se3(position=position, rotation=rotation_vector)
        tilt_rotvec = [3.098, 1.040, 0.290] #[3.134, -1.399, 0.098 ] #[3.057, -1.359, 0.030]
        Tool_length_with_glass = 0.160+0.078
        Tool_long = [0, 0, Tool_length_with_glass, 0, 0, 0] # 3d printed
        hole2_unload = self.tilt_it_in(hole2_T_B_TCP, speed_scalar, acceleration, tilt_rotvec, Tool_long, 0)
        hole2_unload = hole2_unload * sm.SE3(0,0, 0.05)
        self.robot.moveL(hole2_unload, speed=speed_scalar, acc=acceleration)
        self.robot.hande.move(255, 1, 255)
        time.sleep(3)
        hole2_unload = hole2_unload * sm.SE3(0,0, -0.05)
        self.robot.moveL(hole2_unload, speed=speed_scalar, acc=acceleration)
        intermid_position = [0.25775, 0.35171, 0.48121]
        self.move_to_target(intermid_position)
        intermid_q = np.deg2rad(np.array([4, -100, 81, -77, -85, 0]))
        self.robot.moveJ(intermid_q)
        self.move_to_target(reset_point)
        tube_s2[2] = tube_s2[2] - 0.185

        self.move_to_target(tube_s2, vel=speed_scalar)
        time.sleep(3)
        self.move_to_target(reset_point)

        # put it back


if __name__ == "__main__":

    automate = AutomateCapex("192.168.0.24", True)
    # automate.robot.recv.startFileRecording("robot_data.csv")
    try:
        automate.run()
    except KeyboardInterrupt:
        automate.robot.ctrl.speedStop()
        
        print("Program interrupted by user, shutting down.")

        time.sleep(2) 
        automate.robot.ctrl.forceModeStop()
        #automate.robot.ctrl.setTcp([0, 0, 0, 0, 0, 0])

    automate.robot.ctrl.stopScript()