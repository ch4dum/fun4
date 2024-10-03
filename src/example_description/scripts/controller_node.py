#!/usr/bin/env python3
import rclpy
import numpy as np
from roboticstoolbox import DHRobot, RevoluteMDH
from std_msgs.msg import Float64MultiArray, Bool , Float64
from spatialmath import SE3
from fun4_interfaces.srv import SetMode, SetTarget
from rclpy.node import Node

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        '''SER'''
        self.srv = self.create_service(SetMode, 'set_mode', self.set_mode_callback)
        '''CLI'''
        self.random_cli = self.create_client(SetTarget, 'set_target')
        '''PUB'''
        self.robogo_pub = self.create_publisher(Float64MultiArray, "/robogo", 10)
        self.mode_pub = self.create_publisher(Float64, "mode_topic",10)
        '''SUB'''
        self.trigger_sub = self.create_subscription(Bool, "trigger", self.trigger_callback, 10)
        self.current_mode = None
        self.loop = self.create_timer(1, self.loop_mode)
        self.target_pose = None
        self.random = [0.0, 0.0, 0.0]
        self.moving = False
        self.goal_trig = False
        self.last_target = None
        '''DH'''
        self.T = np.array([[0, 0, 1, 0.28],
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
        self.tool = SE3(self.T)
        self.dh_model = DHRobot([
            RevoluteMDH(a=0      , alpha=0       , d=0.2 , offset=0),
            RevoluteMDH(a=0      , alpha=-np.pi/2, d=0.12   , offset=-np.pi/2),
            RevoluteMDH(a=0.25   , alpha=0       , d=-0.1, offset=0),
        ], tool=self.tool,
        name="my_robot")
    def trigger_callback(self, msg):
        self.goal_trig = msg.data

    def set_mode_callback(self, request, response):
        mode = request.mode.data
        mode_msg = Float64()
        mode_msg.data = mode
        self.mode_pub.publish(mode_msg)
        if mode in [1.0, 2.1, 2.2, 3.0]:
            self.current_mode = mode
            self.get_logger().info(f'Mode changed to: {self.current_mode}')
            response.success = True 

            if self.current_mode == 1.0:
                target_pose = (request.ipk.x, request.ipk.y, request.ipk.z)
                ik_response = self.inverse_pose_kinematics(target_pose)
                print(ik_response)
                if self.checkworkspace(target_pose) == False:
                    self.get_logger().warn("No valid IK solution found")
                    response.inverse = False
                else:
                    self.get_logger().info(f"IK solution found: {ik_response}")
                    response.inverse = True
                    ik_array = Float64MultiArray()
                    ik_array.data = ik_response.tolist()
                    self.robogo_pub.publish(ik_array)
                    self.get_logger().info(f"Moving to random target: {target_pose}")
        else:
            self.get_logger().warn('Invalid mode selected!')
            response.success = False
        return response

    def loop_mode(self):
        if self.current_mode == 3.0:
            if not self.moving:
                self.request_random_target()
            if self.goal_trig and self.last_target != (self.random[0], self.random[1], self.random[2]):
                self.moving = False
                self.last_target = (self.random[0], self.random[1], self.random[2])  # อัปเดตตำแหน่งล่าสุด

    def request_random_target(self):
        request = SetTarget.Request()
        request.give_me = True
        
        self.random_cli.call_async(request).add_done_callback(self.random_target_callback)

    def random_target_callback(self, future):
        response = future.result()
        if response.success:
            self.random[0] = response.random.x
            self.random[1] = response.random.y
            self.random[2] = response.random.z
            self.get_logger().info(f'Received random position: [{self.random[0]}, {self.random[1]}, {self.random[2]}]')
            target_pose = (self.random[0], self.random[1], self.random[2])
            ik_response = self.inverse_pose_kinematics(target_pose)
            if ik_response is not None:
                ik_array = Float64MultiArray()
                ik_array.data = ik_response.tolist()
                self.robogo_pub.publish(ik_array)
                self.moving = True 
                self.get_logger().info(f"---------Moving to target!!---------")
            else:
                self.get_logger().warn("No valid IK solution found")
        else:
            self.get_logger().warn('Failed to get random target position!')
    
    def checkworkspace(self,target_pose):
        if -0.53 <= target_pose[0] <= 0.53 and -0.53 <= target_pose[1] <= 0.53 and -0.33 <= target_pose[2] <= 0.73 and 0.03**2 <= target_pose[0]**2 + target_pose[1]**2 + (target_pose[2]-0.2)**2 <= 0.53**2:
            return True
        else:
            return False
            

    def inverse_pose_kinematics(self, target_pose):
        target_transform = SE3(target_pose[0], target_pose[1], target_pose[2])
        q_solution = self.dh_model.ikine_LM(target_transform, mask=[1, 1, 1, 0, 0, 0])
        if q_solution is None:
            return None
        else:
            return q_solution.q

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
