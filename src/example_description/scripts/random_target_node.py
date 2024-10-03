#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from fun4_interfaces.srv import SetTarget
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3

class RandomTargetNode(Node):
    def __init__(self):
        super().__init__('random_target_node')
        '''PUB'''
        self.publisher_ = self.create_publisher(PoseStamped, '/target', 10)
        '''SER'''
        self.give_me_service = self.create_service(SetTarget, 'set_target', self.give_me_callback)
        '''INIT'''
        self.goal_trig = False
        self.random_generated = False
        self.a = [0, 0.12, 0.25, 0.28]
        self.d = [0.2, 0, 0.1, 0]
        '''DH'''
        T = np.array([[0, 0, 1, 0.28],
                       [1, 0, 0, 0],
                       [0, 1, 0, 0],
                       [0, 0, 0, 1]])
        tool = SE3(T)
        self.dh_model = DHRobot([
            RevoluteMDH(a=0, alpha=0, d=0.2, offset=0),
            RevoluteMDH(a=0, alpha=-np.pi/2, d=0.12, offset=-np.pi/2),
            RevoluteMDH(a=0.25, alpha=0, d=-0.1, offset=0),
        ], tool=tool, name="my_robot")


    def forward(self, joint_angles):
        T = self.dh_model.fkine(joint_angles)
        position = T.t
        return position

    def publish_random_target(self):
        theta = [np.random.uniform(-np.pi, np.pi), 
                 np.random.uniform(-np.pi, np.pi), 
                 np.random.uniform(-np.pi, np.pi)]
        position = self.forward(theta)
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = 'link_0'
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0
        self.publisher_.publish(target_pose)
        self.get_logger().info(f'Published target position: [{position[0]}, {position[1]}, {position[2]}]')
        return position
    
    def give_me_callback(self, request, response):
        if request.give_me: 
            position = self.publish_random_target()
            response.random.x = position[0]
            response.random.y = position[1]
            response.random.z = position[2]
            self.get_logger().info(f'Random position provided: [{position[0]}, {position[1]}, {position[2]}]')
            response.success = True 
        else:
            response.success = False 
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RandomTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
