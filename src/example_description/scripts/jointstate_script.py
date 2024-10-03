#!/usr/bin/python3
import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped ,Twist
from std_msgs.msg import Float64MultiArray , Bool ,Float64 ,String
from spatialmath import *
from roboticstoolbox import DHRobot, RevoluteMDH
from spatialmath import SE3

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')
        '''PUB'''
        self.end_effector_pub = self.create_publisher(PoseStamped, "/end_effector", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.trigger_pub = self.create_publisher(Bool, "trigger", 10)
        self.singularity_pub = self.create_publisher(String, 'singularity_warning', 10)
        '''SUB'''
        self.robogo_sub = self.create_subscription(Float64MultiArray, "/robogo",self.robogo_callback, 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states",self.joint_sub_callback, 10)
        self.mode_sub = self.create_subscription(Float64, "mode_topic" , self.mode_callback,10)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel" ,self.cmd_vel_callback,10)
        '''INIT'''
        self.dt = 0.1
        self.create_timer(self.dt, self.sim_loop)
        self.q = [0.0, 0.0, 0.0]
        self.qq = [0.0, 1.57, -1.57]
        self.v = [0.0, 0.0, 0.0]
        self.q
        self.name = ["joint_1", "joint_2", "joint_3"]
        self.a = [0, 0, 0.25, 0.28]
        self.d = [0.24, -0.12, 0.1, 0]
        self.alpha = [0, -np.pi/2, 0, np.pi/2]
        self.target_reached = False
        self.last_target = None
        self.end_effector_positions = []
        self.current_mode = 0
        self.plot_workspace()
        self.in_singularity = False
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

    def cmd_vel_callback(self, msg: Twist):
        self.v = [msg.linear.x,msg.linear.y,msg.linear.z]

    def mode_callback(self, msg:Float64):
        self.current_mode = msg.data

    def robogo_callback(self, msg :Float64MultiArray):
        self.qq[0] = msg.data[0]
        self.qq[1] = msg.data[1]
        self.qq[2] = msg.data[2]

    def joint_sub_callback(self, msg :JointState):
        self.q[0] = msg.position[0]
        self.q[1] = msg.position[1]
        self.q[2] = msg.position[2]
        end_effector_pose = PoseStamped()
        position = self.forward(self.q)

        end_effector_pose.header.stamp = self.get_clock().now().to_msg()
        end_effector_pose.header.frame_id = 'link_0'
        end_effector_pose.pose.position.x = float(position[0])
        end_effector_pose.pose.position.y = float(position[1])
        end_effector_pose.pose.position.z = float(position[2])

        self.end_effector_pub.publish(end_effector_pose)

        # print(f"Published end effector position:")
        # print(f"    x: {position[0]}")
        # print(f"    y: {position[1]}")
        # print(f"    z: {position[2]}")
        # print("-" * 50)

    def sim_loop(self):
        if self.current_mode == 2.1:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            linear_velocity = np.array(self.v)
            linear_velocity_6d = np.array([linear_velocity[0], linear_velocity[1], linear_velocity[2], 0, 0, 0])
            J = self.dh_model.jacobe(self.q)
            det_J = np.linalg.det(J[:3, :3])
            threshold = 2e-3
            if abs(det_J) < threshold:
                if not self.in_singularity:
                    self.get_logger().warn('Robot is in singularity! Movement stopped.')
                    warning_msg = String()
                    warning_msg.data = "Robot is in singularity. Movement stopped."
                    self.singularity_pub.publish(warning_msg)
                    self.in_singularity = True  
                J_new = self.dh_model.jacob0(self.q + np.linalg.pinv(J).dot(linear_velocity_6d) * 0.1)
                det_J_new = np.linalg.det(J_new[:3, :3])
                if abs(det_J_new) > abs(det_J):
                    joint_velocity = np.linalg.pinv(J).dot(linear_velocity_6d)
                    self.q[0] += (joint_velocity[0]) * 0.1
                    self.q[1] += (joint_velocity[1]) * 0.1
                    self.q[2] += (joint_velocity[2]) * 0.1
                else:
                    pass
            else:
                self.in_singularity = False
                joint_velocity = np.linalg.pinv(J).dot(linear_velocity_6d)
                self.q[0] += (joint_velocity[0]) * 0.1
                self.q[1] += (joint_velocity[1]) * 0.1
                self.q[2] += (joint_velocity[2]) * 0.1
            
        elif self.current_mode == 2.2:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            linear_velocity = np.array(self.v)
            linear_velocity_6d = np.array([linear_velocity[0], linear_velocity[1], linear_velocity[2], 0, 0, 0])
            J = self.dh_model.jacob0(self.q)
            det_J = np.linalg.det(J[:3, :3])
            threshold = 2e-3
            if abs(det_J) < threshold:
                if not self.in_singularity:
                    self.get_logger().warn('Robot is in singularity! Movement stopped.')
                    warning_msg = String()
                    warning_msg.data = "Robot is in singularity. Movement stopped."
                    self.singularity_pub.publish(warning_msg)
                    self.in_singularity = True  
                J_new = self.dh_model.jacob0(self.q + np.linalg.pinv(J).dot(linear_velocity_6d) * 0.1)
                det_J_new = np.linalg.det(J_new[:3, :3])
                if abs(det_J_new) > abs(det_J):
                    joint_velocity = np.linalg.pinv(J).dot(linear_velocity_6d)
                    self.q[0] += (joint_velocity[0]) * 0.1
                    self.q[1] += (joint_velocity[1]) * 0.1
                    self.q[2] += (joint_velocity[2]) * 0.1
                else:
                    pass
            else:
                self.in_singularity = False
                joint_velocity = np.linalg.pinv(J).dot(linear_velocity_6d)
                self.q[0] += (joint_velocity[0]) * 0.1
                self.q[1] += (joint_velocity[1]) * 0.1
                self.q[2] += (joint_velocity[2]) * 0.1
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            self.q[0] += (self.qq[0] - self.q[0]) * self.dt
            self.q[1] += (self.qq[1] - self.q[1]) * self.dt
            self.q[2] += (self.qq[2] - self.q[2]) * self.dt

            delta_q = [abs(self.qq[i] - self.q[i]) for i in range(3)]
            goal_msg = Bool()
            if all(dq <= 0.01 for dq in delta_q):
                if not self.target_reached:
                    self.get_logger().info('Target reached!')
                    self.target_reached = True
                goal_msg.data = True
                self.trigger_pub.publish(goal_msg)
            else:
                goal_msg.data = False
                self.trigger_pub.publish(goal_msg)
                self.target_reached = False

        for i in range(len(self.q)):
            msg.position.append(self.q[i])
            msg.name.append(self.name[i])
        self.joint_pub.publish(msg)
        
    def plot_workspace(self):
        x = np.linspace(-0.53, 0.53, 30)
        y = np.linspace(-0.53, 0.53, 30)
        z = np.linspace(-0.33, 0.73, 30)
        X, Y, Z = np.meshgrid(x, y, z)
        r = 0.03
        phi, theta = np.mgrid[0:2*np.pi:15j, 0:np.pi:15j]
        non_x = r * np.outer(np.cos(phi), np.sin(theta))
        non_y = r * np.outer(np.sin(phi), np.sin(theta))
        non_z = r * np.outer(np.ones(np.size(phi)), np.cos(theta)) + 0.2
        condition1 = (X**2 + Y**2 + (Z - 0.2)**2) >= 0.03**2
        condition2 = (X**2 + Y**2 + (Z - 0.2)**2) <= 0.53**2
        region = condition1 & condition2
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(X[region], Y[region], Z[region], color='blue', s=1, alpha=0.1, label='Workspace')
        ax.plot_surface(non_x, non_y, non_z, color='red', alpha=0.5, label='Non-Workspace')
        ax.text(0.4, 0.4, 0.5, "Workspace", color='blue', fontsize=10)
        ax.text(0.4, 0.4, 0.3, "Non-Workspace", color='red', fontsize=10)
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.set_title('3D Workspace Visualization')
        plt.show()
        plt.show()

    def forward(self, joint_angles):
        joint_angles = np.array(joint_angles)
        T = self.dh_model.fkine(joint_angles)
        position = T.t
        return position

    def inverse(self, target_position):
        target_transform = SE3(target_position[0], target_position[1], target_position[2])
        q_solution = self.dh_model.ikine_LM(target_transform,mask = [1,1,1,0,0,0])
        if q_solution is None:
            self.get_logger().warn("No valid solutions found.")
            return None
        else:
            fk_solution = self.dh_model.fkine(q_solution.q)
            error = fk_solution.t - target_position
            print(f"IK solution: {q_solution.q}, Error: {error}")
            return q_solution.q

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
