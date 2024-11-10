#!/usr/bin/env python3
# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from phantomx_reactor.srv import KinematicGoal
# Python libraries
import numpy as np
from numpy.linalg import pinv
from modern_robotics import FKinSpace, ScrewToAxis

class KinematicsServer(Node):
    def __init__(self):
        super().__init__("kinematics_server")
        print("\nWaiting for a client's request")

        p1 = np.array([0.000, 0.000, 0.081])
        p2 = np.array([0.000, 0.000, 0.107])
        p3 = np.array([0.038, 0.000, 0.252])
        p4 = np.array([0.192, 0.000, 0.252])
        p5 = np.array([0.313, 0.002, 0.252])

        ω1 = np.array([0, 0, 1])
        ω2 = np.array([0, 1, 0])
        ω3 = np.array([0, 1, 0])
        ω4 = np.array([0, 1, 0])
        ω5 = np.array([1, 0, 0])

        S1 = ScrewToAxis(p1, ω1, 0)
        S2 = ScrewToAxis(p2, ω2, 0)
        S3 = ScrewToAxis(p3, ω3, 0)
        S4 = ScrewToAxis(p4, ω4, 0)
        S5 = ScrewToAxis(p5, ω5, 0)

        self.S_list = np.array([S1, S2, S3, S4, S5]).T

        self.M = np.array([
            [ 1.000,  0.000, 0.000, 0.313],
            [ 0.000,  1.000, 0.000, 0.002],
            [ 0.000,  0.000, 1.000, 0.252],
            [ 0.000,  0.000, 0.000, 1.000]
        ])

        self.q_solv = np.array([0.00, 0.00, 0.00, 0.00, 0.00])

        self.kinematics_srv = self.create_service(
            KinematicGoal, 
            "/kinematics", 
            self.kinematics_callback
        )

        self.rviz_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.rviz_msg = JointState()
        self.rviz_msg.name = [
            "shoulder_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "gripper_revolute_joint",
            "gripper_right_joint",
            "gripper_left_joint"
        ]
        #self.rviz_timer = self.create_timer(0.01, self.publish_joints_in_rviz)

        self.controller_topic = "/joint_group_position_controller/commands"
        self.controller_pub = self.create_publisher(Float64MultiArray, self.controller_topic, 10)
        self.controller_msg = Float64MultiArray()
    
    def kinematics_callback(self, request, response):
        goal = [request.x, request.y, request.z]
        goal = np.array(goal)
        self.q_solv = self.compute_ikine(goal)

        self.controller_msg.data = [
            self.q_solv[0],
            self.q_solv[1],
            self.q_solv[2],
            self.q_solv[3],
            self.q_solv[4]
        ]
        self.controller_pub.publish(self.controller_msg)
        
        response.q = self.controller_msg.data
        
        return response
    
    def publish_joints_in_rviz(self):
        self.rviz_msg.position = [
            self.q_solv[0],
            self.q_solv[1],
            self.q_solv[2],
            self.q_solv[3],
            self.q_solv[4],
            0.0,
            0.0,
            0.0
        ]
        self.rviz_msg.header.stamp = self.get_clock().now().to_msg()
        self.rviz_pub.publish(self.rviz_msg)

    # Get position from homogeneus transformation matrix
    def get_xyz_from_htm(self, fkine_mat):
        x = fkine_mat[0, 3]
        y = fkine_mat[1, 3]
        z = fkine_mat[2, 3]
        
        xyz = np.array([x, y, z])
        return xyz
    
    # Compute jacobian numerically
    def compute_jacobian(self, q_solv, goal):
        delta = 1/1000
        goal_length = len(goal)
        q_length = len(q_solv)
        J_shape = (goal_length, q_length)
        J_curr = np.zeros(J_shape)

        for k in range(q_length):
            q_plus = np.copy(q_solv)
            q_plus[k] += delta

            fkine_mat = FKinSpace(self.M, self.S_list, q_solv)
            fkine_coor = self.get_xyz_from_htm(fkine_mat)
            
            fkine_plus_mat = FKinSpace(self.M, self.S_list, q_plus)
            fkine_plus_coor = self.get_xyz_from_htm(fkine_plus_mat)
            
            J_curr[:, k] = (1/delta) * (fkine_plus_coor - fkine_coor)
        
        return J_curr
    
    # Compute inverse kinematics numerically
    def compute_ikine(self, goal):
        # Numerical parameters
        max_iters = 500
        e_max = 0.1/100 # Percentual error
        i = 0
        e_curr = 0
        e_norm = 1.0
        while e_norm > e_max and i < max_iters:
            J_curr = self.compute_jacobian(self.q_solv, goal)
            J_inv = pinv(J_curr)

            fkine_curr = FKinSpace(self.M, self.S_list, self.q_solv)
            xyz_curr = self.get_xyz_from_htm(fkine_curr)
            
            # Compute new q-values
            e_curr = goal - xyz_curr
            self.q_solv += 0.1 * np.dot(J_inv, e_curr)
            e_norm = np.linalg.norm(e_curr)

            print(f"Error #{i}: {np.around(e_norm * 100, 2)} %")
            i += 1
        return self.q_solv

def main():
    rclpy.init()
    node = KinematicsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()