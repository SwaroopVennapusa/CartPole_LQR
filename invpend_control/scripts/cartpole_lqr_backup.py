#!/usr/bin/env python3
import rospy
import numpy as np
# import control as ct
import control

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Point
# from gazebo_msgs.msg import LinkState

# MASS_CART = 20
# MASS_POLE = 2
# LENGTH_POLE = 0.5
# RADIUS_POLE = 0.025
# INERTIA = 0.005
# g = 9.81

# den = INERTIA*(MASS_CART + MASS_POLE) + MASS_CART * MASS_POLE * LENGTH_POLE**2
# A = np.matrix([[0,1,0,0],
#                [0, 0, -(MASS_POLE**2 * g * LENGTH_POLE**2)/den, 0],
#                [0, 0, 0, 1],
#                [0, 0, MASS_POLE * g * LENGTH_POLE * (MASS_CART + MASS_POLE)/den, 0]
#                ])
# B = np.matrix([0, (INERTIA + MASS_POLE * LENGTH_POLE**2)/den, 0, -MASS_POLE * LENGTH_POLE/den]).T
# C = np.matrix([[1,0,0,0], [0,0,1,0]])
# D = np.matrix([0,0]).T

# Q = np.diag([1, 1, 1, 1])
# R = np.diag([0.1])

# K, S, E = ct.lqr(A, B, Q, R)

class CartPoleController():
    def __init__(self):
        # Define System Parameters
        self.m_c = 20       # Cart mass
        self.m_p = 2        # pole mass
        self.g = 9.81       # acceleration due to gravity
        self.l = 0.5        # pole length

        # Define State Variables
        self.x = 0          # cart position
        self.x_dot = 0      # cart velocity
        self.theta = 0      # pole angle
        self.theta_dot = 0  # pole angular velocity

        self.state = 0
        self.desired_state = np.matrix([1,0,0,0]).T

        # Define LQR
        self.Q = np.diag([1,1,1,1])
        self.R = np.diag([0.1])
        self.A = np.matrix([[0,1,0,0],
                       [0,0,-1*(self.m_p * self.g)/self.m_c,0],
                       [0,0,0,1],
                       [0,0,((self.m_c+self.m_p)*self.g)/(self.m_c*self.l),0]])
        self.B = np.matrix([0,1/self.m_c,0,-1/(self.m_c*self.l)]).T
        self.C = np.matrix([[1,0,0,0], [0,0,1,0]])
        self.D = np.matrix([0,0]).T

        self.K = control.lqr(self.A,self.B,self.Q,self.R)[0]
        self.K,S,E = control.lqr(self.A,self.B,self.Q,self.R)

        # Define ROS publisher and subscriber
        self.currentStateSub = rospy.Subscriber('/invpend/joint_states', JointState, self.state_callback)
        self.pub_velocity = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)

    """To check the current state"""
    def state_callback(self,msg):
        self.theta = msg.position[0]
        self.theta_dot = msg.velocity[0]
        self.x = msg.position[1]
        self.x_dot = msg.velocity[1]
        self.state = np.matrix([self.x,self.x_dot,self.theta,self.theta_dot]).T

    """Linear control implementation"""
    def control(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            x_e = self.state - self.desired_state
            self.u = -1 * np.matmul(self.K, x_e) # u = -kx
            cmd_vel = float(self.u)
            self.pub_velocity.publish(cmd_vel) 
            # print(self.state)
            rate.sleep()
        # rospy.spin()


    def controlability(self):
        # import pdb; pdb.set_trace()
        # self.sys = control.StateSpace(self.A, self.B, [],[]) # Empty matrices are C and D
        sys = control.ss(self.A, self.B, self.C, self.D)
        # print(sys)
        # ctrb_mat, rank = control.ctrb(sys.A, sys.B)             # Returns controlability matrix and its rank
        ctrb_mat = control.ctrb(sys.A,sys.B)
        # self.rank = control.ctrb(self.sys)[1]                      
        if np.linalg.matrix_rank(ctrb_mat) == np.shape(self.A)[0]:                  # Check if it is full rank
            self.control()


if __name__ == '__main__':
    rospy.init_node('cart_pole_lqr_controller')
    controller = CartPoleController()
    controller.controlability()
    # controller.control()
    rospy.spin()
    # CartPoleController.control()