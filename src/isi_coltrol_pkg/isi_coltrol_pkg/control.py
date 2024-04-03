import rclpy
from rclpy.node import Node

from time import sleep
from std_msgs.msg import String
import numpy as np

class PosControl(Node):
    
	def __init__(self):
		print()

def main(args=None):
    rclpy.init(args=args)
    node = PosControl()
    rclpy.spin(node)
    rclpy.shutdown()

#Aux functions
    
    
def control(xd_ant,xd,xe,ie_ant,Jacobian,timestamp):

	Kp_p = 6.0603
	Ki_p = 4.4567

	Kp_o = 2.8891
	Ki_o = 0.7579
	ve_d = (xd - xd_ant)/timestamp
	ie_ant = np.zeros((6,1))
	e = xd - xe
	ie = ie_ant + e*timestamp
	e_vec = np.concatenate((e, ie), axis=0)

	Ta_matrix = Ta(xe[4], xe[5])
	Ja = np.linalg.inv(Ta_matrix) @ Jacobian
	Kp = np.concatenate((Kp_p, Kp_o), axis=0)
	Ki = np.concatenate((Ki_p, Ki_o), axis=0)

	u = np.dot(np.concatenate((Kp, Ki), axis=1), e_vec)
	d_theta = np.linalg.inv(Ja) @ (u + ve_d)
	return d_theta
    
def Jacobiano_Escalador(theta1, theta2, theta3, theta4, theta5):


	J11 = -np.sin(theta5)*(np.cos(theta1)*np.sin(theta4) - np.cos(theta2)*np.cos(theta3)*np.cos(theta4)*np.sin(theta1) + np.cos(theta4)*np.sin(theta1)*np.sin(theta2)*np.sin(theta3)) - np.cos(theta2)*np.sin(theta1) - np.cos(theta2)*np.sin(theta1)*np.sin(theta3) - np.cos(theta3)*np.sin(theta1)*np.sin(theta2) - np.sin(theta2 + theta3)*np.cos(theta5)*np.sin(theta1)
	J12 = np.cos(theta1)*(np.cos(theta2 + theta3) - np.sin(theta2) + (np.sin(theta2 + theta3)*np.sin(theta4 + theta5))/2 + np.cos(theta2 + theta3)*np.cos(theta5) - (np.sin(theta2 + theta3)*np.sin(theta4 - theta5))/2)
	J13 = np.cos(theta1)*(np.cos(theta2 + theta3) + np.cos(theta2 + theta3)*np.cos(theta5) + np.sin(theta2 + theta3)*np.cos(theta4)*np.sin(theta5))
	J14 = -np.sin(theta5)*(np.cos(theta4)*np.sin(theta1) - np.cos(theta1)*np.cos(theta2)*np.cos(theta3)*np.sin(theta4) + np.cos(theta1)*np.sin(theta2)*np.sin(theta3)*np.sin(theta4))
	J15 = np.cos(theta1)*np.cos(theta4)*np.cos(theta5)*np.sin(theta2)*np.sin(theta3) - np.cos(theta1)*np.cos(theta2)*np.sin(theta3)*np.sin(theta5) - np.cos(theta1)*np.cos(theta3)*np.sin(theta2)*np.sin(theta5) - np.cos(theta1)*np.cos(theta2)*np.cos(theta3)*np.cos(theta4)*np.cos(theta5) - np.cos(theta5)*np.sin(theta1)*np.sin(theta4)
	J16 = 0
		
	J21 = np.cos(theta1)*np.cos(theta2) - np.sin(theta5)*(np.sin(theta1)*np.sin(theta4) + np.cos(theta1)*np.cos(theta2)*np.cos(theta3)*np.cos(theta4) - np.cos(theta1)*np.cos(theta4)*np.sin(theta2)*np.sin(theta3)) + np.cos(theta1)*np.cos(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta3)*np.sin(theta2) + np.sin(theta2 + theta3)*np.cos(theta1)*np.cos(theta5)
	J22 = np.sin(theta1)*(np.cos(theta2 + theta3) - np.sin(theta2) + (np.sin(theta2 + theta3)*np.sin(theta4 + theta5))/2 + np.cos(theta2 + theta3)*np.cos(theta5) - (np.sin(theta2 + theta3)*np.sin(theta4 - theta5))/2)
	J23 = np.sin(theta1)*(np.cos(theta2 + theta3) + np.cos(theta2 + theta3)*np.cos(theta5) + np.sin(theta2 + theta3)*np.cos(theta4)*np.sin(theta5))
	J24 = np.sin(theta5)*(np.cos(theta1)*np.cos(theta4) + np.cos(theta2)*np.cos(theta3)*np.sin(theta1)*np.sin(theta4) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3)*np.sin(theta4))
	J25 = np.cos(theta1)*np.cos(theta5)*np.sin(theta4) - np.cos(theta2)*np.sin(theta1)*np.sin(theta3)*np.sin(theta5) - np.cos(theta3)*np.sin(theta1)*np.sin(theta2)*np.sin(theta5) - np.cos(theta2)*np.cos(theta3)*np.cos(theta4)*np.cos(theta5)*np.sin(theta1) + np.cos(theta4)*np.cos(theta5)*np.sin(theta1)*np.sin(theta2)*np.sin(theta3)
	J26 = 0

	J31 = 0
	J32 = np.cos(theta2) + np.cos(theta2)*np.sin(theta3) + np.cos(theta3)*np.sin(theta2) + np.cos(theta2)*np.cos(theta5)*np.sin(theta3) + np.cos(theta3)*np.cos(theta5)*np.sin(theta2) - np.cos(theta2)*np.cos(theta3)*np.cos(theta4)*np.sin(theta5) + np.cos(theta4)*np.sin(theta2)*np.sin(theta3)*np.sin(theta5)
	J33 = np.cos(theta2)*np.sin(theta3) + np.cos(theta3)*np.sin(theta2) + np.cos(theta2)*np.cos(theta5)*np.sin(theta3) + np.cos(theta3)*np.cos(theta5)*np.sin(theta2) - np.cos(theta2)*np.cos(theta3)*np.cos(theta4)*np.sin(theta5) + np.cos(theta4)*np.sin(theta2)*np.sin(theta3)*np.sin(theta5)
	J34 = np.sin(theta2 + theta3)*np.sin(theta4)*np.sin(theta5)
	J35 = np.cos(theta2)*np.cos(theta3)*np.sin(theta5) - np.sin(theta2)*np.sin(theta3)*np.sin(theta5) - np.cos(theta2)*np.cos(theta4)*np.cos(theta5)*np.sin(theta3) - np.cos(theta3)*np.cos(theta4)*np.cos(theta5)*np.sin(theta2)
	J36 = 0

	J41 = 0
	J42 = np.sin(theta1)
	J43 = np.sin(theta1)
	J44 = np.sin(theta2 + theta3)*np.cos(theta1)
	J45 = -np.sin(theta4)*(np.cos(theta1)*np.sin(theta2)*np.sin(theta3) - np.cos(theta1)*np.cos(theta2)*np.cos(theta3)) - np.cos(theta4)*np.sin(theta1)
	J46 = np.sin(theta5)*(np.cos(theta4)*(np.cos(theta1)*np.sin(theta2)*np.sin(theta3) - np.cos(theta1)*np.cos(theta2)*np.cos(theta3)) - np.sin(theta1)*np.sin(theta4)) + np.cos(theta5)*(np.cos(theta1)*np.cos(theta2)*np.sin(theta3) + np.cos(theta1)*np.cos(theta3)*np.sin(theta2))

	J51 = 0
	J52 = -np.cos(theta1)
	J53 = -np.cos(theta1)
	J54 = np.sin(theta2 + theta3)*np.sin(theta1)
	J55 = np.cos(theta1)*np.cos(theta4) + np.sin(theta4)*(np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3))
	J56 = np.cos(theta5)*(np.cos(theta2)*np.sin(theta1)*np.sin(theta3) + np.cos(theta3)*np.sin(theta1)*np.sin(theta2)) - np.sin(theta5)*(np.cos(theta4)*(np.cos(theta2)*np.cos(theta3)*np.sin(theta1) - np.sin(theta1)*np.sin(theta2)*np.sin(theta3)) - np.cos(theta1)*np.sin(theta4))

	J61 = 1
	J62 = 0
	J63 = 0
	J64 = -np.cos(theta2 + theta3)
	J65 = np.sin(theta2 + theta3)*np.sin(theta4)
	J66 = -np.cos(theta2 + theta3)*np.cos(theta5) - np.sin(theta2 + theta3)*np.cos(theta4)*np.sin(theta5)
    
	J = np.array([[J11, J12, J13, J14, J15, J16],
					[J21, J22, J23, J24, J25, J26],
					[J31, J32, J33, J34, J35, J36],
					[J41, J42, J43, J44, J45, J46],
					[J51, J52, J53, J54, J55, J56],
					[J61, J62, J63, J64, J65, J66]])
	return J

def Ta(roll, pitch):
    T = np.array([[0, -np.sin(roll), np.cos(roll)*np.sin(pitch)],
                  [0, np.cos(roll), np.sin(roll)*np.sin(pitch)],
                  [1, 0, np.cos(pitch)]])
    Ta = np.block([[np.eye(3), np.zeros((3,3))],
                   [np.zeros((3,3)), T]])
    return Ta
    

if __name__ == '__main__':
    main()