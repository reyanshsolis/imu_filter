import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from math import *
import numpy as np 
import matplotlib.pyplot as plt
import random
from numpy import linalg as LA
from geometry_msgs.msg import Twist 
from std_msgs.msg import *
import csv

imu_msg = Imu()
imu_msg.header.frame_id = "/imu"
mag_msg = Vector3Stamped()
# def publish_imu(imu_msg):
  
    
  
#   imu_msg.angular_velocity.x = 0
#   imu_msg.angular_velocity.y = 0
#   imu_msg.angular_velocity.z = Z
#   imu_msg.linear_acceleration.x = 0
#   imu_msg.linear_acceleration.y = 0
#   imu_msg.linear_acceleration.z = 0
#   pub_imu.publish(imu_msg)


pub = rospy.Publisher('/imu/data_raw', Imu)
pub2 = rospy.Publisher('/imu/mag', Vector3Stamped)



  
def kalman(data):

  Z=data.data

  imu_msg.angular_velocity.x = 0
  imu_msg.angular_velocity.y = 0
  imu_msg.angular_velocity.z = Z
  imu_msg.linear_acceleration.x = 0
  imu_msg.linear_acceleration.y = 0
  imu_msg.linear_acceleration.z = 0

  mag_msg.vector.x=0
  mag_msg.vector.y=0
  mag_msg.vector.z=0
  pub.publish(imu_msg)
  pub2.publish(mag_msg)

def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/l_vel",Float32 , kalman)

  pub.publish((imu_msg))
  pub2.publish((mag_msg))
  rospy.spin()


# def talker():
#     rospy.init_node('talker', anonymous=True)
#     pub = rospy.Publisher("/w_cal", Float32, queue_size=1)
#     rate = rospy.Rate(100) # 10hz 
#     while not rospy.is_shutdown():
#       pub.publish(W)   
#       rate.sleep()
#     return

if __name__ == '__main__':
  global K # filter gain
  global k1
  global k2
  global Z_orig
  global W_cal
  global m
  global S
  global D
  global lam # eigen values - lamda 1, lamda 2
  global Slam1
  global Slam2
  global A 
  global B 
  global W # W- omega - input true angular rate
  global Z # y- measured angular rate | Z = y
  global b # bias drift # b'=W_b
  global X 
  global F # KF coefficient matrix
  global P # estimated covariance

  global qn # ARW noise
  global qw # white noise
  global qb # RRW noise

  global Pk  #P(k)
  global Pk1 #P(k/k+1)
  global count
  global H # measurement matrix
  global Q # covariance matrix
  global R # covariance matrix R= qn - variance of white noise

  count=0
  F = np.array([[1.,0.],[0.,0.]], np.float64)
  H = np.array([[1.,1.]], np.float64)
  e1 = np.array([[1.,0.]], np.float64)
  e2 = np.array([[0.,1.]], np.float64)
  qn = 0.1667
  qb = 1200
  qw = 1000
  T = 0.01
  I = np.array([[1.,0.],[0.,1.]], np.float64)
  K = np.array([[1.],[1.]], np.float64)
  P = np.zeros((2,2))
  Pk1 = np.zeros((2,2))
  m = np.zeros((2,2))
  A = np.zeros((2,2))
  B = np.zeros((2,2))
  X = np.zeros((2,1))
  Z = 0
  W = 0
  b = 0
  W_cal=np.zeros((20000,1))
  Z_orig=np.zeros((20000,1))

  Q= np.array([[qw,0],[0,qb]], np.float64)
  R= qn

  for i in range(0,500):
  	Pk1= np.matmul(np.matmul(F,P),np.transpose(F))+Q
  	K= np.matmul(np.matmul(Pk1,np.transpose(H)),np.linalg.inv(np.matmul(np.matmul(H,Pk1),np.transpose(H))+R))
  	P= np.matmul(np.matmul((I-np.matmul(K,H)),Pk1),np.transpose(I-np.matmul(K,H)))+ np.matmul(np.multiply(K,R),np.transpose(K)) 
  	k1=K.item(0)
	k2=K.item(1)

	m=np.array([[k1,k1],[k2,k2]], np.float64)
	lam,S=LA.eig(m)
	D=np.diag(lam)
	lam1=lam.item(0)
	lam2=lam.item(1)
	print(lam)
	Slam1= np.array([[np.exp(-lam1*T),0],[0,1]], np.float64)
	Slam2= np.array([[-(np.exp(-lam1*T)-1)/lam1,0],[0,T]], np.float64)

	A= np.matmul(np.matmul(S,Slam1),np.linalg.inv(S))
	B= np.matmul(np.matmul(S,Slam2),np.linalg.inv(S))
  	print(K)


    # writer.writerow({'emp_name': 'Erica Meyers', 'dept': 'IT', 'birth_month': 'March'})
    # with gyrofile:
  	# 	writer=csv.writer(gyrofile)
  	# 	writer.writerows(W_cal) 	
#  hz = 2
#  time_start = 25
#  time_end = 45 #seconds
#  count_min = hz * time_start
#  count_max = hz * time_end
#  size = count_max - count_min  
#  g = np.empty([size,3], dtype = np.float64)
#  g_correct = np.empty([size,3], dtype = np.float64)

#  b = np.array([[0],[0],[0]], np.float64) #matrix([[0.],[0,],[0.]])
#  P = R = sigma = np.array([[0.0282,0,0],[0,0.0247,0],[0,0,0.0328]], np.float64) #matrix([[0.0282,0.,0.],[0.,0.0247,0.],[0.,0.,0.0328]])
#  I = np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], np.float64) #matrix([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])

# count = 0
  try:	
  	listener() 
  except rospy.ROSInterruptException:
    pass 
