import rospy
from sensor_msgs.msg import Imu
from math import *
import numpy as np 
import matplotlib.pyplot as plt
import random
from numpy import linalg as LA
from geometry_msgs.msg import Twist 


def kalman(data):

  global K # filter gain
  global K_s # steady state gain
  global k1
  global k2

  global m
  global S
  global D
  global lam # eigen values - lamda 1, lamda 2
  global Slam1
  global Slam2
  global A 
  global B 
  global W # W- omega - input true angular rate
  global y # y- measured angular rate | Z = y
  global b # bias drift # b'=W_b
  global X 
  global F # KF coefficient matrix
  global P # estimated covariance

  global qn
  global qw
  global qb

  global Pk  #P(k)
  global Pk1 #P(k/k+1)

  global H # measurement matrix
  global Q # covariance matrix
  global R # covariance matrix R= qn - variance of white noise

  


#  if count < count_min:
#    return
#  
#  if count > count_max :  
#    g_correct = g - np.transpose(b)
#    plt.subplot(121)
#    plt.hist(g, bins='auto')
#    plt.title('Original data')
#    plt.subplot(122)
#    plt.hist(g_correct, bins='auto')
#    plt.title('Corrected data')
#    plt.suptitle('Kalman Correction')
#    plot.show()
    
  # gx = data.angular_velocity.x
  # gy = data.angular_velocity.y
  # gz = data.angular_velocity.z

  # rospy.loginfo("I heard %s", gx)
  Z=data.angular.z
  
  X= np.matmul(A,X)+np.multiply(np.matmul(B,K),Z)
  W= np.matmul(e1,X)
  b= np.matmul(e2,X)
  print(W)
  # g[count - count_min] = G 
  # count+=1

  # print("bias : [ ", b[0][0]," , ",b[1][0]," , ",b[2][0]," ]")
  return



def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/wsl/enc_left",Twist , kalman)

  rospy.spin()

if __name__ == '__main__':
  global K # filter gain
  global k1
  global k2

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

  global H # measurement matrix
  global Q # covariance matrix
  global R # covariance matrix R= qn - variance of white noise


  F = np.zeros((2,2))
  H = np.array([[1.,1.]], np.float64)
  e1 = np.array([[1.,0.]], np.float64)
  e2 = np.array([[0.,1.]], np.float64)
  qn = 0.1667
  qb = 1200
  qw = 1000
  T = 0.005
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

  Q= np.array([[qw,0],[0,qb]], np.float64)
  R= qn

  for i in range(0,1):
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
  Slam1= np.array([[np.exp(-lam2*T),0],[0,1]], np.float64)
  Slam2= np.array([[-(np.exp(-lam2*T)-1)/lam2,0],[0,T]], np.float64)

  A= np.matmul(np.matmul(S,Slam1),np.linalg.inv(S))
  B= np.matmul(np.matmul(S,Slam2),np.linalg.inv(S))
    print(K)
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
  listener()  
