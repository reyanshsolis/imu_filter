#include <iostream>
#include<fstream>
#include<stdlib.h>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


MatrixXd G(3,1);    //(3,1)
MatrixXd G_meas(3,1);   //3,1
MatrixXd b(3,1);   //3,1
MatrixXd H(1,3);   //1,3
MatrixXd R(3,3);
MatrixXd P(3,3);
MatrixXd sigma(3,3);
MatrixXd Z(3,1);   //3,1
MatrixXd Y(3,1);   //3,1
MatrixXd S(3,3);  	
MatrixXd K(3,3);
MatrixXd I(3,3);
MatrixXd Gnew(3,3);
MatrixXd Hnew(3,3);

struct data_imu{
	float angular_velocity_x;
	float angular_velocity_y;
	float angular_velocity_z;

};
data_imu gval;

void x(const std_msgs::Float64::ConstPtr& msg)
{
   ::gval.angular_velocity_x=msg->data;
   //ROS_INFO("I heard: [%f]", msg->data);

}
void y(const std_msgs::Float64::ConstPtr& msg)
{
   ::gval.angular_velocity_y=msg->data;
}
void z(const std_msgs::Float64::ConstPtr& msg)
{
   ::gval.angular_velocity_z=msg->data;
}

void kalman(data_imu data_gyro)
{
	float gx,gy,gz;
	int i,j;
  	gx = data_gyro.angular_velocity_x;//rand()%100;
	gy = data_gyro.angular_velocity_y;//rand()%100;
	gz = data_gyro.angular_velocity_z;//rand()%100;
	cout<<"gx :"<<gx<<"\n";
 	
  	//v and epsilon are to be added...


  	G <<gx, gy, gz;
	b<< 0,
		0,
		0;
	P<< 0.0282,0,0,
		0,0.0247,0,
		0,0,0.0328;
	R<< 0.0282,0,0,
		0,0.0247,0,
		0,0,0.0328;
	sigma<< 0.0282,0,0,
			0,0.0247,0,
			0,0,0.0328;
	I<< 1,0,0,
		0,1,0,
		0,0,1;		


	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Hnew(i,j)=H(0,j);
		}
	}	
	//Hnew<<1,1,1,1,1,1,1,1,1;	
	G_meas = G-b; // G = G_true + epsilon, where epsilon is the gyroscope noise			

	H = 2.0*(G_meas-b).transpose();
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			Gnew(i,j)=G_meas(i,0)-b(i,0);
		}
	}
	//Gnew<<0,0,0,0,0,0,0,0,0;

	R = 4.0*(((Gnew).cwiseProduct(sigma.cwiseProduct(Gnew.transpose()))) + 2.0*(sigma.cwiseProduct(sigma)));
	Z = G_meas.cwiseProduct(G_meas);  //np.linalg.det
	Y = (Z.array() - (H*b)(0,0)).matrix();
	S = Hnew.cwiseProduct(P.cwiseProduct(Hnew.transpose())) + R;
	K = P.cwiseProduct(Hnew.transpose().cwiseProduct(S.inverse()));
	b = b + K*Y; // try dot K.dot(Y)
	P = (I - K.cwiseProduct(Hnew)).cwiseProduct(P);
	cout<<"G_meas : "<<R<<"\n";


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "kalman");
	ros::NodeHandle n;
	ros::Subscriber sub1 = n.subscribe("/gx",1, x);
	ros::Subscriber sub2 = n.subscribe("/gy",1, y);
	ros::Subscriber sub3 = n.subscribe("/gz",1, z);
	kalman(gval);

	 
	// gval.angular_velocity_x=-0.099135;	
	// gval.angular_velocity_y=0.900865;	
	// gval.angular_velocity_z=1.900865;
	int i=1;
	while(i<100000)
	{

		// sub = n.subscribe("gx",1, x);
		// // sub = n.subscribe("gx",1, y);
		// // sub = n.subscribe("gx",1, z);
		kalman(gval);

		i++;
	}

	ros::spin();
	return 0;


}

