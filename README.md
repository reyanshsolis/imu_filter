# gyro_filter
# DATA BIAS CORRECTION AND NOISE REMOVAL :
Applied for Data from IMU_Sensors mounted on wheels for ODOMETRY
Link for Bag Files (RAW DATA from Gyroscope) : 

## 1.	Kalman Filter based Bias Correction for Gyroscope
		
		This contains the code for using Linear Kalman Filter for 
		estimating the bias present in gyroscope data. Here we take 
		the x, y and z axis data from the gyroscope when the bot is at 
		rest. The data is used in combination with linear kalman 
		filter to estimate the initial bias in the data and then this 
		bias is subtracted from the incoming data to obtain the 
		corrected output.

[Link](https://www.researchgate.net/publication/299584644_Kalman_Filter_based_estimation_of_constant_angular_rate_bias_for_MEMS_Gyroscope) for the paper


## 2.	Noise Reduction of MEMS Gyroscope Based on Direct Modeling for an Angular Rate Signal
		
		This contains the code for using a novel approach for 
		processing the outputs signal of the microelectromechanical 
		systems (MEMS) gyroscopes to reduce the bias drift and noise. 
		The principle for the noise reduction was presented, and an 
		optimal Kalman filter (KF) was designed by a steady-state 
		filter gain obtained from the analysis of
		KF observability. In particular, the true angular rate signal 
		was directly modeled to obtain an optimal estimate and make a 
		self-compensation for the gyroscope without needing other 
		sensor’s information, whether in static or dynamic condition.

[Link](https://drive.google.com/file/d/1bqYdvCc0q_q_8K4BJrSQlS8Jh0HRDFFp/view?usp=sharing) for the paper



## 3.	Using Accelerometer and Gyroscope data and using Kalman Filter	for Odometry using a Wheel Mounted Inertial Sensor
		
[Link](https://drive.google.com/file/d/1ByBOscddfkhi3RtzwNHf2j-36tSprxVQ/view?usp=sharing) for the paper 

		(Yet to be implemented)

________________________________________________________________
