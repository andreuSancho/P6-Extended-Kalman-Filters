# P6-Extended-Kalman-Filters
Sixth assignment of Udacity's Self-Driving Car Engineer Nanodegree Program

---

## Abstract
Accurate tracking is a critical component of self-driving cars as it is the key safety component in automation: detecting and tracking 
the position and velocity of pedestrians and other vehicles while considering the associated uncertainties is a must have for any 
autonomous device. The purpose of this project is to design and develop an Extended Kalman Filter (EKF) that is capable of fusing Laser 
and Radar measurements to accurately track a bicycle that travels around our simulated vehicle. As most of the hardware available for 
vehicles are small, embedded systems, the EKF has been implemented in C++.

## Rubric points
### Code organization
The code is organized following the standard C++ Object Oriented Programing paradigm (OOP). There are the following classes:
* **Tools**. This class allows (1) the computation of the error of our predictions by means of the Root Mean Squared Error (RMSE), (2) the calculation of the Jacobian matrix for the EKF, and (3) it also incorporated some useful constants shared through the code.
* **MeasurementPackage**. Another utility class. It encapsulates the raw sensor measurements.
* **GroundTruthPackage**. Similar to the MeasurementPackage class, it stores the ground truth values of the measures used in the EKF.
* **KalmanFilter**. It is the class that abstracts the EKF. This class initializes and deploys the filter, allowing its use.
* **FusionEKF**. This class fuses the data from Laser and Radar measurements in order to launch the predictions with the KalmanFilter class.


