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

### Build instructions
The herein provided code has been developed in a Windows 10 platform using [MinGW.](http://www.mingw.org/) The steps for compiling the code are the following, assuming the repository is already cloned and that you are in it with the console (CMD):
1. Create the directory 'build' and enter on it.
2. Make and build the code: `cmake .. -G "MinGW Makefiles" && make`
If correctly done, the file `ExtendedKF.exe` will be placed on the build directory.

### Project dependencies
The dependencies are the following (assuming a Windows OS):
* cmake >= 3.5
  * [Click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
   * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Windows: [MinGW](http://www.mingw.org/)
 
