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
 
### Project results
A EKF is a two-step estimation problem: (1) a state prediction, in which we predict the state of the object of interest (the bicycle in our case) until the new measurement arrives and (2) the measurement update, in which new observations correct our belief about the state of the object of interest. Recall that for this project we fuse the data from Laser and Radar data, being the first a Cartesian system and the latter a Polar one. 

To test the presented implementation, two different environments are provided:
1.	`sample-laser-radar-measurement-data-1.txt` in which an eight-alike shape has to be correctly predicted, and
2.	`sample-laser-radar-measurement-data-2.txt` in which a skewed sinusoid-alike shape has to be correctly predicted.
 
These environments are available in the `data` folder.

#### First experiment
The first experiment is launched using the following command (again, assuming that we are in the build folder):
`ExtendedKF.exe ../data/sample-laser-radar-measurement-data-1.txt output1.txt`

The RMSE obtained is: **[0.0651649, 0.0605378,  0.54319, 0.544191]** and the results are shown in Figure 1 (taken using the kindly provided SF utilities).
![Figure 1: results using EKF in the first environment](andreuSancho/P6-Extended-Kalman-Filters/images/output1.png)

