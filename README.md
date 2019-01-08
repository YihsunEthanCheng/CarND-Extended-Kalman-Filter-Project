# Extended Kalman Filter

[//]: # (Image References)

[image0]: ./Docs/kalman_filter_cycles.png "Kalman filter cycle"
[image1]: ./Docs/kalman_filter_formula.png "Kalman Filter formula"
[image2]: ./Docs/sensor_fusion.png "flow of sensor fusion"
[image3]: ./Docs/simulation_result_1.png "simulation result-1"


## Objecitives

To apply Extended Kalman Filter theory on tracking moving objects. In particular, achieve superior tracking accruacy by fusing measurements from independent sensors.

---

## How to run this project

To run this project, the following three modules need to be installed on the same system.
1. Simulator <BR>
The simulator is owned by Udacity and can be downloded [here](https://github.com/udacity/self-driving-car-sim/releases). The simulator acts as a sensor hub and spits out synthesis data simulating Lidar and Radar sensor readings through a ethernet socket. Once the simulator is downloaded/installed/launched, the sythesized sensor data are sent over the ports for grabbing in the sensing frequency similar to the real sensors.
2. uWebSocketIO <BR>
uWebSocketIO is a communication library which implements the data transimitting/receiving through a socket on the system.  This library offers C/C++ interface for socket programing to communicate with any other device through Ethernet port. This library can be downloaded [here](https://github.com/uNetworking/uWebSockets) but can only be installed in Linux-like systems.
3. Extended Kalman Filters <BR>
This module is coded in C++ and is stored the [src](./src) folder. The executable binary is available at the [build](./build) folder and can be launched by the command below. Once launched, the Kalman filter is waiting for Lidar/Radar data from the simulator to perform tracking of a moving object. The updated states are send to trhe simulator for display.

     ```sh
      ./build/ExtendedKF
     ```

---

## Kalman Filter in a Nutshell


A Kalman filter estimates hidden state variables of a dynamic system. The hidden state variables can predict the next observable state varaibles by using the optimal gain, namely the Kalman gain, estimated from the latest prediction error. The active gain filter is known to be optimal and superior to well known fixed-gain filters like alph, or alph-beta filters. Operation of a Kalman filter can be described in the following two stages.

![alt text][image0]

1. The Prediction stage<BR>
  The prediction stage is a state transition of a dynamic system which can be described as a linear system with a transition matrix as.  

    ```sh
      X(k+1) = F * X(k)
    ```
   The state transition also implies a uncertinty of the next state which can be described as a covariance matrix, often named Q. The uncertainty Q provides a mean of computing the active gain in the second stage to achieve better prediction.

2. The updating stage <BR>
  In the updating stage, the prediction error, called residual, is first computed. This error is then used to combined with the prediction uncertainty and process noise to comute the Kalman gain. This Kalman gain is a well established by optimization theory, thus the details are skipped. <BR>

## Kalman Filter formulae

![alt text][image1]

---

## Sensor Fusion Through Kalman Filter

1. How fusion are achieved <BR> 
  Sensor fusion in Kalman Filter is achieved through individual updating steps by multiple sensors of the same purpose. Lidar and Radar sensors are perfect examples for sensing moving objects. Both are intented to sense the location of a object. As they sense the object in different mechanism, the two are known to be independent agents. Thus, a perfect pair of candidates for sensor fusion through Bayseain theory.  

2. Only affecting the updating stage <BR>
  Sensors reading are used to tune the parameters used for predictions at a particular time stamp. The higher prediction error leads to more aggresive updating gains. As the prediction stage only involves state transition, the sensor fusion is independet from the prediction stage.

3. Linearization as a mean to non-symetric sensing noise model <BR>
  Kalman filter uses Bayesian theroy for prediction which does not work if the sensing noise is not symmetric.  Take Radar as an example, the sensor noise in its domain is symmetric. But the symmetric property is lost once the data in converted to the state variable domain. As sensor readings are samples of continuous signal, local linearity is often preserved which enables Kalman filter to work within a local linearized model. As the extraction matrix H constantly being updated to reflect the current operating point, Kalman filter still works in non-symmetric sensor noise, thus the name of **_Extended Kalman Filter_**. The figure blow summarizes the flow of sensor fusion in Extended Kalman Filter.

![alt text][image2]


---
## Simulation Results

* Tracked states of the vehicle drawn in green dots.
* Red dots are locations measurements by Laser.
* Blue dots are locations measurements from Radar.


![alt text][image3]

---
## Takeaways

* Bayesian Theorem - the backbone of fusion independent statistics

* Importance of Initial conditions

