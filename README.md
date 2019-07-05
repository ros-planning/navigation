MEL (Michael's Epic Localisation)
====================

Incorporates fusion of GPS and lidar data by modifying the AMCL package from the ROS Navigation Stack and running all data in an Extended Kalman Filter from the robot_localisation package.


TODO: Finish adding GPS.

TODO: Remove the need for the extra EKF by adding a GPS sensor model to the AMCL node and fusing there.

TODO: Adaptive weightings between lidar and GPS information sources based on, among other things, the scan match at the given pose. (should take care of lag in GPS error covariance updates and for indoor/outdoor nav)
