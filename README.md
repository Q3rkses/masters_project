# Trajectory reconstruction and sensor fusion for underwater robotics

A place to store code that will be used in my masters pre-project. This Repo will likely see many revisions and changes, and nothing here is permanent.

## Plans

#### Prototype & play around to figure stuff out stage

1. Make simple testcases, like gaussian random walk, gaussian white noise, CV model, CT model.
2. Make standard KF and filter the trajectory forward
3. Make a standard forward-backward smoother and RTS smoother and use it on the path
4. Plot the results, remember to evaluate filter consistency, smoother consistency.

#### Sophistication phase

1. Extend the current filter and smoother to be able to handle nonlinearities.
   1a. Add and experiment with EKF and ERTSS
   1b. Add and experiment with UKF and URTSS
   (stretch goal) add and experiment with ESKF and ESRTSS

#### Towards realistic scenarios

1. Make test cases more realistic and aligned with the goal of the project
   1a. Implement dynamical AUV model from Fossen
   1b. Implement models for sensors which will be used
   (stretch goal) test using the Stonefish simulator, hereunder find a good way to get GNSS and ground truth data to play with
2. Adopt the existing codebase to accomodate for the new AUV model and Sensor models
3. Run experiments and document consistency, accuracy and other factors that might be of interest
