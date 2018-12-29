# Overview
My solutions to quizzes in Artificial Intelligence for Robotics Udacity Online Courses.

Concepts:
- Localization
  - Histogram Filter
  - Kalman Filter
  - Particle Filter
- Path Planning
  - A*
  - D*
  - Constrained Smoothing
- Control
  - PID Control
  - Twiddle
- SLAM
  - online SLAM
  
Final Exam:
-  Runaway Robot Recoverer

This is a program which the chaser must intercept the target. The target is moving in an almost-circle pattern. The chaser is slower than the target. The circular motion of the target is programmed using PID motion controller. The location of the target and chaser is predicted using extended Kalman filter and particle filter respectively. The chaser path is planned using A* algorithm.

[![Runaway Robot Recoverer](http://img.youtube.com/vi/WnfZBd7JljU/0.jpg)](https://www.youtube.com/watch?v=WnfZBd7JljU "Runaway Robot Recoverer")
