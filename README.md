# Self-Driving Cars Specialization

[Self-Driving Cars Specialization](https://www.coursera.org/specializations/self-driving-cars) Repository.

### What you'll learn

- Understand the detailed architecture and components of a self-driving car software stack
- Implement methods for static and dynamic object detection, localization and mapping, behaviour and maneuver planning, and vehicle control
- Use realistic vehicle physics, complete sensor suite: camera, LIDAR, GPS/INS, wheel odometry, depth map semantic segmentation, object bounding boxes
- Demonstrate skills in CARLA and build programs with Python


## Specialization - 4 course series:
Be at the forefront of the autonomous driving industry. With market researchers predicting a $42-billion market and more than 20 million self-driving cars on the road by 2025, the next big job boom is right around the corner.

This Specialization gives you a comprehensive understanding of state-of-the-art engineering practices used in the self-driving car industry. You'll get to interact with real data sets from an autonomous vehicle (AV)―all through hands-on projects using the open source simulator CARLA.

Throughout your courses, you’ll hear from industry experts who work at companies like Oxbotica and Zoox as they share insights about autonomous technology and how that is powering job growth within the field.

You’ll learn from a highly realistic driving environment that features 3D pedestrian modelling and environmental conditions. When you complete the Specialization successfully, you’ll be able to build your own self-driving software stack and be ready to apply for jobs in the autonomous vehicle industry.

It is recommended that you have some background in linear algebra, probability, statistics, calculus, physics, control theory, and Python programming. You will need these specifications in order to effectively run the CARLA simulator: Windows 7 64-bit (or later) or Ubuntu 16.04 (or later), Quad-core Intel or AMD processor (2.5 GHz or faster), NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series card or higher, 8 GB RAM, and OpenGL 3 or greater (for Linux computers).

### Applied Learning Project

You’ll learn from a highly realistic driving environment that features 3D pedestrian modelling and environmental conditions. When you complete the Specialization successfully, you’ll be able to build your own self-driving software stack and be ready to apply for jobs in the autonomous vehicle industry.
## Applied Learning Project:
By the end of this Specialization, you will be ready to:

• Build machine learning models in Python using popular machine learning libraries NumPy and scikit-learn.

• Build and train supervised machine learning models for prediction and binary classification tasks, including linear regression and logistic regression.

• Build and train a neural network with TensorFlow to perform multi-class classification.

• Apply best practices for machine learning development so that your models generalize to data and tasks in the real world.

• Build and use decision trees and tree ensemble methods, including random forests and boosted trees.

• Use unsupervised learning techniques for unsupervised learning: including clustering and anomaly detection.

• Build recommender systems with a collaborative filtering approach and a content-based deep learning method.

• Build a deep reinforcement learning model.

### There are 4 Courses in this Specialization:
<details>
  <summary>Introduction to Self-Driving Cars</summary>

  <h6 align="left">
Welcome to Introduction to Self-Driving Cars, the first course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the terminology, design considerations and safety assessment of self-driving cars.  By the end of this course, you will be able to:
- Understand commonly used hardware used for self-driving cars
- Identify the main components of the self-driving software stack
- Program vehicle modelling and control
- Analyze the safety frameworks and current industry practices for vehicle development

For the final project in this course, you will develop control code to navigate a self-driving car around a racetrack in the CARLA simulation environment. You will construct longitudinal and lateral dynamic models for a vehicle and create controllers that regulate speed and path tracking performance using Python. You’ll test the limits of your control design and learn the challenges inherent in driving at the limit of vehicle performance.

This is an advanced course, intended for learners with a background in mechanical engineering, computer and electrical engineering, or robotics. To succeed in this course, you should have programming experience in Python 3.0, familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses), Statistics (Gaussian probability distributions), Calculus and Physics (forces, moments, inertia, Newton's Laws).

You will also need certain hardware and software specifications in order to effectively run the CARLA simulator: Windows 7 64-bit (or later) or Ubuntu 16.04 (or later), Quad-core Intel or AMD processor (2.5 GHz or faster), NVIDIA GeForce 470 GTX or AMD Radeon 6870 HD series card or higher, 8 GB RAM, and OpenGL 3 or greater (for Linux computers).
</h6>

</details>

<details>
  <summary>State Estimation and Localization for Self-Driving Cars</summary>

<h6 align="left">
Welcome to State Estimation and Localization for Self-Driving Cars, the second course in University of Toronto’s Self-Driving Cars Specialization. We recommend you take the first course in the Specialization prior to taking this course.

This course will introduce you to the different sensors and how we can use them for state estimation and localization in a self-driving car. By the end of this course, you will be able to:
- Understand the key methods for parameter and state estimation used for autonomous driving, such as the method of least-squares
- Develop a model for typical vehicle localization sensors, including GPS and IMUs
- Apply extended and unscented Kalman Filters to a vehicle state estimation problem
- Understand LIDAR scan matching and the Iterative Closest Point algorithm
- Apply these tools to fuse multiple sensor streams into a single state estimate for a self-driving car

For the final project in this course, you will implement the Error-State Extended Kalman Filter (ES-EKF) to localize a vehicle using data from the CARLA simulator.

This is an advanced course, intended for learners with a background in mechanical engineering, computer and electrical engineering, or robotics. To succeed in this course, you should have programming experience in Python 3.0, familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses), Statistics (Gaussian probability distributions), Calculus and Physics (forces, moments, inertia, Newton's Laws).
</h6>

</details>

<details>
  <summary>Visual Perception for Self-Driving Cars</summary>

<h6 align="left">
Welcome to Visual Perception for Self-Driving Cars, the third course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the main perception tasks in autonomous driving, static and dynamic object detection, and will survey common computer vision methods for robotic perception.  By the end of this course, you will be able to work with the pinhole camera model, perform intrinsic and extrinsic camera calibration, detect, describe and match image features and design your own convolutional neural networks.  You'll apply these methods to visual odometry, object detection and tracking, and semantic segmentation for drivable surface estimation. These techniques represent the main building blocks of the perception system for self-driving cars.

For the final project in this course, you will develop algorithms that identify bounding boxes for objects in the scene, and define the boundaries of the drivable surface.  You'll work with synthetic and real image data, and evaluate your performance on a realistic dataset.

This is an advanced course, intended for learners with a background in computer vision and deep learning. To succeed in this course, you should have programming experience in Python 3.0, and familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses).
</h6>
</details>

</details>

<details>
  <summary>Motion Planning for Self-Driving Cars</summary>

<h6 align="left">
Welcome to Motion Planning for Self-Driving Cars, the fourth course in University of Toronto’s Self-Driving Cars Specialization.

This course will introduce you to the main planning tasks in autonomous driving, including mission planning, behavior planning and local planning.   By the end of this course, you will be able to find the shortest path over a graph or road network using Dijkstra's and the A* algorithm, use finite state machines to select safe behaviors to execute, and design optimal, smooth paths and velocity profiles to navigate safely around obstacles while obeying traffic laws.  You'll also build occupancy grid maps of static elements in the environment and learn how to use them for efficient collision checking. This course will give you the ability to construct a full self-driving planning solution, to take you from home to work while behaving like a typical driving and keeping the vehicle safe at all times.

For the final project in this course, you will implement a hierarchical motion planner to navigate through a sequence of scenarios in the CARLA simulator, including avoiding a vehicle parked in your lane, following a lead vehicle and safely navigating an intersection.  You'll face real-world randomness and need to work to ensure your solution is robust to changes in the environment.

This is an intermediate course, intended for learners with some background in robotics, and it builds on the models and controllers devised in Course 1 of this specialization. To succeed in this course, you should have programming experience in Python 3.0, and familiarity with Linear Algebra (matrices, vectors, matrix multiplication, rank, Eigenvalues and vectors and inverses) and calculus (ordinary differential equations, integration).
</h6>
</details>






<h3 align="left">Languages and Tools:</h3>
<p align="left"> <a href="https://jupyter.org/" target="_blank" rel="noreferrer"> <img src="https://jupyter.org/assets/try/jupyter.png" alt="JupyterLab" width="40" height="40"/> </p>
