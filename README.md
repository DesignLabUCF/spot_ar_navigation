# Evaluating the Effectiveness of Augmented Reality Interfaces for Quadrupedal Robot Motion Control

This repository is the official implementation of [**Evaluating the Effectiveness of Augmented Reality Interfaces for Quadrupedal Robot Motion Control**](). 

![Heder Imager for project.](images/header.png)

This study examines how Augmented Reality (AR) interfaces affect navigation control of the [**Boston Dynamics Spot**](https://bostondynamics.com/products/spot/) quadrupedal robot UCF [**TapeMeasure**](https://www.instagram.com/ucf.tapemeasure/). Testing with 33 non-experts and two experts showed that AR provided smoother robot movement and similar usability and trust compared to tablet controls, despite longer task times. The findings highlight AR’s potential to make advanced robotic systems more accessible and trusted by a wider range of users. This repository allows researchers to replicate the environment for our study and help aid their own studies that leverage quadrupedal robotic platforms.



## HoloLens 2 Training
![hololens training view](images/20240411_145648_HoloLens.jpg)


The `Spot-AR-main` folder contains the Unity projects used for the AR control study, including the **participant HoloLens 2 training module**. In this module, participants see three virtual blue balls in the AR environment. Using a **ray cast pointer**, they must hover over each ball and perform a **pinch gesture** to make it disappear. This exercise allows participants to practice the core interaction mechanics—target selection and gesture input—before moving on to the main navigation tasks. The training also provides an opportunity to get comfortable with controlling Spot’s movement in AR, ensuring participants are ready for the full study.

| ![Ball 1](images/20240411_145622_HoloLens.jpg) | ![Ball 2](images/20240411_145631_HoloLens.jpg) | ![Ball 3](images/20240411_145639_HoloLens.jpg) |
|-----------------------------|-----------------------------|-----------------------------|



## Tablet Training

![tablet training view](images/TabletTraining2.PNG)

The `Spot-TabletTraining` folder contains a Unity project designed to help users understand and practice with the two major joystick controls for Spot:

- **Left joystick** – Controls Spot’s rotation (turning left or right).
- **Right joystick** – Controls Spot’s movement (forward, backward, and sideways).

#### Training Objective

The goal of the training is to **navigate to the four blue boxes displayed on screen** as quickly and precisely as possible. This exercise develops both accuracy and speed in operating Spot’s controls.

#### Project Details

- **Unity Version:** 2021.3.5  (*Works with upgrading to ver 6+*)
- **Main Scene:** `SampleScene.unity`  

To launch the training, open: `SampleScene.unity` and start the scene. This project is desigend for tablet touch screen controls, but the left and right mouse buttons work to simulate the robot controls.

| ![Joystick Left](images/tablet5.png) | ![Joystick Right](images/tablet4.png) |
|:------------------------------------------:|:---------------------------------------------:|
| Left Joystick                              | Right Joystick                               |
| ![Spot Navigation](images/tablet2.png) | ![Blue Boxes](images/tablet3.png) |
| Navigation View                            | Target Blue Boxes                            |


