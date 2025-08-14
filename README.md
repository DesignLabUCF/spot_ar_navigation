# Evaluating the Effectiveness of Augmented Reality Interfaces for Quadrupedal Robot Motion Control

This repository is the official implementation of [**Evaluating the Effectiveness of Augmented Reality Interfaces for Quadrupedal Robot Motion Control**](). 

![Heder Imager for project.](images/header.png)

This study examines how Augmented Reality (AR) interfaces affect navigation control of the [**Boston Dynamics Spot**](https://bostondynamics.com/products/spot/) quadrupedal robot UCF [**TapeMeasure**](https://www.instagram.com/ucf.tapemeasure/). Testing with 33 non-experts and two experts showed that AR provided smoother robot movement and similar usability and trust compared to tablet controls, despite longer task times. The findings highlight AR’s potential to make advanced robotic systems more accessible and trusted by a wider range of users. This repository allows researchers to replicate the environment for our study and help aid their own studies that leverage quadrupedal robotic platforms.

## Tablet Training

![](images/TabletTraining2.PNG)

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


