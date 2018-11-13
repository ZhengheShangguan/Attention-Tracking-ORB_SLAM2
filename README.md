# Attention-Tracking-ORB_SLAM2
**Authors: ORB SLAM 2** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Authors: [Attention and Anticipation in Fast Visual-Inertial Navigation]** [Luca Carlone and Sertac Karaman](https://arxiv.org/pdf/1610.03344.pdf)

**Authors: Attention-Tracking ORB_SLAM2** Zhenghe Shangguan


# 1. Main Contribution
- The authors of the above "Attention" paper have not opened their source codes yet, and this work serves as a reproduction and extension of their paper.
- Adapted Attention Method from VIO Tracking thread to Visual SLAM Tracking threads
- Derived the method with Lie Algebra, Implemented the new method on ORB-SLAM2


# 2. Methodology
Remodel the attention algorithm for a VO/Visual SLAM. This is based on two main reasons:
1. the authors did not open source their codes of the paper;
2. the attention model is for Visual Inertial Odometry, whereas my system and data is collected from my Visual SLAM. 

- I took some time to understand all the theoretical parts, then gave the adapted analytical model for my data based on the previous attention algorithm.

- Briefly demonstrated as follow:
According to the paper, it tries to set up a good feature selection method based on each feature’s contribution to the final system’s information matrix, and the best set of features is the one which makes the smallest eigenvalue of the information matrix largest.

The key thing here is to calculate the final information matrix based on each feature’s contribution to the information, and for a single feature, its information contribution can be propagated under two periods. The first period is for IMU which I don’t have, thus this can be eliminated. What I did is to remodel the second period based on the condition:

![first_equation](https://latex.codecogs.com/gif.latex?u_%7Bkl%7D%20%5Ctimes%20%28R_%7Bk%7Dp_%7Bl%7D%20&plus;%20t_%7Bk%7D%29%3D0_%7B3%7D)

where: u_kl is the feature’s image coordinate in camera frame, R_{cam,k}^{w}, t_{cam,k}^{w}  are the rotation matrix and translation matrix at time-step k, p_{l} is the feature’s 3D coordinate in world frame.

The physical meaning for this is the vector from camera center to the image plane’s feature is collinear to the vector from camera center to the 3D point in real world. For ORB-SLAM system, the state should be changed from kinematic state (i.e.: t, v, b in paper) to the prediction difference of robot’s pose like follow:

![second_equation](https://latex.codecogs.com/gif.latex?x_%7Bk%7D%20%3D%20%5Cleft%5B%5Cvarphi%3Bt_%7Bcam%2Ck%7D%5E%7Bw%7D%5Cright%5D)

where: ![third_equation](https://latex.codecogs.com/gif.latex?R_%7Bcam%2Ck%7D%5E%7Bw%7D%20%3D%20exp%28%7B%5Cvarphi%7D%5E%7B%5Cland%7D%29exp%28%7B%5Cphi%7D%5E%7B%5Cland%7D%29), denote ![fourth_equation](https://latex.codecogs.com/gif.latex?t%20%3D%20t_%7Bcam%2Ck%7D%5E%7Bw%7D%2C%20R%20%3D%20R_%7Bcam%2Ck%7D%5E%7Bw%7D%2C%20%5Chat%7BR%7D%20%3D%20exp%28%7B%5Cphi%7D%5E%7B%5Cland%7D%29) are the prediction difference of robot’s pose by tracking with constant motion model.

Now denote the left side of the equation as $f(\varphi,t)$ (we choose this because the left perturbation model of derivative of $\varphi$ is more straightforward than the derivative of $R$ itself.), and linearize it with total differentiation formula:

![fifth_equation](https://latex.codecogs.com/gif.latex?f%28%5Cvarphi%2Ct%29%20%3D%20f%28%5Chat%7B%5Cvarphi%7D%2C%5Chat%7Bt%7D%29%20&plus;%20%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20%5Cvarphi%7D%7C_%7B%28%5Chat%7B%5Cvarphi%7D%2C%5Chat%7Bt%7D%29%7D%28%5Cvarphi-%5Chat%7B%5Cvarphi%7D%29%20&plus;%20%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20t%7D%7C_%7B%28%5Chat%7B%5Cvarphi%7D%2C%5Chat%7Bt%7D%29%7D%28t-%5Chat%7Bt%7D%29)

Some calculations:
- ![sixth_equation](https://latex.codecogs.com/gif.latex?%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20t%7D%7C_%7B%28%5Chat%7B%5Cvarphi%7D%2C%5Chat%7Bt%7D%29%7D%20%3D%20%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20t%7D%7C_%7B%280%2C%5Chat%7Bt%7D%29%7D%20%3D%20-%7BU%7D_%7Bkl%7D)
- ![seventh_equation](https://latex.codecogs.com/gif.latex?%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20%5Cvarphi%7D%7C_%7B%28%5Chat%7B%5Cvarphi%7D%2C%5Chat%7Bt%7D%29%7D%20%3D%20%5Cfrac%7B%5Cpartial%20f%7D%7B%5Cpartial%20%5Cvarphi%7D%7C_%7B%280%2C%5Chat%7Bt%7D%29%7D%3D%7BU%7D_%7Bkl%7D%28%5Chat%7BR%7D%7Bp%7D%29%5E%7B%7B%5Cland%7D%7D)

Thus, the contribution of l^th feature during H-time horizon can be calculated as:
![eighth_equation](https://latex.codecogs.com/gif.latex?A%20%3D%20%5Cleft%5B%5Cbegin%7Barray%7D%7Bccc%7D%20U_%7Bkl%7D%20%28%5Chat%7B%7BR%7D_%7Bk%7D%7D%20p_%7Bl%7D%29%5E%7B%5Cland%7D%2C%20-U_%7Bkl%7D%20%5C%5C%20...%5C%5C%20U_%7B%28k&plus;H%29l%7D%20%28%7B%5Chat%7BR%7D%7D_%7Bk&plus;H%7D%20p_%7Bl%7D%29%5E%7B%5Cland%7D%2C%20-U_%7B%28k&plus;H%29l%7D%20%5Cend%7Barray%7D%20%5Cright%5D)

![ninth_equation](https://latex.codecogs.com/gif.latex?%5Ctriangle_%7Bl%7D%20%3D%20A%5E%7BT%7DA)

In my codes, I use H = 1, but the users can easily change it to be a multi-frame time horizon.

# 3. Prerequisites
I have tested the library in **14.04** with ROS indigo. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
I use the new thread and chrono functionalities of C++11.

## Pangolin
I use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
I use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
I use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS 
ROS indigo is required [ros](http://wiki.ros.org/indigo/Installation/Ubuntu).


# 4. Building Active-ORB-SLAM2 library

Clone the repository:
```
git clone https://github.com/ZhengheShangguan/Attention-Tracking-ORB_SLAM2.git
```

First of all: If you set your default ROS PATH as /../../../ORB_SLAM2, you should change file name to be ORB_SLAM2  before you build.

```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 5. Building ROS

First of all: If you set your default ROS PATH as /../../../ORB_SLAM2, you should change file name to be ORB_SLAM2  before you build.

```
chmod +x build_ros.sh
./build_ros.sh
```

# 6. Run the Attention-Tracking VO of ORB-SLAM2
```
The same as the original ORB-SLAM2 system, please refer to https://github.com/raulmur/ORB_SLAM2
