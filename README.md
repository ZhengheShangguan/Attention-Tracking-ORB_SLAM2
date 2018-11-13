# Attention-Tracking_ORB-SLAM2
**Authors: ORB SLAM 2** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Authors: [Attention and Anticipation in Fast Visual-Inertial Navigation]** [Luca Carlone and Sertac Karaman](https://arxiv.org/pdf/1610.03344.pdf)

**Authors: Attention-Tracking ORB-SLAM2** Zhenghe Shangguan


# 1. Main Contribution
- Adapted Attention Method from VIO Tracking thread to Visual SLAM Tracking threads
- Derived the method with Lie Algebra, Implemented the new method on ORB-SLAM2


# 2. Prerequisites
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


# 3. Building Active-ORB-SLAM2 library

Clone the repository:
```
git clone https://github.com/ZhengheShangguan/Attention-Tracking_ORB-SLAM2.git
```

```
cd Attention-Tracking_ORB-SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

# 4. Building ROS
```
chmod +x build_ros.sh
./build_ros.sh
```

# 5. Run the Attention-Tracking VO of ORB-SLAM2
```
The same as the original ORB-SLAM2 system, please refer to https://github.com/raulmur/ORB_SLAM2
