/** Implement the algorithm of the paper "Attention and Anticipation in Fast Visual-Inertial Navigation"
* PS: the paper author did not open source his code, thus I implement the method based on the paper's 
* thought. Moreover, I sightly changed the algorithm to make it adapted to my system with my data.
*/

/* According to the paper, the system used is a Visual-Inertial Odometry, which consists of an IMU 
* and a stereo visual odometry. 
* 1.IMU will provide its own covariance matrix (with its inverse matrix - information matrix "omega_IMU"),
* and the paper also considered the rotation information are known and accurate. 
* 2.Then, the paper tried to find out the contribution of each single feature to the final state 
* information matrix.
* 3.Finally, it used greedy algorithm to pick up a certain number of best features according to their 
* contributions to the final state information matrix to get a near-optimal accurate state estimation.
*/

/* However, for my data and system, I get the ground-truth of robot's position and rotation information
* from Vicon motion capture system in UIUC Intelligent Robotics Lab (IRL). Thus, the measured rotation 
* matrix with covariance matrix from IMU part can be ignored. The contribution of each single feature to 
* the system's state can be straightforwardly calculated, and the sum of those contribution will be the 
* final information matrix of the system's state, and those features with highest contributions will be 
* selected for the system's localization and obtaining the near-optimal accurate system state estimation.
*/

#ifndef ATTENTION_H
#define ATTENTION_H

#include <vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>


namespace ORB_SLAM2
{

class MapPoint;
class KeyFrame;
class Frame;

class Attention
{
public:
    Attention();
    // Attention Feature Selection method
    std::vector<int> AttentionPick(Frame* mCurrentFrame, float ratio);
    // Random Feature Selection method
    std::vector<int> RandomPick(Frame* mCurrentFrame, float ratio);
    
public:

private:

};

}// namespace ORB_SLAM2

#endif // ATTENTION_H