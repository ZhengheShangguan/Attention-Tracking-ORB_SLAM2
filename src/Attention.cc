/** Implement the algorithm of the paper "Attention and Anticipation in Fast Visual-Inertial Navigation"
* PS: the paper author did not open source his code, thus I implement the method based on the paper's 
* thought. Moreover, I changed the algorithm to make it adapted to my system with my data.
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

#include"Attention.h"

#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>

#include "Frame.h"
#include <algorithm>
#include <thread>

using namespace std;

namespace ORB_SLAM2
{

Attention::Attention()
{}


// Attention Feature Selection function
std::vector<int> Attention::AttentionPick(Frame* mCurrentFrame, float ratio)
{
    // // for debugging
    // clock_t select_start_time = clock();


    // constant numbers: constrained match number & keypoint number
    int constr_matches = 0;
    int match_num = 0;

    // variables needed for calculating the contribution
    cv::Mat u_klcal(3,3,CV_32F);
    cv::Mat mK_inv(3,3,CV_32F);
    cv::Mat U_kl(3,3,CV_32F);
    cv::Mat plt(3,1,CV_32F);
    cv::Mat pt_skew(3,3,CV_32F);
    cv::Mat p_l(3,1,CV_32F);

    // the contribution matrix: for all features & for a single feature
    cv::Mat A_tmp = cv::Mat::zeros(3,6,CV_32F);
    cv::Mat A_part1 = A_tmp(cv::Rect(0,0,3,3)); // PS: rect(col,row,col_num,row_num). rect(Tp_x, Tp_y, width, height)
    cv::Mat A_part2 = A_tmp(cv::Rect(3,0,3,3));

    // set index matrices
    std::vector<int> Index_match;
    std::vector<int> Index_best;

    // variables wrt selection function f
    cv::Mat f_val(6,1,CV_32F);
    cv::Mat f_vec(6,1,CV_32F);
    cv::Mat S = cv::Mat::zeros(6, 6, CV_32F);
    int max_ind;
    std::vector<float>::iterator it_max_ctrb;
    float f_tmp;
    cv::Mat delta_tmp(6,6,CV_32F);

    // // for debugging
    // clock_t select_stop_time01 = clock();


    // For-loop One: Compute information contribution for each matched feature

    // first, get the number of valid matches
    for (int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            match_num++;
        }
    }

    // // for debugging
    // clock_t select_stop_time02 = clock();

    // get inverse of calibration matrix mK
    cv::invert(mCurrentFrame->mK, mK_inv, 2);
    // initialize: upbounds matrix & information matrix -- Omega (= delta) matrix
    cv::Mat upbounds = cv::Mat::zeros(match_num,1, CV_32F);
    cv::Mat boundInd = cv::Mat::zeros(match_num,1, CV_16U);
    std::vector<cv::Mat> delta_local(match_num);
    // counting the serial number of matched features only
    int count_match = 0;

    // get matched feature's candidates
    for (int i=0; i<mCurrentFrame->N; i++)
    {
        if(mCurrentFrame->mvpMapPoints[i])
        {
            // give values to the needed variables
            u_klcal.at<float>(0,0) = mCurrentFrame->mvKeysUn[i].pt.x;
            u_klcal.at<float>(1,0) = mCurrentFrame->mvKeysUn[i].pt.y;
            u_klcal.at<float>(2,0) = 1.0f;
            u_klcal = mK_inv * u_klcal;

            U_kl.at<float>(0,0) = 0.0f;
            U_kl.at<float>(0,1) = -u_klcal.at<float>(2,0);
            U_kl.at<float>(0,2) = u_klcal.at<float>(1,0);
            U_kl.at<float>(1,0) = u_klcal.at<float>(2,0);
            U_kl.at<float>(1,1) = 0.0f;
            U_kl.at<float>(1,2) = -u_klcal.at<float>(0,0);
            U_kl.at<float>(2,0) = -u_klcal.at<float>(1,0);
            U_kl.at<float>(2,1) = u_klcal.at<float>(0,0);
            U_kl.at<float>(2,2) = 0.0f;

            p_l = mCurrentFrame->mvpMapPoints[i]->GetWorldPos().clone();

            plt = mCurrentFrame->mRcw * p_l;
            pt_skew.at<float>(0,0) = 0.0f;
            pt_skew.at<float>(0,1) = -plt.at<float>(2,0);
            pt_skew.at<float>(0,2) = plt.at<float>(1,0);
            pt_skew.at<float>(1,0) = plt.at<float>(2,0);
            pt_skew.at<float>(1,1) = 0.0f;
            pt_skew.at<float>(1,2) = -plt.at<float>(0,0);
            pt_skew.at<float>(2,0) = -plt.at<float>(1,0);
            pt_skew.at<float>(2,1) = plt.at<float>(0,0);
            pt_skew.at<float>(2,2) = 0.0f;
            
            // calculate two contribution matrices
            A_part1 = -U_kl * pt_skew;
            A_part2 = U_kl;
            delta_tmp = A_tmp.t() * A_tmp;
            delta_tmp.convertTo(delta_tmp, CV_32F);
            delta_local[count_match] = delta_tmp.clone();
            
            Index_match.push_back(i);
            count_match++;
        }
    }

    // // for debugging
    // clock_t select_stop_time03 = clock();


    // For-loop Two: Compute upperbounds for each possible added feature & Find out the best feature with Greedy algorithm

    // Choose max of both as our constr_matches
    constr_matches = 20;//std::max((int)(ratio*match_num), std::min(60, match_num));
    // Make the final selected number to be a multiples of 5
    constr_matches = 5 * ((int)(constr_matches/5));


    // // for debugging
    // clock_t select_stop_time04 = clock();





    // Greedy algorithm with lazy evaluation: get the best ranked features
    std::vector<float> ctrb_curr(match_num);
    for (int i2=0; i2<(constr_matches/5); i2++)
    {
        // calculate and pushback all available features' current scores
        for (int i4=0; i4<match_num; i4++)
        {
            // if the feature has been selected, ignore it where -1.0f as a mark.
            if (ctrb_curr[i4] == -1.0f)
            {
                continue;
            }

            // calculate the information matrix if this feature is added
            cv::Mat mat_tmp = S + delta_local[i4];
            mat_tmp.convertTo(mat_tmp, CV_32F);
            cv::eigen(mat_tmp, f_val, f_vec);
            // obtain the feature's contribution and push it back the vector
            f_tmp = f_val.at<float>(5,0);
            // cancel the rounding error, choose the threshold as 1e-6
            if (f_tmp < 1e-6)
                f_tmp = 0.0f;
            ctrb_curr[i4] = f_tmp;
        }

        // find the top 5 features and push them into the Index_best vector
        for (int i5=0; i5<5; i5++)
        {
            it_max_ctrb = std::max_element(ctrb_curr.begin(), ctrb_curr.end());
            max_ind = std::distance(ctrb_curr.begin(), it_max_ctrb);
            Index_best.push_back(Index_match[max_ind]);
            S = S + delta_local[max_ind];

            ctrb_curr[max_ind] = -1.0f;
        }
    }

    // // for debugging
    // clock_t select_stop_time05 = clock();


    // sort the vector before return
    std::sort(Index_best.begin(), Index_best.end());


    // // for debugging
    // clock_t select_stop_time06 = clock();


    // // for debugging: write these vectors into several .txt files
    // double time01 = ((double)(select_stop_time01 - select_start_time))/CLOCKS_PER_SEC;
    // double time02 = ((double)(select_stop_time02 - select_start_time))/CLOCKS_PER_SEC;
    // double time03 = ((double)(select_stop_time03 - select_start_time))/CLOCKS_PER_SEC;
    // double time04 = ((double)(select_stop_time04 - select_start_time))/CLOCKS_PER_SEC;
    // double time05 = ((double)(select_stop_time05 - select_start_time))/CLOCKS_PER_SEC;
    // double time06 = ((double)(select_stop_time06 - select_start_time))/CLOCKS_PER_SEC;
    // std::ofstream Select_data;
    // Select_data.open("Select_data01.txt", std::fstream::app);
    // Select_data << std::fixed << time01 << endl;
    // Select_data.close();
    // Select_data.open("Select_data02.txt", std::fstream::app);
    // Select_data << std::fixed << time02 << endl;
    // Select_data.close();
    // Select_data.open("Select_data03.txt", std::fstream::app);
    // Select_data << std::fixed << time03 << endl;
    // Select_data.close();
    // Select_data.open("Select_data04.txt", std::fstream::app);
    // Select_data << std::fixed << time04 << endl;
    // Select_data.close();
    // Select_data.open("Select_data05.txt", std::fstream::app);
    // Select_data << std::fixed << time05 << endl;
    // Select_data.close();
    // Select_data.open("Select_data06.txt", std::fstream::app);
    // Select_data << std::fixed << time06 << endl;
    // Select_data.close();



    // set the flag for those best MapPoints 
    // (also we need to change OptimizaPose function)
    return Index_best;
}



// Random Feature Selection Method
// Attention Feature Selection function
std::vector<int> Attention::RandomPick(Frame* mCurrentFrame, float ratio)
{
    std::vector<int> seq_rdm;
    std::vector<int> Index_best;
    for (int i=0;i<mCurrentFrame->N;i++)
        if (mCurrentFrame->mvpMapPoints[i])
            seq_rdm.push_back(i);
    std::random_shuffle(seq_rdm.begin(), seq_rdm.end());

    int constr = 20;// std::max((int)(ratio*((float)seq_rdm.size())), std::min(60, (int)seq_rdm.size()));
    constr = 5 * ((int)(constr/5));

    for(int ii=0; ii < constr; ii++)
    {
        int kk = seq_rdm[ii];
        Index_best.push_back(kk);
    }

    return Index_best;
}

} //namespace ORB_SLAM2
