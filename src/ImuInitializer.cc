/**************************************************************************
* VIG-Init
*
* Copyright SenseTime. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/

#include "ImuInitializer.h"
#include "ImuTypes.h"
#include <bits/stdc++.h>
#include <iostream>

using namespace std;

namespace ORB_SLAM3
{
using namespace Eigen;

ImuInitializer::ImuInitializer(std::vector<KeyFrame*>& kfs, Eigen::Vector3f bg): kfs(kfs), bg(bg)
{
    GRAVITY_NOMINAL = 9.80665;
    ba.setZero();
    gravity.setZero();
    scale = 1.0;
    imuPres.resize(kfs.size());
    for (int i = 0; i < kfs.size(); i++)
    {
        imuPres[i] = std::make_shared<ORB_SLAM3::IMU::Preintegrated>();
        if (kfs[i]->mpImuPreintegrated)
            imuPres[i]->CopyFrom(kfs[i]->mpImuPreintegrated);
    }

}

ImuInitializer::~ImuInitializer() {};


bool ImuInitializer::init_imu() {
    
    //check imu observibility
    preintegrate();
    Eigen::Vector3f sum_g;
    float dis=0.0;
    for (int i = 1; i < imuPres.size(); i++)
    {
        float dt = imuPres[i]->dT;
        Eigen::Vector3f tmp_g = imuPres[i]->dV / dt; 	// v=g*t
        sum_g += tmp_g;
        dis += imuPres[i]->dP.norm();
    }

    dis = dis / (imuPres.size()-1);
    if (dis < 0.02)
        return false; 

    Eigen::Vector3f aver_g;
    aver_g = sum_g * 1.0 / ((int)imuPres.size() - 1);
    float var = 0;
    for (int i = 1; i < imuPres.size(); i++)
    {
        float dt = imuPres[i]->dT;
        Eigen::Vector3f tmp_g = imuPres[i]->dV / dt; 
        var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }

    var = sqrt(var / ((int)imuPres.size() - 1));
    if(var < 0.4)
    {
        return false;
    }
    
/*
    float scale_min_thre = 0.002;
    float scale_max_thre = 1.0;

    float bg_min_thre = -0.1;
    float bg_max_thre = 0.1;

    float bg_min, bg_max;
    float ba_min, ba_max;
    

    reset_states();
    solve_gyro_bias();

    bg_max = bg.maxCoeff();
    bg_min = bg.minCoeff();
    if (bg_min < bg_min_thre || bg_max > bg_max_thre)
        return false;
    */
    solve_gravity_scale();
/*
    if (scale < scale_min_thre || scale > scale_max_thre) 
    {
        std::cout << "scale=" << scale << ", failed in solve_gravity_scale" << std::endl;
        return false;
    }
*/
    solve_scale_ba_given_gravity();
/*
    ba_max = ba.maxCoeff();
    ba_min = ba.minCoeff();
    if (scale < scale_min_thre || scale > scale_max_thre || ba_max > 1.0 || ba_min < -1.0) 
    {
        std::cout << "scale=" << scale << ", failed in solve_scale_ba_given_gravity" << std::endl;
        return false;
    }
*/
    refine_scale_ba_via_gravity();
/*
    ba_max = ba.maxCoeff();
    ba_min = ba.minCoeff();
    if (scale < scale_min_thre || scale > scale_max_thre || ba_max > 1.0 || ba_min < -1.0)
    {
        std::cout << "scale=" << scale << ", failed in refine_scale_ba_via_gravity" << std::endl;
        return false;
    } 
*/

    std::cout << std::fixed << std::setprecision(5) << "scale=" << scale << ", bg=" << bg.transpose() << ", ba=" << ba.transpose() << ", gravity=" << gravity.transpose() << std::endl;
    
    return true;
}


// IMU::Bias b(0,0,0,0,0,0); frames[i].mpImuPreintegratedFrame->Initialize(b);
void ImuInitializer::preintegrate() {
    IMU::Bias b(ba[0],ba[1],ba[2],bg[0],bg[1],bg[2]);
    for (int i = 1; i < imuPres.size(); i++)
    {
        imuPres[i]->SetNewBias(b);
        imuPres[i]->Reintegrate();
    }
}

/*void ImuInitializer::solve_gyro_bias()
{
    preintegrate();
    Eigen::Matrix3f A;
    Eigen::Vector3f b;

    A.setZero();
    b.setZero();
    //std::cout << "frames[0].mnId=" << frames[0].mnId << std::endl;

    for (size_t j = 1; j < frames.size(); j++){
        const size_t i = j - 1;

        Sophus::SE3<float> pose_i = frames[i].GetImuPose();
        Sophus::SE3<float> pose_j = frames[j].GetImuPose();

        Eigen::Quaternionf& dq = frames[j].mpImuPreintegratedFrame->delta.q;
        Eigen::Matrix3f& dq_dbg = frames[j].mpImuPreintegratedFrame->jacobian.dq_dbg;
        if(0) //j == 1 || true)
        {
            std::cout << "pose_i=" << pose_i.matrix() << std::endl;
            std::cout << "pose_j=" << pose_j.matrix() << std::endl;
            std::cout << "dq(w, x, y, z)=" << dq.w() << ", " << dq.x() << ", " << dq.y() << ", " << dq.z() << std::endl;
            std::cout << "dq_dbg=" << dq_dbg << std::endl;
        }
        
        A += dq_dbg.transpose() * dq_dbg;
        b += dq_dbg.transpose() * ORB_SLAM3::IMU::logmap((pose_i.unit_quaternion() * dq).conjugate() * pose_j.unit_quaternion());
    }
    
    //std::cout << std::fixed << std::setprecision(5) << "A=" << A << std::endl;
    //std::cout << "b=" << b.transpose() << std::endl;

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    bg = svd.solve(b);
    //std::cout << "bg=" << bg.transpose() << std::endl;
    //std::cout << "A=" << A << std::endl;
    //std::cout << "b=" << b << std::endl;

    //std::cout << "bg=" << bg.transpose() << std::endl;
}
*/


/*void ImuInitializer::solve_gravity_scale_velocity()
{
    preintegrate();
    int N = frames.size();
    Eigen::MatrixXf A;
    Eigen::VectorXf b;
    A.resize((N - 1) * 6, 3 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    A.setZero();
    b.setZero();

    for (size_t j = 1; j < N; ++j) {
        const size_t i = j - 1;

        ORB_SLAM3::IMU::Delta &delta = frames[j].mpImuPreintegratedFrame->delta;
        Sophus::SE3<float> pose_i = frames[i].GetPose().inverse(); // Twc
        Sophus::SE3<float> pose_j = frames[j].GetPose().inverse();

        Sophus::SE3<float> pose_i_w_i = frames[i].GetImuPose(); // Twb
        Sophus::SE3<float> pose_j_w_i = frames[j].GetImuPose();

        Sophus::SE3<float> Tbc = frames[i].mImuCalib.mTcb.inverse();


        // translation()
        // rotationMatrix()

        A.block<3, 3>(i * 6, 0) += -0.5 * delta.t * delta.t * Eigen::Matrix3f::Identity();
        A.block<3, 1>(i * 6, 3) += pose_j.translation() - pose_i.translation();
        A.block<3, 3>(i * 6, 4 + i * 3) += -delta.t * Eigen::Matrix3f::Identity();

        b.segment<3>(i * 6) += pose_i_w_i.unit_quaternion() * delta.p + (pose_j_w_i.unit_quaternion() * Tbc.translation() - pose_i_w_i.unit_quaternion() * Tbc.translation());
        A.block<3, 3>(i * 6 + 3, 0) += -delta.t * Eigen::Matrix3f::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + i * 3) += -Eigen::Matrix3f::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + j * 3) += Eigen::Matrix3f::Identity();
        b.segment<3>(i * 6 + 3) += pose_i_w_i.unit_quaternion() * delta.v;
        
    }

    Eigen::VectorXf x = A.fullPivHouseholderQr().solve(b);
    
    gravity = x.segment<3>(0).normalized() * PVIO_GRAVITY_NOMINAL;
    scale = x(3);
    for (size_t i = 0; i < frames.size(); ++i) {
        velocities[i] = x.segment<3>(4 + i * 3);
    }
}*/

/*void ImuInitializer::refine_scale_velocity_via_gravity()
{
    static const double damp = 0.1;
    preintegrate();
    int N = frames.size();
    Eigen::MatrixXf A;
    Eigen::VectorXf b;
    Eigen::VectorXf x;

    A.resize((N - 1) * 6, 2 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    x.resize(2 + 1 + 3 * N);

    for (size_t iter = 0; iter < 5; ++iter) {
        A.setZero();
        b.setZero();
        x.setZero();
        Eigen::Matrix<float, 3, 2> Tg = s2_tangential_basis(gravity);

        for (size_t j = 1; j < frames.size(); j++){
            const size_t i = j - 1;

            const ORB_SLAM3::IMU::Delta &delta = frames[j].mpImuPreintegratedFrame->delta;
            Sophus::SE3<float> pose_i = frames[i].GetPose().inverse(); // Twc
            Sophus::SE3<float> pose_j = frames[j].GetPose().inverse(); // Twc

            Sophus::SE3<float> pose_i_w_i = frames[i].GetImuPose();
            Sophus::SE3<float> pose_j_w_i = frames[j].GetImuPose();

            Sophus::SE3<float> Tbc = frames[i].mImuCalib.mTcb.inverse();;

            A.block<3, 2>(i * 6, 0) += -0.5 * delta.t * delta.t * Tg;
            A.block<3, 1>(i * 6, 2) += pose_j.translation() - pose_i.translation();
            
            A.block<3, 3>(i * 6, 3 + i * 3) += -delta.t * Eigen::Matrix3f::Identity();
            b.segment<3>(i * 6) += 0.5 * delta.t * delta.t * gravity + pose_i_w_i.unit_quaternion() *  delta.p + (pose_j_w_i.unit_quaternion() * Tbc.translation() - pose_i_w_i.unit_quaternion() * Tbc.translation());


            A.block<3, 2>(i * 6 + 3, 0) += -delta.t * Tg;
            A.block<3, 3>(i * 6 + 3, 3 + i * 3) += -Eigen::Matrix3f::Identity();
            
            A.block<3, 3>(i * 6 + 3, 3 + j * 3) += Eigen::Matrix3f::Identity();
            b.segment<3>(i * 6 + 3) += delta.t * gravity + pose_i_w_i.unit_quaternion() * delta.v;
        }

        x = A.fullPivHouseholderQr().solve(b);
        Eigen::Vector2f dg = x.segment<2>(0);
        gravity = (gravity + damp * Tg * dg).normalized() * PVIO_GRAVITY_NOMINAL;
    }

    scale = x(2);
    for (size_t i = 0; i < frames.size(); ++i) {
        velocities[i] = x.segment<3>(3 + i * 3);
    }
}*/




void ImuInitializer::solve_gravity_scale() {
    Eigen::Matrix4f A;
    Eigen::Vector4f b;
    A.setZero();
    b.setZero();

    for (size_t i = 2; i < imuPres.size(); ++i) {

        ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i-1].get();
        ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

        Sophus::SE3f Twb1 = kfs[i-2]->GetImuPose();
        Sophus::SE3f Twb2 = kfs[i-1]->GetImuPose();
        Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

        Eigen::Matrix<float, 3, 4> C;
        C.block<3, 3>(0, 0) = -0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * Matrix3f::Identity();
        C.block<3, 1>(0, 3) = pInt12->dT * (Twb3.translation() - Twb2.translation()) - pInt23->dT * (Twb2.translation() - Twb1.translation());
        Eigen::Vector3f d = pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) + pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) - pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
        A += C.transpose() * C;
        b += C.transpose() * d;
    }

    JacobiSVD<Eigen::Matrix4f> svd(A, ComputeFullU | ComputeFullV);
    Eigen::Vector4f x = svd.solve(b);
    gravity = x.segment<3>(0).normalized() * GRAVITY_NOMINAL;
    scale = x(3);
}



void ImuInitializer::solve_scale_ba_given_gravity() {
    Eigen::Matrix4f A;
    Eigen::Vector4f b;
    A.setZero();
    b.setZero();

    for (size_t i = 2; i < imuPres.size(); ++i) {

        ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i-1].get();
        ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

        Sophus::SE3f Twb1 = kfs[i-2]->GetImuPose();
        Sophus::SE3f Twb2 = kfs[i-1]->GetImuPose();
        Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

        Eigen::Matrix<float, 3, 4> C;

        C.block<3, 1>(0, 0) = pInt12->dT * (Twb3.translation() - Twb2.translation()) - pInt23->dT * (Twb2.translation() - Twb1.translation());
        C.block<3, 3>(0, 1) = -(Twb2.rotationMatrix() * pInt23->JPa * pInt12->dT + Twb1.rotationMatrix() * pInt12->JVa * pInt12->dT * pInt23->dT - Twb1.rotationMatrix() * pInt12->JPa * pInt23->dT);
        Vector3f d = 0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * gravity + pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) + pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) - pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
        A += C.transpose() * C;
        b += C.transpose() * d;
    }

    JacobiSVD<Eigen::Matrix4f> svd(A, ComputeFullU | ComputeFullV);
    Eigen::Vector4f x = svd.solve(b);
    scale = x(0);
    ba = x.segment<3>(1);
}



void ImuInitializer::refine_scale_ba_via_gravity() {
    static const float damp = 0.1;
    Eigen::Matrix<float, 6, 6> A;
    Eigen::Matrix<float, 6, 1> b;
    for (size_t iter = 0; iter < 3; ++iter) {
        A.setZero();
        b.setZero();
        Eigen::Matrix<float, 3, 2> Tg = s2_tangential_basis(gravity);

        preintegrate();
        for (size_t i = 2; i < imuPres.size(); ++i) {
            
            ORB_SLAM3::IMU::Preintegrated* pInt12 = imuPres[i-1].get();
            ORB_SLAM3::IMU::Preintegrated* pInt23 = imuPres[i].get();

            Sophus::SE3f Twb1 = kfs[i-2]->GetImuPose();
            Sophus::SE3f Twb2 = kfs[i-1]->GetImuPose();
            Sophus::SE3f Twb3 = kfs[i]->GetImuPose();

            Matrix<float, 3, 6> C;
            C.block<3, 1>(0, 0) = pInt12->dT * (Twb3.translation() - Twb2.translation()) - pInt23->dT * (Twb2.translation() - Twb1.translation());
            C.block<3, 3>(0, 1) = -(Twb2.rotationMatrix() * pInt23->JPa * pInt12->dT + Twb1.rotationMatrix() * pInt12->JVa * pInt12->dT * pInt23->dT - Twb1.rotationMatrix() * pInt12->JPa * pInt23->dT);
            C.block<3, 2>(0, 4) = -0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * Tg;
            Vector3f d = 0.5 * pInt12->dT * pInt23->dT * (pInt12->dT + pInt23->dT) * gravity + pInt12->dT * (Twb2.rotationMatrix() * pInt23->dP) + pInt12->dT * pInt23->dT * (Twb1.rotationMatrix() * pInt12->dV) - pInt23->dT * (Twb1.rotationMatrix() * pInt12->dP);
            A += C.transpose() * C;
            b += C.transpose() * d;
        }

        JacobiSVD<Eigen::Matrix<float, 6, 6>> svd(A, ComputeFullU | ComputeFullV);
        Eigen::Matrix<float, 6, 1> x = svd.solve(b);
        scale = x(0);
        ba += damp * x.segment<3>(1);
        gravity = (gravity + damp * Tg * x.segment<2>(4)).normalized() * GRAVITY_NOMINAL;
    }
}



Eigen::Matrix<float, 3, 2> ImuInitializer::s2_tangential_basis(Eigen::Vector3f &x) {
    int d = 0;
    for (int i = 1; i < 3; ++i) {
        if (abs(x[i]) > abs(x[d])) d = i;
    }
    Eigen::Vector3f b1 = x.cross(Eigen::Vector3f::Unit((d + 1) % 3)).normalized();
    Eigen::Vector3f b2 = x.cross(b1).normalized();
    return (Eigen::Matrix<float, 3, 2>() << b1, b2).finished();
}




}
