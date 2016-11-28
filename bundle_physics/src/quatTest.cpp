//
// Created by bontius on 17/05/16.
//

#include <tracking/phys/physUtils.h>
#include <tracking/phys/typedefsGeometry.h>
#include "tracking/phys/inertiaEstimation.hpp"
#include "tracking/phys/typedefs.h"
#include "tracking/common/eigen.h"
#include <ceres/typedefs.h>
#include <iostream>

template<typename _QuaternionT>
inline _QuaternionT getEmpOmega(_QuaternionT prevPose, _QuaternionT nextPose, typename _QuaternionT::Scalar dt)
{
    typedef typename _QuaternionT::Scalar Scalar;
    _QuaternionT tmp(Scalar(2.) * (nextPose.coeffs() - prevPose.coeffs()));
    tmp.coeffs() /= dt;
    tmp *= prevPose.inverse();
    if ( prevPose.coeffs().dot( nextPose.coeffs() ) < Scalar(0.) )
//            return (_QuaternionT(Scalar(2.) * (nextPose.coeffs() - prevPose.coeffs())) / dt * prevPose.inverse()).conjugate();
        return tmp.conjugate();
    else
        return tmp;
//            return _QuaternionT(Scalar(2.) * (nextPose.coeffs() - prevPose.coeffs())) / dt * prevPose.inverse();
}

using namespace tracking::bundle_physics;
Eigen::Vector3f getMomentum( QuaternionT prevPose, double prevT, QuaternionT initPose, double currT, Eigen::Vector3f size) {
    using ceres::CeresScalar;
    using ceres::CeresMatrix3;

    prevPose.normalize();
    initPose.normalize();
    ceres::CeresVector3 sqrSized(size(0)*size(0), size(1)*size(1), size(2) * size(2));
    ceres::CeresVector3 initialI;
    ceres::CeresScalar  massd(0.393 / 0.237);
    getI(tracking::SHAPE::BOX, &massd, sqrSized.data(), initialI.data());
    const CeresMatrix3 R = initPose.toRotationMatrix().cast<CeresScalar>();
    std::cout << "R:\n" << R << std::endl;
    CeresMatrix3 currI = R * initialI.asDiagonal() * R.transpose();
    std::cout << "currI of cuboid " << ":\n" << currI << std::endl;

    const QuaternionT omega0 = ::getEmpOmega<QuaternionT>(prevPose, initPose, QuaternionT::Scalar((currT - prevT) / 240.f));
    std::cout << "[" << __func__ << "] " << "omega0: " << omega0.coeffs().transpose() << std::endl;
//    std::cout << "[" << __func__ << "] " << "momentum: " << currI.cast<float>() * omega0.coeffs().head<3>() << std::endl;
    return currI.cast<float>() * omega0.coeffs().head<3>();
}

QuaternionT fromBlender(QuaternionT q) {
//    return (QuaternionT(q.coeffs()(3), q.coeffs()(0), -q.coeffs()(2), q.coeffs()(1)) * QuaternionT(0.7071067690849304, 0.7071067690849304, 0.0, 0.0)).normalized();
    return QuaternionT(q.coeffs()(3), q.coeffs()(0), -q.coeffs()(2), q.coeffs()(1));
}

int quatTest(int /*argc*/, char **/*argv*/) {

    Eigen::Vector3f siz(0.197, 0.161,  0.146);
    QuaternionT q0(0.578f, 0.308f, 0.495f, 0.668f); // 224
    QuaternionT q1(0.427f, 0.921f, 0.240f, 0.027f); // 240
    std::cout << "[" << __func__ << "] " << "duckAfter: " << getMomentum(q0, 224., q1, 240.,siz).transpose() << std::endl;

    q0 = QuaternionT(0.103f, 0.112f, 0.634f, -0.674f); // 134
    q1 = QuaternionT(0.115f, 0.052f, 0.827f, -0.618f); // 124
    std::cout << "[" << __func__ << "] " << "duckBefore: " << getMomentum(q0, 124., q1, 134., siz).transpose() << std::endl;

    siz = Eigen::Vector3f(0.349, 0.207, 0.208);
    q0 = QuaternionT(0.151,-0.447,0.452,-0.522); // 206
    q1 = QuaternionT(0.081,-0.813,-0.057,-0.170); //260
    std::cout << "[" << __func__ << "] " << "rugbyAfter: " << getMomentum(fromBlender(q0), 206., fromBlender(q1), 260., siz).transpose() << std::endl;

    q0 = QuaternionT(-0.446, -0.344, 0.472, -0.399); // 121
    q1 = QuaternionT(0.187,0.236,0.523,-0.579); //98
    std::cout << "[" << __func__ << "] " << "rugbyBefore: " << getMomentum(q0, 121, q1, 98, siz).transpose() << std::endl;

    return EXIT_SUCCESS;
}