#include "tracking/vis/visualizer.h"
#include "tracking/common/maths/impl/quaternion.hpp"
#include <math.h>

namespace tracking {
    int testQuat( int /*argc*/, char **/*argv*/ ) {
        typedef float Scalar;
        typedef pa::Quaternion<Scalar> QuaternionT;
        typedef Eigen::Matrix<Scalar,3,1> Vector3;
        using namespace Soup;

        vis::Visualizer<Scalar> vis( "testQuat" );
        vis.setBackgroundColor( 1., 1., 1.);

        Vector3 pos( Vector3::Zero() );
        Eigen::Matrix3f frame( Eigen::Matrix3f::Identity() );
        QuaternionT pose( QuaternionT::Unit() );
        QuaternionT prevPose( pose), prevPose2( prevPose );
        QuaternionT omega( 0., M_PI/5., M_PI/4., M_PI/10. );
        for ( int i = 0; i != 10; ++i )
        {

            prevPose2 = prevPose;
            prevPose = pose;

            // rotate: pose = (pose + ( _Quat(_Scalar(0.5) * omega * q) * dt )).normalized();
            pa::applyOmega( pose, omega, Scalar(1.), 10. );
            std::cout << "diff: " << (pose.coeffs() - prevPose.coeffs()).transpose() << std::endl;
            // move:
            pos += Vector3(1.,0.,0.);

            std::cout << "\tpose:\t " << pose << "\n\tprevPose:\t" << prevPose << std::endl;
            std::cout << "\tomega: "
                      << QuaternionT((pose.coeffs() - prevPose2.coeffs())) * prevPose.inverse()
                      << "\n\tderiv:" << pose * prevPose.inverse()
                      << "\n\tnew: " << getOmega( prevPose, pose, 1.f ).transpose()
                      << "\n\tomega_gt \t" << omega << std::endl << std::endl;

            // display
            frame = pose * Eigen::Matrix3f::Identity();
            for ( int d = 0; d != 3; ++d )
            {
                vis.addArrow( pos, pos + frame.col(d), Vector3(d==0, d==1, d==2), "camOrient" + std::to_string(d) + "_" + std::to_string(i), .5 );
            }
        }

        vis.spin();

        return 0;
    } //...testQuat()s
} //...ns tracking
