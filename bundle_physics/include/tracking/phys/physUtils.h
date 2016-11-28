//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_PHYSUTILS_H
#define TRACKVIDEO_PHYSUTILS_H

namespace tracking {
namespace bundle_physics {
    template<typename _QuaternionT>
    inline _QuaternionT getEmpOmega(_QuaternionT prevPose, _QuaternionT nextPose, typename _QuaternionT::Scalar dt) {
        typedef typename _QuaternionT::Scalar Scalar;
        if ( prevPose.coeffs().dot( nextPose.coeffs() ) < Scalar(0.) )
            return (_QuaternionT(Scalar(2.) * (nextPose.coeffs() - prevPose.coeffs())) / dt * prevPose.inverse()).conjugate();
        else
            return _QuaternionT(Scalar(2.) * (nextPose.coeffs() - prevPose.coeffs())) / dt * prevPose.inverse();
    }
}
}

#endif //TRACKVIDEO_PHYSUTILS_H
