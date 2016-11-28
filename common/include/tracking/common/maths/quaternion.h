#ifndef PA_QUATERNION_H
#define PA_QUATERNION_H

#include "tracking/common/eigen.h"
#include <ostream>

// ____________________________________________________________________________
// ______________________ GLOBAL EIGEN QUAT UTILS _____________________________
// ____________________________________________________________________________

// ostream Eigen
template <typename __Scalar>
std::ostream& operator<<( std::ostream& os, Eigen::Quaternion<__Scalar> const& quaternion );

template <typename __Scalar>
std::ostream& operator<<( Eigen::Quaternion<__Scalar> const& quaternion, std::ostream& os );

template <typename _Scalar>
Eigen::Quaternion<_Scalar> operator*( Eigen::Quaternion<_Scalar> const& quat, _Scalar scalar );

template <typename _Scalar>
Eigen::Quaternion<_Scalar> operator*( _Scalar scalar, Eigen::Quaternion<_Scalar> const& quat );

template <typename _Scalar>
Eigen::Quaternion<_Scalar> operator/( Eigen::Quaternion<_Scalar> const& quat, _Scalar scalar );

// ____________________________________________________________________________
// __________________________ CLASS QUATERNION ________________________________
// ____________________________________________________________________________


namespace pa {
    template <typename _Scalar>
    class Quaternion : public Eigen::Quaternion<_Scalar> {
        public:
            typedef _Scalar Scalar;
            typedef Eigen::Quaternion<_Scalar> ParentT;
            using   ParentT::Quaternion;
            using   ParentT::operator*;

            inline Quaternion() : ParentT() {}
            inline Quaternion( _Scalar w, Eigen::Matrix<_Scalar,3,1> omega ) : ParentT(w,omega(0),omega(1),omega(2)) {}

            inline Quaternion   operator+ ( Quaternion const& other ) const;
                   Quaternion&  operator+=( Quaternion const& other );
            inline Quaternion   operator* ( _Scalar scalar ) const;
            inline void         print( std::ostream& os ) const;

            template <typename __Scalar>
            friend std::ostream& operator<<( std::ostream& os, Quaternion<__Scalar> const& quaternion );
            template <typename __Scalar>
            friend std::ostream& operator<<( Quaternion<__Scalar> const& quaternion, std::ostream& os );

            static inline Quaternion Unit() { return Quaternion(_Scalar(1.), _Scalar(0.), _Scalar(0.), _Scalar(0.)); }
            static inline Quaternion Zero() { return Quaternion(_Scalar(0.), _Scalar(0.), _Scalar(0.), _Scalar(0.)); }
    }; //...cls Quaternion
} //...ns pa

// ____________________________________________________________________________
// _____________________________ QUATUTIL _____________________________________
// ____________________________________________________________________________

namespace pa
{
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    _Quat getEmpiricalOmega( _Quat q, _Quat prevQ, _Scalar dt = _Scalar(1.) );

    /*! \brief Adds angular velocity quaternion \p omega to current rotation \p q.
     *  \note Source: http://people.sc.fsu.edu/~jburkardt/cpp_src/rk4/rk4.cpp
     */
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    void applyOmega( _Quat &q, _Quat omega, _Scalar dt = _Scalar(1.), int substeps = 1 );

    /*! \brief Adds angular velocity vector \p omega to current rotation \p q.
     */
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    inline void applyOmega( _Quat &q, Eigen::Matrix<_Scalar,3,1> const& omega, _Scalar dt = _Scalar(1.), int substeps = 1 );

} //...ns pa

#endif // PA_QUATERNION_H
