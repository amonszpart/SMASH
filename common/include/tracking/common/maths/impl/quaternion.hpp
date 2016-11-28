#ifndef PA_QUATERNION_HPP
#define PA_QUATERNION_HPP

#include "tracking/common/eigen.h"
#include "tracking/common/maths/quaternion.h"
#include <ostream>

// ____________________________________________________________________________
// ______________________ GLOBAL EIGEN QUAT UTILS _____________________________
// ____________________________________________________________________________

// ostream Eigen
template <typename __Scalar>
std::ostream& operator<<( std::ostream& os, Eigen::Quaternion<__Scalar> const& quaternion )
{
    os << quaternion.coeffs().transpose();
    return os;
}

template <typename __Scalar>
std::ostream& operator<<( Eigen::Quaternion<__Scalar> const& quaternion, std::ostream& os )
{
    os << quaternion.coeffs().transpose();
    return os;
}

template <typename _Scalar>
inline Eigen::Quaternion<_Scalar> operator*( Eigen::Quaternion<_Scalar> const& quat, _Scalar scalar );
template <>
inline Eigen::Quaternion<float> operator*( Eigen::Quaternion<float> const& quat, float scalar )
{
    return Eigen::Quaternion<float>( quat.coeffs() * scalar );
}
template <>
inline Eigen::Quaternion<double> operator*( Eigen::Quaternion<double> const& quat, double scalar )
{
    return Eigen::Quaternion<double>( quat.coeffs() * scalar );
}

template <typename _Scalar>
inline Eigen::Quaternion<_Scalar> operator*( _Scalar scalar, Eigen::Quaternion<_Scalar> const& quat )
{
    return quat * scalar;
}

//template <>
//inline Eigen::Quaternion<double> operator*( double scalar, Eigen::Quaternion<double> const& quat )
//{
//    return quat * scalar;
//}

template <typename _Scalar>
inline Eigen::Quaternion<_Scalar> operator/( Eigen::Quaternion<_Scalar> const& quat, _Scalar scalar )
{
    return Eigen::Quaternion<_Scalar>( quat.coeffs() / scalar );
}

// ____________________________________________________________________________
// ________________________________ HPP _______________________________________
// ____________________________________________________________________________

namespace pa
{
    template <typename _Scalar>
    Quaternion<_Scalar> Quaternion<_Scalar>::operator*( _Scalar scalar ) const
    {
        return Quaternion( this->coeffs() * scalar );
    }

    template <typename _Scalar>
    Quaternion<_Scalar> Quaternion<_Scalar>::operator+( Quaternion<_Scalar> const& other ) const
    {
        return Quaternion( this->coeffs() + other.coeffs() );
    }

    template <typename _Scalar>
    Quaternion<_Scalar>& Quaternion<_Scalar>::operator+=( Quaternion<_Scalar> const& other )
    {
        this->coeffs() += other.coeffs();
        return *this;
    }

    template <typename _Scalar>
    void Quaternion<_Scalar>::print( std::ostream& os ) const
    {
        os << this->coeffs().transpose();
    }

    template <typename _Scalar>
    Quaternion<_Scalar> operator*( _Scalar scalar, Quaternion<_Scalar> const& quat )
    {
        return quat.operator *( scalar );
    }

    template <typename _Scalar>
    Quaternion<_Scalar> operator*( Quaternion<_Scalar> const& quat, _Scalar scalar )
    {
        return quat.operator *( scalar );
    }

    template <typename _Scalar>
    std::ostream& operator<<( std::ostream& os, pa::Quaternion<_Scalar> const& quaternion )
    {
        quaternion.print( os );
        return os;
    }

    template <typename _Scalar>
    std::ostream& operator<<( pa::Quaternion<_Scalar> const& quaternion, std::ostream& os )
    {
        quaternion.print( os );
        return os;
    }
} //...ns pa

// ____________________________________________________________________________
// _____________________________ QUATUTIL _____________________________________
// ____________________________________________________________________________

namespace pa
{
    template <typename _QuaternionT, typename _Scalar = typename _QuaternionT::Scalar>
    inline Eigen::Matrix<_Scalar,3,1> getOmega( const _QuaternionT& prevPose, const _QuaternionT& pose, _Scalar dt = _Scalar(1.) )
    {
        typedef Eigen::Matrix<_Scalar,3,1> Vector3;

        _QuaternionT q = pose * prevPose.inverse();

        _Scalar theta0  = _Scalar(2.) * std::acos( q.coeffs()(3) ); // angle of prev pose
        Vector3 e0      = q.coeffs().template head<3>();
        _Scalar norm0   = std::sin( theta0/_Scalar(2.) );
        if ( norm0 > _Scalar(0.) )  { e0 /= norm0; }

        return -theta0 * dt * e0;
    } //...getOmega()

    /** \brief Estimates angular velocity from two consequtive quaternion poses.
     * \tparam _QuaternionT Quaternion type. Concept: \ref pa::Quaternion.
     * \tparam _Scalar      Floating point type. Concept: float.
     * \param[in] pose      Current pose.
     * \param[in] prevPose  Previous pose.
     * \param[in] dt        Time elapsed between the two poses.
     * \return              Angular velocity vector \f$ \begin{bmatrix} \omega_x, \omega_y, \omega_z \end{bmatrix} \f$.
     * \see                 Formula by reverting https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Quaternions.
     */
    template <typename _QuaternionT, typename _Scalar = typename _QuaternionT::Scalar>
    inline Eigen::Matrix<_Scalar,3,1> getOmega2( const _QuaternionT& prevPose, const _QuaternionT& pose, _Scalar dt = _Scalar(1.) )
    {
        typedef Eigen::Matrix<_Scalar,3,1> Vector3;

        Vector3 outAxis ( Vector3::Zero() ); // axis to be returned
        int     outCount( 0               ); // #axes to average to output

        _Scalar theta0  = _Scalar(2.) * std::acos( prevPose.coeffs()(3) ); // angle of prev pose
        Vector3 e0      = prevPose.coeffs().template head<3>();
        _Scalar norm0   = e0.norm();

        if ( norm0 > _Scalar(0.) )  { e0 /= norm0; outAxis += e0; ++outCount; }
        std::cout << "theta0: " << theta0 << ", e: " << e0.transpose() << std::endl;

        _Scalar theta1 = 2. * std::acos( pose.coeffs()(3) );
        Vector3 e1 = pose.coeffs().template head<3>();

        //  fix opposite axes
        if ( e1.dot(e0) < _Scalar(0.) )
            e1 *= _Scalar(-1.);

        _Scalar norm1 = e1.norm();
        if ( norm1 > _Scalar(0.) )  { e1 /= norm1; outAxis += e1; ++outCount; }
        std::cout << "theta1: "<< theta1 << ", e: " << e1.transpose() << std::endl;

        if ( (e1 - e0).norm() > _Scalar(0.1) )
        {
            std::cerr << "[" << __func__ << "]: " << "not the same rotation axis...this won't work: " << (e0 - e1).norm() << " from " << e0.transpose() << " and " << e1.transpose() << std::endl;
            return (_QuaternionT(_Scalar(2.) * (pose.coeffs() - prevPose.coeffs())) * pose.inverse()).coeffs().template head<3>();
        }

        return (theta0-theta1) * dt * outAxis/_Scalar(outCount);
    } //...getOmega()

    /** \brief Estimates angular velocity from two quaternion poses.
     *  \deprecated Is not accurate for large omegas because of dq/dt being approximate.
     *  \todo Use central differences: QuaternionT((nextPose.coeffs() - prevPose.coeffs())) * pose.inverse()
     */
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    inline _Quat getEmpiricalOmega( _Quat q, _Quat prevQ, _Scalar dt )
    {
        // W = 2 dq/dt q' = 2 (q prevQ')/dt q'
        // WORKED(real): return _Quat(-_Scalar(2.) * q * prevQ.inverse()) / dt * prevQ.inverse();
        return _Quat(-_Scalar(2.) * q * prevQ.inverse()) / dt * prevQ.inverse();
    }

    /*! \brief Adds angular velocity quaternion \p omega to current rotation \p q.
     *  \note Source: http://people.sc.fsu.edu/~jburkardt/cpp_src/rk4/rk4.cpp
     */
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    inline void applyOmega( _Quat &q, _Quat omega, _Scalar dt, int substeps)
    {
        // q(t+dt) = q(t) + q(t)/dt = q(t) + ½ W(t) q(t)
        // WORKED(real): q = (q + ( _Quat(_Scalar(0.5) * omega * q) * dt )).normalized();
        if ( substeps == 1 )
            q = (q + ( _Quat(_Scalar(0.5) * omega * q) * dt )).normalized();
        else
        {
            const _Scalar dtFrac = dt / _Scalar( substeps );
            for ( int i = 0; i != substeps; ++i )
            {
                applyOmega( q, omega, dtFrac, 1 );
            }
        }
    }

    /*! \brief Adds angular velocity vector \p omega to current rotation \p q.
     */
    template <class _Quat, typename _Scalar = typename _Quat::Scalar>
    inline void applyOmega( _Quat &q, Eigen::Matrix<_Scalar,3,1> const& omega, _Scalar dt, int substeps )
    {
        // W(t) = [ 0, omega_x, omega_y, omega_z ]
        applyOmega( q, _Quat(_Scalar(0.), omega(0), omega(1), omega(2)), dt, substeps );
    }

} //...ns pa

#endif // PA_QUATERNION_HPP
