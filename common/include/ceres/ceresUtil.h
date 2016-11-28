#ifndef TV_CERESUTIL_H
#define TV_CERESUTIL_H

#include "ceres/ceres.h"
#include "ceres/typedefs.h"
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

namespace ceres {
  template <typename T, int N = 0> inline const double* getVal( const      T               * val );// { return &(val->a); }
  template <>           inline const double* getVal( const double              * val ) { return   val;     }
  template <int N=0>    inline const double* getVal( const ceres::Jet<double,N>* val ) { return &(val->a); }
//    template <>           inline const double* getVal( const ceres::Jet<double,9 >* val ) { return &(val->a); }
//    template <>           inline const double* getVal( const ceres::Jet<double,19>* val ) { return &(val->a); }
//    template <>           inline const double* getVal( const ceres::Jet<double,22>* val ) { return &(val->a); }
  template <typename T> inline std::string printJet( T      val ) { std::stringstream ss; ss << val.a; return ss.str(); }
  template <>  inline std::string printJet( double val ) { std::stringstream ss; ss << val; return ss.str(); }

  template <typename T> inline std::string printJetVector3( const T val[3] ) { std::stringstream ss; ss << printJet(val[0]) << "," << printJet(val[1]) << "," << printJet(val[2]); return ss.str(); }
  template <typename T> inline std::string printJetVector4( const T val[4] ) { std::stringstream ss; ss << printJet(val[0]) << "," << printJet(val[1]) << "," << printJet(val[2]) << "," << printJet(val[3]); return ss.str(); }

  template <typename Derived> inline
  std::string printJetM( Eigen::DenseBase<Derived> const& m ) {
      std::stringstream ss;
      for ( int row = 0; row != m.rows(); ++row )
      {
          for ( int col = 0; col != m.cols(); ++col )
               ss << printJet<typename Derived::Scalar>( m(row,col) ) << ",";
          ss << "\n";
      }
      return ss.str();
  }

  template <typename T>
  inline T sqrLength( const T& x, const T& y, const T& z )
  {
      return x*x + y*y + z*z;
  }

  template <typename T>
  inline T length( const T& x, const T& y, const T& z )
  {
      T l2 = x*x + y*y + z*z;
      if ( l2 == T(0.) )
          return T(0.);
      else
          return ceres::sqrt( l2 );
  }
  template <typename T>
  inline T length( const T& x, const T& y, const T& z, const T& w )
  {
      T l2 = x*x + y*y + z*z + w*w;
      if ( l2 == T(0.) )
          return T(0.);
      else
          return ceres::sqrt( l2 );
  }

  template <typename T>
  inline T length( const T* p )
  {
      return length(p[0],p[1],p[2]);
  }
  template <typename T>
  inline T length4( const T* p )
  {
      return length(p[0],p[1],p[2],p[3]);
  }

  template <typename T>
  inline void setConstant3( const T* p, const T constant = T(0.) )
  {
      p[0] = constant;
      p[1] = constant;
      p[2] = constant;
  }

  /** \brief Implements c = a + b for 3 dimensions. */
  template<typename T>
  inline void plus3(const T a[3], const T b[3], T c[3])
  {
      c[0] = a[0] + b[0];
      c[1] = a[1] + b[1];
      c[2] = a[2] + b[2];
  }

/** \brief Implements c = a - b for 3 dimensions. */
  template<typename T>
  inline void minus3(const T a[3], const T b[3], T c[3])
  {
      c[0] = a[0] - b[0];
      c[1] = a[1] - b[1];
      c[2] = a[2] - b[2];
  }

/** \brief Implements c += a for 3 dimensions. */
  template<typename T>
  inline void plusEqual3(const T a[3], T c[3])
  {
      c[0] += a[0];
      c[1] += a[1];
      c[2] += a[2];
  }

/** \brief Implements c = a / s for 3 dimensions. */
  template<typename T>
  inline void div3(const T a[3], const T s, T c[3])
  {
      c[0] = a[0] / s;
      c[1] = a[1] / s;
      c[2] = a[2] / s;
  }

/** \brief Copies \p a to \p c
 * \param[in] a Source.
 * \param[in] c Destination.
 */
  template<typename T>
  inline void equal3(const T a[3], T c[3])
  {
      c[0] = a[0];
      c[1] = a[1];
      c[2] = a[2];
  }

  inline ceres::LossFunctionWrapper*
  createLoss(ceres::CeresScalar const weight, ceres::Ownership const wrapperOwnership = ceres::TAKE_OWNERSHIP) {
      return new ceres::LossFunctionWrapper(
          new ceres::ScaledLoss(NULL, weight < std::numeric_limits<ceres::Scalar>::epsilon() ? 0.
                                                                                             : std::sqrt(weight),
                                ceres::TAKE_OWNERSHIP), wrapperOwnership);
  }

#if 0
  ResidualBlockId AddResidualBlock(
        Problem& problem,
        CostFunction* cost_function,
        LossFunction* loss_function,
        double* x0, double* x1, double* x2, double* x3, double* x4, double* x5,
        double* x6, double* x7, double* x8, double* x9, double* x10, double* x11,
        double* x12, double* x13, double* x14, double* x15, double* x16, double* x17
    )
    {
        std::vector<double*> residual_parameters;
        residual_parameters.push_back(x0);
        residual_parameters.push_back(x1);
        residual_parameters.push_back(x2);
        residual_parameters.push_back(x3);
        residual_parameters.push_back(x4);
        residual_parameters.push_back(x5);
        residual_parameters.push_back(x6);
        residual_parameters.push_back(x7);
        residual_parameters.push_back(x8);
        residual_parameters.push_back(x9);
        residual_parameters.push_back(x10);
        residual_parameters.push_back(x11);
        residual_parameters.push_back(x12);
        residual_parameters.push_back(x13);
        residual_parameters.push_back(x14);
        residual_parameters.push_back(x15);
        residual_parameters.push_back(x16);
        residual_parameters.push_back(x17);
        return problem.AddResidualBlock( cost_function, loss_function, residual_parameters );
    }
#endif

} //...ns ceres

#endif // TV_CERESUTIL_H
