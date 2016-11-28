#ifndef TV_FITCUBOID_H
#define TV_FITCUBOID_H

#include "ceres/ceres.h"
#include "tracking/common/track.h"
#include "tracking/common/mapper.h"
#include "ceres/ceresUtil.h"
#include "tracking/common/correspondence.h"
#include "tracking/annot/cuboid.h"
#include "soup/util/exception.h"

namespace tracking
{
    /** \brief Converts u,v, and depth (z) to x and y coordinates using the intrinsic matrix.
     *  \tparam T        Type of variable. Concept: ceres::Jet::... or plain double.
     *  \tparam _Matrix3 3x3 matrix to store the intrinsics. Concept: Eigen::Matrix3f.
     */
    template <typename T, typename _Matrix3>
    inline bool getXy( const T* uvz, T* xy, const _Matrix3& intrinsics )
    {
        xy[0] = (  uvz[0] / T(intrinsics(0,0)) - T(intrinsics(0,2) / intrinsics(0,0))  ) * uvz[2];
        xy[1] = (  uvz[1] / T(intrinsics(1,1)) - T(intrinsics(1,2) / intrinsics(1,1))  ) * uvz[2];
        //y = (  (T(height - 1) - uvz[1]) / T(intrinsics(1,1)) - T(intrinsics(1,2) / intrinsics(1,1))  ) * uvz[2];
        return true;
    }

    /** \brief Ensures that tracks remain true to 2D observations.
      * \tparam _Matrix3 3x3 matrix to store the intrinsics. Concept: Eigen::Matrix3f.
      */
    template <typename _Matrix3>
    struct OnTrackCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            OnTrackCostFunctor( const _Matrix3& intrinsics, double u, double v, const int width, const int height, const double weight )
                : _intrinsics( intrinsics ), _u( u ), _v ( v ), _width( width ), _height( height ), _weight( weight )
            {} //...CostFunctor()

            template <typename T> bool operator() ( const T* uvz, T* residuals ) const
            {
                //          _u, _v: observation
                //             uvz: current variable value
                // _width, _height: image dimensions for normalization
                residuals[0] = T(_weight) * (T(_u) - uvz[0]) / T(_width );
                //std::cout << "_weight(" << _weight << ") * (_u( " << _u << ") - u(" << uvz[0] << ")) / _width(" << _width << ") = " << residuals[0] << std::endl;
                residuals[1] = T(_weight) * (T(_v) - uvz[1]) / T(_height);
                //std::cout << "_weight(" << _weight << ") * (_v( " << _v << ") - v(" << uvz[1] << ")) / _height(" << _height << ") = " << residuals[1] << std::endl;
                if ( uvz[2] < T(0.1) )
                    residuals[2] = T(_weight) * (T(0.1) - uvz[2]);
                else
                    residuals[2] = T(0.);

                return true;
            } //...operator()

            // Factory to hide the construction of the CostFunction object from
            // the client code.
            static ceres::CostFunction* Create( const _Matrix3& intrinsics, double u, double v, const int width, const int height, const double weight )
            {
                return
                        new ceres::AutoDiffCostFunction<OnTrackCostFunctor<_Matrix3>, /* num_residuals: */ 2, /* num_params: */ 3>(
                            new OnTrackCostFunctor<_Matrix3>(intrinsics(), u, v, width, height, weight) );

            } //...Create()
        protected:
            const _Matrix3  _intrinsics;
            const double    _u, _v;
            const int       _width, _height;
            const double    _weight;
    }; //...OnTrackCostFunctor

    /** \brief Ensures rigid distance between 3D points of two tracks: \f$ X^0_t - X^1_t == X^0_{t-1} - X^1_{t-1} \f$.
      *  \tparam _Matrix3 3x3 matrix to store the intrinsics. Concept: Eigen::Matrix3f.
      */
    template <typename _Matrix3>
    struct EqualDistanceCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            EqualDistanceCostFunctor( const _Matrix3& intrinsics, const double weight, const double smoothWeight )
                : _intrinsics( intrinsics ), _weight( weight ), _smoothWeight( smoothWeight )
            {} //...CostFunctor()

            template <typename T> bool operator() ( const T* prev0, const T* curr0, const T* prev1, const T* curr1, T* residuals ) const
            {
                using ceres::length;

                T prevX0[2], currX0[2], prevX1[2], currX1[2];
                getXy( prev0, prevX0, _intrinsics );
                getXy( curr0, currX0, _intrinsics );
                getXy( prev1, prevX1, _intrinsics );
                getXy( curr1, currX1, _intrinsics );

                T prevLen = length( prevX0[0] - prevX1[0], prevX0[1] - prevX1[1], prev0[2] - prev1[2] );
                T currLen = length( currX0[0] - currX1[0], currX0[1] - currX1[1], curr0[2] - curr1[2] );

                residuals[0] = currLen - prevLen;

                // point smoothness
                residuals[1] = T(_smoothWeight) * (prevX0[0] - currX0[0]);
                residuals[2] = T(_smoothWeight) * (prevX0[1] - currX0[1]);
                residuals[3] = T(_smoothWeight) * (prevX0[2] - currX0[2]);

                return true;
            }

        protected:
            const _Matrix3  _intrinsics;
            const double    _weight, _smoothWeight;
    }; //...EqualDistanceCostFunctor

    /** \brief Ties a hand-aligned cuboid edge distance to the distances of two tracks at a time point.
      *  \tparam _Matrix3 3x3 matrix to store the intrinsics. Concept: Eigen::Matrix3f.
      */
    template <typename _Matrix3>
    struct FixedDistanceCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            FixedDistanceCostFunctor( const _Matrix3& intrinsics, const double distance, const double weight )
                : _intrinsics( intrinsics ), _distance(distance), _weight( weight )
            {} //...CostFunctor()

            template <typename T> bool operator() ( const T* curr0, const T* curr1, T* residuals ) const
            {
                using ceres::length;

                T currX0[2], currX1[2];
                getXy( curr0, currX0, _intrinsics );
                getXy( curr1, currX1, _intrinsics );

                T currLen = length( currX0[0] - currX1[0], currX0[1] - currX1[1], curr0[2] - curr1[2] );

                residuals[0] = currLen - T(_distance);

                return true;
            }

        protected:
            const Scalar    _distance; //!< prescribed
            const _Matrix3  _intrinsics;
            const double    _weight;
    }; //...FixedDistanceCostFunctor

    /** \brief Makes sure, two cuboid edges remain parallel or perpendicular over time. Uses 4 points to describe the 2 edges. */
    template <typename _Matrix3>
    struct DotProductCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            /**
             * \param[in] targetDotProduct ==0.0 for perpendicular, ==1.0 for parallel constraint.
            */
            DotProductCostFunctor( const _Matrix3& intrinsics, const double targetDotProduct, const double weight )
                : _intrinsics( intrinsics ), _targetDotProduct(targetDotProduct), _weight( weight )
            {} //...CostFunctor()

            /**
             * \param[in] p0 u,v, and z coordinates of edge0 start vertex.
             * \param[in] p1 u,v, and z coordinates of edge0 end vertex.
             * \param[in] q0 u,v, and z coordinates of edge1 start vertex.
             * \param[in] q1 u,v, and z coordinates of edge1 end vertex.
             */
            template <typename T> bool operator() ( const T* p0, const T* p1, const T* q0, const T* q1, T* residuals ) const
            {
                // get 3D x and y coordinates:
                T P0[2], P1[2], Q0[2], Q1[2];
                getXy( p0, P0, _intrinsics );
                getXy( q0, Q0, _intrinsics );
                getXy( p1, P1, _intrinsics );
                getXy( q1, Q1, _intrinsics );

                residuals[0] =  (P1[0] - P0[0]) * (Q1[0] - Q0[0]) + // e0_x * e1_x +
                                (P1[1] - P0[1]) * (Q1[1] - Q0[1]) + // e0_y * e1_y +
                                (p1[2] - p0[2]) * (q1[2] - q0[2]) - // e0_z * e1_z
                                T(_targetDotProduct);               // = targetDotProduct (1: parallel, 0: perpendicular)

                return true;
            } //...operator()

        protected:
            const _Matrix3  _intrinsics;
            const Scalar    _targetDotProduct; //!< prescribed
            const double    _weight;
    }; //...DotProductCostFunctor

    /** \brief Makes sure, two cuboid edges remain perpendicular over time. Uses 3 points to describe the 2 edges.
     *  \tparam _Matrix3 3x3 matrix to store the intrinsics. Concept: Eigen::Matrix3f.
     */
    template <typename _Matrix3>
    struct DotProductFrom3PointsCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            /**
             * \param[in] targetDotProduct ==0.0 for perpendicular, ==1.0 for parallel constraint.
            */
            DotProductFrom3PointsCostFunctor( const _Matrix3& intrinsics, const double targetDotProduct, const double weight )
                : _intrinsics( intrinsics ), _targetDotProduct(targetDotProduct), _weight( weight )
            {} //...CostFunctor()

            /**
             * \param[in] p0 u,v, and z coordinates of edge0 start vertex.
             * \param[in] p1 u,v, and z coordinates of edge0 end vertex.
             * \param[in] q0 u,v, and z coordinates of edge1 start vertex.
             * \param[in] q1 u,v, and z coordinates of edge1 end vertex.
             */
            template <typename T> bool operator() ( const T* p0, const T* p1, const T* q1, T* residuals ) const
            {
                // get 3D x and y coordinates:
                T P0[2], P1[2], Q1[2];
                getXy( p0, P0, _intrinsics );
                getXy( p1, P1, _intrinsics );
                getXy( q1, Q1, _intrinsics );

                residuals[0] =  (P1[0] - P0[0]) * (Q1[0] - P0[0]) + // e0_x * e1_x +
                                (P1[1] - P0[1]) * (Q1[1] - P0[1]) + // e0_y * e1_y +
                                (p1[2] - p0[2]) * (q1[2] - p0[2]) - // e0_z * e1_z
                                T(_targetDotProduct);               // = targetDotProduct (1: parallel, 0: perpendicular)

                return true;
            } //...operator()

        protected:
            const _Matrix3  _intrinsics;
            const Scalar    _targetDotProduct; //!< prescribed
            const double    _weight;
    }; //...DotProductCostFunctor

    // TODO: actually use
    template <typename _Matrix3, typename _Vector3>
    struct CloseTo3DObservationCostFunctor
    {
            typedef typename _Matrix3::Scalar Scalar;

            CloseTo3DObservationCostFunctor( const _Matrix3& intrinsics, const _Vector3& x, const int height, const double weight )
                : _intrinsics( intrinsics ), _x( x ), _height( height ), _weight( weight )
            {} //...CostFunctor()

            template <typename T> bool operator() ( const T* uvz, T* residuals ) const
            {
                // x,y,uvz[2]: 3D coordinates
                //         _x: 3D observation
                T x,y;
                getXy( uvz, x, y, _intrinsics, _height );

                residuals[0] = T(_weight) * (T(_x(0)) - x);
                residuals[1] = T(_weight) * (T(_x(1)) - y);
                residuals[2] = T(_weight) * (T(_x(2)) - uvz[2]);

//                std::cout << "3d: " << _x(0) << "," << _x(1) << "," << _x(2)
//                          << " - " << printJet(x) << "," << printJet(y) << "," << printJet(uvz[2])
//                          << " = " << printJet(residuals[0]) << "," << printJet(residuals[1]) << "," << printJet(residuals[2]) << std::endl;

                return true;
            } //...operator()

        protected:
            const _Matrix3  _intrinsics;        //<! Camera intrinsics matrix
            const _Vector3  _x;                 //<! 3D observation to be close to
            const double    _height;            //<! Depth image width and height
            const double    _weight;            //<! Term scale

    }; //...CloseTo3DObservationCostFunctor

    /** \brief Keeps track of mapping between unknowns and variables for \ref Fitter. */
    class Indexer
    {
        public:
            static constexpr int UVZS_STRIDE = 3; // u_t, v_t, z_t

            Indexer( FrameId startFrame, TrackId trackCount ) : _startFrame( startFrame ), _trackCount( trackCount ) {}

            /** \brief Returns the linear ID k of a point in a track at a time frameId. */
            inline size_t getPId( const tracking::FrameId frameId, const tracking::TrackId trackId )
            {
                return (frameId - _startFrame) * _trackCount + trackId;
            }

            /** \brief Returns the unknownId of the k-th unknown uvz_k. */
            inline size_t getUId( const tracking::FrameId frameId, const tracking::TrackId trackId )
            {
                return getPId(frameId, trackId) * UVZS_STRIDE;
            }

            /** \brief Returns the starting address of the k-th unknown uvz_k. */
            template <typename _UnknownsT>
            inline typename _UnknownsT::value_type* getUnknownAddress( _UnknownsT& unknowns, const tracking::FrameId frameId, const tracking::TrackId trackId )
            {
                return &( unknowns.at(getUId(frameId,trackId)) );
            }
        protected:
            FrameId _startFrame;
            TrackId _trackCount;
    };

    /** \brief Fits a 3D cuboid to 8 2D tracks. */
    class Fitter
    {
            DEFINE_EXCEPTION(Fitter_NoKnownCuboidPoseGiven)
            DEFINE_EXCEPTION(Fitter_TrackToCornerAssignmentIsNotUnique)

        public:
            typedef tracking::FrameId                                   FrameId;
            typedef double                                              CeresScalar;
            typedef tracking::Mapper::Vector3                           Vector3;
            typedef tracking::Mapper::Matrix3                           Matrix3;
            typedef Eigen::Map<       Eigen::Matrix< CeresScalar,3,1> > MapCeresVector3;
            typedef Eigen::Map< const Eigen::Matrix< CeresScalar,3,1> > MapConstCeresVector3;
            typedef OnTrackCostFunctor              < Matrix3 >         OnTrackFunctorT;
            typedef EqualDistanceCostFunctor        < Matrix3 >         EqualDistanceFunctorT;
            typedef FixedDistanceCostFunctor        < Matrix3 >         FixedDistanceFunctorT;
            typedef DotProductCostFunctor           < Matrix3 >         DotProductFunctorT;
            typedef DotProductFrom3PointsCostFunctor< Matrix3 >         DotProductFrom3PointsFunctorT;

            //typedef std::map< FrameId, Correspondence::TransformationT> TransformsT;


            /** \brief Optimization weights for \ref Fitter 's cuboid fitting. */
            struct Weights
            {
                    Weights() : uvWeight(4.), rigidWeight(2.), paralWeight(1.), perpWeight(2.), fixedWeight(0.1) {}
                    double uvWeight;    //!< fidelity to 2D observations
                    double rigidWeight; //!< keep point distances in consequtive frames
                    double paralWeight; //!< Cuboid edges should be parallel
                    double perpWeight;  //!< Cuboid edges should be perpendicular
                    double fixedWeight; //!< Cuboid edges should be exactly as long as the input cuboid's lengths

            };

            static int fitCuboid(       Tracks3D &tracks3d,
                                        TransformsT& outTransforms,
                                  const TransformsT& inTransforms,
                                  const Tracks2D &tracks2D,
                                  const Mapper   &mapper,
                                  const Weights  &weights,
                                  const int       width,
                                  const int       height,
                                  const FrameId   startFrame,
                                  const FrameId   endFrame, // inclusive!
                                  const std::vector< ::Eigen::Vector3f >& colors,
                                  const std::vector<Cuboid>* cuboid = nullptr,
                                        std::vector<Cuboid>* outCuboids = nullptr
                                );
    }; //...Fitter
} //...ns tracking

#endif // TV_FITCUBOID_H


