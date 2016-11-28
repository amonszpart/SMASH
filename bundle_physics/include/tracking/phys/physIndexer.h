//
// Created by bontius on 05/12/15.
//

#ifndef TRACKVIDEO_PHYSINDEXER_H
#define TRACKVIDEO_PHYSINDEXER_H

#include "tracking/phys/typedefs.h"
#include "tracking/phys/typdefsTermTypes.h"
#include "tracking/common/typedefs.h"
#include "tracking/common/util/exception.h"
#include "ceres/ceres.h"
#include "ceres/ceresUtil.h"
#include <vector>

namespace tracking {
  namespace bundle_physics {
    /** Class taking care of the memory management of the optimization problem. */
    class PhysIndexer {
        protected:
            enum { FREE_ANGLE_ALIAS = 0, ALIAS_LAST = FREE_ANGLE_ALIAS};
        public:
            DEFINE_EXCEPTION( PhysIndexer_NotInited )
            DEFINE_EXCEPTION( PhysIndexer_AlreadyInited )
            DEFINE_EXCEPTION( PhysIndexer_addPoint_TrackIdAlreadyAdded )
            DEFINE_EXCEPTION( PhysIndexer_getPointAddress_TrackIdUnknown )
            DEFINE_EXCEPTION( PhysIndexer_NeedTwoFrameIds )
            DEFINE_EXCEPTION( PhysIndexer_GetParabolaTransform_Overindexing )
            DEFINE_EXCEPTION( PhysIndexer_GetParabolaAbs_Overindexing )
            DEFINE_EXCEPTION( PhysIndexer_GetMomentum_Overindexing )
            DEFINE_EXCEPTION( PhysIndexer_GetCollisionTime_Overindexing )
            DEFINE_EXCEPTION( PhysIndexer_GetMass_Overindexing )
            DEFINE_EXCEPTION( PhysIndexer_CollIdNegative )

            typedef ceres::CeresScalar CeresScalar; //!< Ceres uses double, whilst Scalar usually is float, so we need a separate typename.

            // trajectory:
            static constexpr int PARABOLA_SHARED_ANGLES_STRIDE     = 2; //!< 2 angles
            static constexpr int PARABOLA_SHARED_A_STRIDE          = 1; //!< "a"
            static constexpr int PARABOLA_SHARED_STRIDE            = PARABOLA_SHARED_ANGLES_STRIDE + PARABOLA_SHARED_A_STRIDE; //!< 2 angles and "a"
            static constexpr int PARABOLA_SHARED_ANGLES_OFFSET     = 0; //!< 2 angles, and "a"
            static constexpr int PARABOLA_SHARED_A_OFFSET          = 2; //!< 2 angles, and "a"

            static constexpr int PARABOLA_FREE_STRIDE              = 3; //!< 1 angle, 2 for "b" and "s".
            static constexpr int PARABOLA_FREE_ANGLE_OFFSET        = 0; //!< 1 angle, 2 for "b" and "s"
            static constexpr int PARABOLA_FREE_B_OFFSET            = 1; //!< 1 angle, 2 for "b" and "s"
            static constexpr int PARABOLA_FREE_S_OFFSET            = PARABOLA_FREE_B_OFFSET + 1; //!< 1 angle, 2 for "b" and "s"

            static constexpr int PARABOLA_TRANSLATION_STRIDE       = 3; //!< 3 for translation
            static constexpr int COLL_TIME_STRIDE                  = 1; //!< Floating point time representation of collision time
            static constexpr int IMPULSE_STRIDE                    = 3; //!< 3 coordinates for collision impulse \f$ J \mathbf{n} = \mathbf{m} \f$
            static constexpr int MASS_STRIDE                       = 1; //!< 1 Scalar for object relative mass \f$ M_b \f$
            static constexpr int COLL_POINTS_STRIDE                = 3; //!< \f$ x,y,z \f$ for relative collision vectors points

            static constexpr int POSE_STRIDE        = 4; //!< Quaternion variables \f$ w,x,y,z \f$ for poses.
            static constexpr int MOMENTUM_STRIDE    = 3; //!< 3 values for angular momentum \f$ \mathbf{L} \f$
            // optical flow:
            static constexpr int POINTS_STRIDE      = 3; //!< \f$ x,y,z \f$ for points


            PhysIndexer(FrameIdsT const frameIds = {}, size_t const nCuboids = 0)
                : _startFrameId( frameIds.front() ),
                  _sequenceLength( frameIds.back() - frameIds.front() + 1), // one intermediate variable for each transition
                  _nParts( frameIds.size() - 1 ),
                  _nCuboids(nCuboids),
                  _nParabolas ( _nParts * _nCuboids ),
                  _nCollisions( _nParts - 1 ),
                  _inited( false ),
                  _aliases(ALIAS_LAST + 1)
            {
                if ( frameIds.size() < 2 )
                    throw new PhysIndexer_NeedTwoFrameIdsException("Need at least two, start and end (inclusive, both)");
            }

            /** \brief Estimate, how much storage is needed, and allocate. This should only be called once.
             */
            inline void allocate()
            {
                if ( _inited )
                    throw new PhysIndexer_AlreadyInitedException("Cannot call allocate twice");

                _sharedAnglesAndA       .resize( 1            * PARABOLA_SHARED_STRIDE    );
                _freeAngleAndLinParams  .resize( _nParabolas  * PARABOLA_FREE_STRIDE      );
                _parabolaTranslations   .resize( size_t(_nCollisions) * _nCuboids * PARABOLA_TRANSLATION_STRIDE);
                _collTimes              .resize( size_t(_nCollisions) * COLL_TIME_STRIDE);
                _momenta                .resize( _nParabolas  * MOMENTUM_STRIDE );
                _poses                  .resize( static_cast<unsigned long>(_nCollisions) * _nCuboids * POSE_STRIDE );
                _points                 .resize( _linPointIds.size() * POINTS_STRIDE );
                _collPoints             .resize( static_cast<unsigned long>(_nCollisions) * COLL_POINTS_STRIDE);
                _masses                 .resize( _nCuboids    * MASS_STRIDE );
                _impulses               .resize( static_cast<unsigned long>(_nCollisions) * IMPULSE_STRIDE );
                _inited = true;
            }

            /// ====================================================== ///
            /// ====================== Parabola ====================== ///
            /// ====================================================== ///

            /** \brief Returns the address of the part of the rotation, that is specific for each parabola (i.e. the first angle around Y in the YXY Euler angle convention).
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            CeresScalar const* getParabolaFreeParamsConst( const CuboidId cuboidId, const PartId partId ) const;
            CeresScalar      * getParabolaFreeParams( const int cuboidId, const int partId );

            /** \brief Returns the address the parabola specific translation.
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getParabolaTranslationConst( const CuboidId cuboidId, const CollId collId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( collId >= _nCollisions )
                    throw new PhysIndexer_GetParabolaTransform_OverindexingException("We don't have this many parabolas for this cuboid");
                return &( _parabolaTranslations.at((cuboidId * _nCollisions + collId) * PARABOLA_TRANSLATION_STRIDE) );
            }
            inline CeresScalar* getParabolaTranslation(const CuboidId cuboidId, const CollId collId)
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaTranslationConst(cuboidId,collId)); }

            CeresScalar const* getParabolaRotationFreeConst(const CuboidId cuboidId, const PartId partId) const;
            CeresScalar      * getParabolaRotationFree(const CuboidId cuboidId, const PartId partId);

            inline const CeresScalar* getParabolaRotationSharedConst() const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                return getParabolaSharedParamsConst() + PARABOLA_SHARED_ANGLES_OFFSET;
            }
            inline CeresScalar* getParabolaRotationShared()
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaRotationSharedConst()); }

            /** \brief Returns the address the squared parabola parameter "a" in \f$ [ st, a^2 t + bt, 0 ] \f$. Since gravity is constant, this paramter is shared.
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getParabolaSquaredParamConst() const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");

                return &( _sharedAnglesAndA.at(PARABOLA_SHARED_A_OFFSET) );
            }
            inline CeresScalar* getParabolaSquaredParam()
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaSquaredParamConst()); }


            /** \brief Returns the address the linear parabola parameters "s" and "b" in \f$ [ st, a^2 t + bt, 0 ] \f$
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getParabolaBParamConst(const CuboidId cuboidId, const PartId partId) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( partId >= _nParabolas / _nCuboids )
                    throw new PhysIndexer_GetParabolaAbs_OverindexingException("We don't have this many parabolas for this cuboid");

                return  getParabolaFreeParamsConst(cuboidId,partId) + PARABOLA_FREE_B_OFFSET;
            }
            inline CeresScalar* getParabolaBParam(const CuboidId cuboidId, const PartId partId)
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaBParamConst(cuboidId,partId)); }

            /** \brief Returns the address the linear parabola parameters "s" and "b" in \f$ [ st, a^2 t + bt, 0 ] \f$
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getParabolaSParamConst(const CuboidId cuboidId, const PartId partId) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( partId >= _nParabolas / _nCuboids )
                    throw new PhysIndexer_GetParabolaAbs_OverindexingException("We don't have this many parabolas for this cuboid");

                return getParabolaFreeParamsConst(cuboidId,partId) + PARABOLA_FREE_S_OFFSET;
            }
            inline CeresScalar* getParabolaSParam(const CuboidId cuboidId, const PartId partId)
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaSParamConst(cuboidId,partId)); }

            /// ====================================================== ///
            /// ======================== Pose ======================== ///
            /// ====================================================== ///
#if 0
            /** \brief Gives you the address to the pose of the object at a certain time \p frameId.
             */
            inline CeresScalar* getPose( const FrameId frameId, const int cuboidId /*, FrameId integralSteps = 0 */)
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");

                return &( _poses.at((cuboidId * _sequenceLength + (frameId -_startFrameId)) * POSE_STRIDE) );
            }
#endif

            /** \brief Gives you the address to the pose of the object at a certain collision.
              */
            inline const CeresScalar* getCollisionPoseConst( const CuboidId cuboidId, const CollId collId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( collId < 0 )
                    throw new PhysIndexer_CollIdNegativeException("");

                return &( _poses.at( (cuboidId * size_t(_nCollisions) + size_t(collId)) * POSE_STRIDE) );
            }
            inline CeresScalar* getCollisionPose( const CuboidId cuboidId, const CollId collId )
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getCollisionPoseConst(cuboidId,collId)); }

            inline const CeresScalar* getCollisionPointConst( const CollId collId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");

                return &( _collPoints.at((size_t(_nCollisions) * size_t(collId)) * COLL_POINTS_STRIDE) );
            }
            inline CeresScalar* getCollisionPoint( const CollId collId )
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getCollisionPointConst(collId)); }

            /** \brief Returns address of angular momentum, that is constant along the parabola of this cuboid and this part.
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getMomentumConst( const CuboidId cuboidId, const PartId partId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( partId >= _nParabolas / _nCuboids )
                    throw new PhysIndexer_GetMomentum_OverindexingException("We don't have this many parabolas for this cuboid");
                return &( _momenta.at((cuboidId * _nParts + partId) * MOMENTUM_STRIDE) );
            }
            inline CeresScalar* getMomentum( const CuboidId cuboidId, const PartId partId )
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getMomentumConst(cuboidId,partId)); }

            /** \param[in] linId Linear id of collision in time. It denotes the collision between parts \p linId and \p linId+1.
             */
            inline const CeresScalar* getCollisionTimeConst( const CollId collId) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( int(collId) >= _nParabolas / _nCuboids - 1 )
                {
                    std::cout << int(collId) << " > " << _nParabolas << "/" << _nCuboids << " - 1 = " << _nParabolas / _nCuboids - 1 << std::endl;
                    throw new PhysIndexer_GetCollisionTime_OverindexingException("We don't have this many collisions");
                }
                return &( _collTimes.at(collId) );
            }
            inline CeresScalar* getCollisionTime( const CollId collId)
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getCollisionTimeConst(collId)); }

            /// ====================================================== ///
            /// ======================= Points ======================= ///
            /// ====================================================== ///

            /** \brief Register a 2D track (a single 3D point) for optimization. Call before allocate.
             */
            inline void addPoint( const TrackId trackId )
            {
                if ( _inited )
                    throw new PhysIndexer_AlreadyInitedException("");

                if ( _linPointIds.find(trackId) != _linPointIds.end() )
                    throw new PhysIndexer_addPoint_TrackIdAlreadyAddedException("");

                _linPointIds.insert( std::make_pair(trackId,_linPointIds.size()) );
            }

            inline const CeresScalar* getPointAddressConst( const TrackId trackId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");

                auto it = _linPointIds.find( trackId );
                if ( it == _linPointIds.end() )
                    throw new PhysIndexer_getPointAddress_TrackIdUnknownException("");
                return &( _points.at( it->second * POINTS_STRIDE) );
            }
            inline CeresScalar* getPointAddress( const TrackId trackId )
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getPointAddressConst(trackId)); }

            std::map<TrackId,LinId> const& getLinPointIds() const { return _linPointIds; }

            /// ====================================================== ///
            /// ==================== Collisions ====================== ///
            /// ====================================================== ///

            inline const CeresScalar* getMassConst( const CuboidId cuboidId, bool *fixed = nullptr ) const {
                if ( fixed ) {
                    if (cuboidId == 0)
                        *fixed = true;
                    else
                        *fixed = false;
                }
                if ( cuboidId >= _nCuboids )
                    throw new PhysIndexer_GetMass_OverindexingException("");
                return &( _masses.at(cuboidId * MASS_STRIDE) );
            }
            inline CeresScalar* getMass( const CuboidId cuboidId, bool *fixed = nullptr )
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getMassConst(cuboidId,fixed)); }

            inline const CeresScalar* getImpulseConst( const CollId collId ) const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                if ( collId >= _nParabolas / _nCuboids - 1 )
                {
                    std::cout << size_t(collId) << " > " << _nParabolas << "/" << _nCuboids << " - 1 = " << _nParabolas / _nCuboids - 1 << std::endl;
                    throw new PhysIndexer_GetCollisionTime_OverindexingException("We don't have this many collisions");
                }
                return &( _impulses.at(int(collId) * IMPULSE_STRIDE) );
            }
            inline CeresScalar* getImpulse( const CollId collId )
            { return  const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getImpulseConst(collId)); }

            struct WrapperInfo{
                ceres::LossFunctionWrapper* lossWrapper;
                CeresScalar weight;
                size_t termsCount;
            };
            void setWrapperInfo(TERM_TYPE const termType, WrapperInfo wrapperInfo) { _wrapperInfos[termType] = {wrapperInfo}; }
            WrapperInfo const& getWrapperInfo(TERM_TYPE const termType, int id = 0) const { return _wrapperInfos.at(termType).at(id); }
            WrapperInfo      & getWrapperInfo(TERM_TYPE const termType, int id = 0)       { return _wrapperInfos.at(termType).at(id); }
        protected:
            const FrameId                _startFrameId,             //!< Starting frame Id to calculate linear index of actual frameIds: linFrameId = frameId - _fwFrameId.
                                         _sequenceLength;           //!< Sum of frames solved for.
            const PartId                 _nParts;                   //!< How many parabolas per cuboid. Usually |collisions|+1.
            const CuboidId               _nCuboids;                 //!< How many cuboids we are optimizing simultaneously.
            const PartId                 _nParabolas;               //!< How many different parabolas we are fitting. Usually _nParts * _nCuboids.
            const CollId                 _nCollisions;              //!< How many collisions or transitions (_nParts - 1 ).
            bool                         _inited;                   //!< State flag, to see, whether data has been allocated.
            std::vector<CeresScalar>     _sharedAnglesAndA;         //!< 2+1 variables: two parabola rotation angles and the parameter "a".
            std::vector<CeresScalar>     _freeAngleAndLinParams;    //!< 1+3+2 variables: one free angle and 3 variables for the parabola translation, 2 for "b" and "s".
            std::vector<CeresScalar>     _parabolaTranslations;
            std::vector<CeresScalar>     _collTimes;                //!< Floating exact point value for collision time points.
            //std::vector<CeresScalar>     _bss;                    //!< "b" and "s" for the parabola shape \f$ [ s t, a t^2 + b t, 0] = 0 \f$
            std::vector<CeresScalar>     _momenta;                  //!< \f$ \mathbf{L} = x,y,z \f$ for object angular momentum
            std::vector<CeresScalar>     _poses;                    //!< Stores quaternions for each cuboid and timepoint.
            std::map   <TrackId,LinId>   _linPointIds;              //!< _linPointIds[ TrackId ] = linear id in _points.
            std::vector<CeresScalar>     _points;                   //!< \f$ x,y,z \f$ for each point
            std::vector<CeresScalar>     _collPoints;               //!< \f$ x,y,z \f$ for each relative collision point

            std::vector<CeresScalar>     _masses,
                                         _impulses;
            std::map< TERM_TYPE,std::vector<WrapperInfo> > _wrapperInfos;

            std::vector< std::map<std::pair<CuboidId,PartId>, CeresScalar* > > _aliases;

            /** \brief Returns the address the part of the rotation, that is shared between parabolas (i.e. the last two angles around X and Y in the YXY Euler angle convention).
             *  \param[in] cuboidId Which cuboid.
             *  \param[in] partId   LinIndex of parabola among parabolas of that cuboid.
             */
            inline const CeresScalar* getParabolaSharedParamsConst() const
            {
                if ( !_inited )
                    throw new PhysIndexer_NotInitedException("Please call allocate()");
                return &( _sharedAnglesAndA.at(0) );
            }
            inline CeresScalar* getParabolaSharedParams()
            { return const_cast<CeresScalar*>(static_cast<PhysIndexer&>(*this).getParabolaSharedParamsConst()); }
    }; //...class PhysIndexer
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYSINDEXER_H
