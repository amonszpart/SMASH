#if 0

//
// Created by bontius on 12/12/15.
//

#ifndef TRACKVIDEO_TRACKINGLOGGER_H
#define TRACKVIDEO_TRACKINGLOGGER_H

#include "physacq/simulation/simulationLogger.hpp"
#include "tracking/annot/cuboid.h"

namespace tracking
{
    template <typename _Scalar>
    class TrackingLogger : public physacq::SimulationLogger<_Scalar>
    {
        public:
            typedef physacq::SimulationLogger<_Scalar> Base;
            TrackingLogger(const std::string outPath = "") : Base(outPath) {}
            using          Base::_actors;
            using typename Base::ObservationT;
            using typename Base::RigidBodyT;

            /**
             * @brief update
             * @param rbs
             * @return Number of collisions detected
             */
            int update()
            {
#if 0
                int collCount( 0 );
                for ( size_t i = 0; i != rbs.getRigidBodies().size(); ++i )
                {
                    physsim::RigidBody const &rb = rbs.getRigidBodies().at( i );
                    if ( _actors.find(i) == _actors.end() )
                    {
                        char name[255];
                        sprintf( name, "RigidBody%02lu", i );
                        physacq::RigidBodyFactory<_Scalar> factory;
                        _actors.insert( std::make_pair(i, factory.withId         ( i )
                                                                 .withMass       ( rb.getMass() )
                                                                 .withSize       ( rb.getSize().head<3>() )
                                                                 .withType       ( RigidBodyT::BOX )
                                                                 .withRestitution( rb.getElasticity() )
                                                                 .withInertia    ( rb.getInertiaTensor() )
                                                                 .withName       ( name )
                                                                 .create()
                        )
                        );
                    } // ...if new body
                    ObservationT obs( rb.getVelocity(), rb.getAngularVelocity() );
                    obs.setPose     ( rb.getRotation() );
                    obs.setPos      ( rb.getCenter().head<3>() );
                    obs.setTimestamp( _timestamp );
                    obs.setObjId    ( i );
                    obs.setName     ( "noname" );
                    if ( rb.getCollisions().size() )
                    {
                        if ( rb.getCollisions().size() > 1 )
                            std::cerr << "[" << __func__ << "]: more, than one collision info...are we prepared?" << std::endl;
                        const CollisionInfo& info = rb.getCollisions().at(0);
                        obs.setCollNormal( info.normalWorld.head<3>() );
                        obs.setCollPoint( info.collisionPointWorld.head<3>() );
                        obs.setImpulse  ( info.impulse ); // "J"
                        std::cerr << "[" << _timestamp << "] added collision: " << info.collisionPointWorld.transpose() << ", " << info.normalWorld.transpose() << std::endl;
                        ++collCount;
                    }
                    _actors.at(i).addObservation( _timestamp, obs );
                } //...for each body

                ++_timestamp;
                return collCount / 2;
#endif
            } //...update()

            void digest( const CuboidsConstPtrT& cuboids );
    }; //...cls TrackingLogger

} //...ns tracking

namespace tracking
{
    void digest( const CuboidsConstPtrT& cuboids )
    {}
//        for (auto const &cuboid : cuboids)
//        char name[255];
//        sprintf( name, "RigidBody%02lu", i );
//        physacq::RigidBodyFactory<_Scalar> factory;
//        _actors.insert( std::make_pair(i, factory.withId         ( i )
//                                                 .withMass       ( rb.getMass() )
//                                                 .withSize       ( rb.getSize().head<3>() )
//                                                 .withType       ( RigidBodyT::BOX )
//                                                 .withRestitution( rb.getElasticity() )
//                                                 .withInertia    ( rb.getInertiaTensor() )
//                                                 .withName       ( name )
//                                                 .create()
//        )
//        );
//
//        for ( FrameId frameId = frameIds.front(); frameId <= frameIds.back(); ++frameId )
//        {
//            FrameId         linId = frameId - frameIds.at(0);
//            int             cuboidId(0);
//            for (auto const &cuboid : cuboids)
//            {
//                if ( cuboid.hasFrame(frameId) )
//                {
//
//                }
//            }
//        }
//    } //...digest()
} //...ns tracking

#endif //TRACKVIDEO_TRACKINGLOGGER_H

#endif