#ifndef TV_CUBOID_H
#define TV_CUBOID_H

#include "tracking/phys/inertiaEstimation.hpp"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/mapper.h"
#include "tracking/common/maths/impl/quaternion.hpp"
#include "ceres/ceresUtil.h"
#include <memory>
#include <vector>

namespace tracking {
  namespace bundle_physics {
    struct PoseLoc {
            using Vector3 = tracking::bundle_physics::Vector3;
            using Scalar = tracking::bundle_physics::Scalar;
            PoseLoc()
                : _center(Vector3::Zero()), _pose(QuaternionT(0., 0., 0., 0.)), _linVel({0., 0., 0.}), _angVel({0., 0., 0.}),
                 _hasPos(false), _hasPose(false), _hasLinVel(false), _hasAngVel(false)
            {}

            PoseLoc(const Vector3 &center)
                : _center(center), _pose(QuaternionT(0., 0., 0., 0.)),_linVel({0., 0., 0.}), _angVel({0., 0., 0.}),
                _hasPos(true), _hasPose(false), _hasLinVel(false), _hasAngVel(false)
            {}

            PoseLoc(const Vector3 &center, const QuaternionT &pose/* = QuaternionT(1., 0., 0., 0.)*/)
                : _center(center), _pose(pose), _linVel({0., 0., 0.}), _angVel({0., 0., 0.}),
                _hasPos(true), _hasPose(true), _hasLinVel(false), _hasAngVel(false)
            {}


            inline Vector3 const& getPosition() const { if (!_hasPos) std::cerr << "[" << __func__ << "] " << "getting pos, but don't have one..." << std::endl; return _center; }
//            inline Vector3      & getPosition()       { if (!_hasPos) std::cerr << "[" << __func__ << "] " << "getting pos, but don't have one..." << std::endl; return _center; }
            inline void           setPosition(const Vector3 &pos)
            { _hasPos = true; _center = pos; }
            inline bool hasPos() const
            { return _hasPos; }


            inline QuaternionT const& getPose() const
            { if (!_hasPose) { std::cerr << "[" << __func__ << "] " << "warning...don't have pose, but returning..." << std::endl;
                                 throw new std::runtime_error(""); }
                return _pose;
            }
            inline void setPose(const QuaternionT &pose)
            { _pose = pose; _hasPose = true; }
            inline bool hasPose() const
            { return _hasPose; }


            inline Vector3 const& getLinVel() const
            { if (!_hasLinVel) std::cerr << "[" << __func__ << "] " << "getting linVel, but don't have one..." << std::endl; return _linVel; }
            inline void setLinVel(const Vector3 &linVel)
            { _linVel = linVel; _hasLinVel = true; }
            inline bool hasLinVel() const
            { return _hasLinVel; }


            inline const Vector3 &getAngVel() const
            { if (!_hasAngVel) std::cerr << "[" << __func__ << "] " << "getting angVel, but don't have one..." << std::endl; return _angVel; }
            inline void setAngVel(const Vector3 &angVel)
            { _angVel = angVel; _hasAngVel = true; }
            inline bool hasAngVel() const
            { return _hasAngVel; }


            inline TransformationT getTransform() const {
                return Eigen::Translation<Scalar, 3>(this->getPosition())
                       * TransformationT(this->getPose());
            }

        protected:
            Vector3     _center;
            QuaternionT _pose;
            Vector3     _linVel;
            Vector3     _angVel;
            bool        _hasPos, _hasPose, _hasLinVel, _hasAngVel;
    }; //...PoseLoc

    struct Cuboid {
            DEFINE_EXCEPTION(Cuboid_MatchTrackIds_LogicError)
            using Vector3 = PoseLoc::Vector3;
            using Scalar = PoseLoc::Scalar;
            //typedef PoseLoc::Scalar             Scalar;
            //typedef tracking::FrameId           FrameId;
            //typedef PoseLoc::Vector3            Vector3;           //!< \copydoc PoseLoc::Vector3
            //typedef PoseLoc::QuaternionT        QuaternionT;
            typedef Eigen::Matrix4f             Transform4x4;
            //typedef PoseLoc::TransformationT    TransformationT;
            //typedef PoseLoc::TranslationT       TranslationT;
            typedef Eigen::Matrix<Scalar, 4, 8> CornersT;          //!< homogenous corners in columns
            typedef std::unique_ptr<CornersT>   CornersPtrT;
            typedef std::vector<int>            EdgesT;
            typedef std::map<FrameId, PoseLoc>  StatesT;

            void                                init();

//            enum SHAPE { BOX, SPHERE, ELLIPSOID, BOX_HOLLOW };

            Cuboid() { init(); }

            Cuboid(const Vector3 &size)
                : _size(size) {
                init();
            }

            static inline const CornersPtrT &getCorners() { return _corners; }

            static inline const EdgesT &getEdges() { return _edges; }

            static inline const std::vector<EdgesT> &getParallelEdges() { return _parallelEdges; }

            inline bool hasFrame(const FrameId frameId) const { return _states.find(frameId) != _states.end(); }

            inline const PoseLoc &getState(const FrameId frameId) const { return _states.at(frameId); }

            inline const Vector3 &getPosition(const FrameId frameId) const { return _states.at(frameId).getPosition(); }

            inline const QuaternionT &getPose(const FrameId frameId) const { return _states.at(frameId).getPose(); }
            inline bool hasPose(FrameId const frameId) const { if (_states.find(frameId) == _states.end()) return false; return _states.at(frameId).hasPose(); }

            inline void setPosition(const FrameId frameId, const Vector3 &pos) { _states[frameId].setPosition(pos); }

            inline void setPose(const FrameId frameId, const QuaternionT &pose) { _states[frameId].setPose(pose); }

            inline void setPose(const FrameId frameId, const Eigen::Matrix<Scalar, 3, 3> &pose) { _states[frameId].setPose(QuaternionT(pose)); }

            inline const Vector3 &getSize() const { return _size; }

            inline Vector3 &getSize() { return _size; }

            inline void addState(const FrameId &frameId, const PoseLoc &state) { _states[frameId] = state; }

            inline void setLinVel(const FrameId frameId, const Vector3 &linVel) { _states[frameId].setLinVel(linVel); }

            inline void setAngVel(const FrameId frameId, const Vector3 &angVel) { _states[frameId].setAngVel(angVel); }

            inline const Vector3 &getLinVel(const FrameId frameId) const { return _states.at(frameId).getLinVel(); }

            //inline       Vector3&     getLinVel( const FrameId frameId ) { return _states.at(frameId).getLinVel(); }
            inline const Vector3 &getAngVel(const FrameId frameId) const { return _states.at(frameId).getAngVel(); }
            //inline       Vector3&     getAngVel( const FrameId frameId ) { return _states.at(frameId).getAngVel(); }

            Transform4x4    getTransform4x4(const FrameId &frameId) const;

            TransformationT getTransformation(const FrameId &frameId) const;

            inline TransformationT getSizeTransform() const { return TransformationT(this->getSize().asDiagonal()); } //...getSizeTransform()

            inline void rotate(const FrameId &frameId, const Eigen::Matrix<Scalar, 3, 3> &rot) {
                if (!hasFrame(frameId)) {
                    std::cerr << "[" << __func__ << "]: " << "does not have frame, cannot  rotate" << std::endl;
                    return;
                }
                //std::cout << "pose was " << _states[ frameId ].getPose().toRotationMatrix() << "\n";
                _states[frameId].setPose(QuaternionT(rot) * _states[frameId].getPose());
                //std::cout << ", is now " << _states[ frameId ].getPose().toRotationMatrix() << std::endl;
            }

            inline void translate(const FrameId &frameId, const Eigen::Matrix<Scalar, 3, 1> &t) {
                if (!hasFrame(frameId)) {
                    std::cerr << "[" << __func__ << "]: " << "does not have frame, cannot translate" << std::endl;
                    return;
                }
                _states[frameId].setPosition(_states[frameId].getPosition() + t);
            }

            inline void scale(const Vector3 &s) {
                this->_size = s.asDiagonal() * this->_size;
            }

            inline const StatesT &getStates() const { return _states; }

            inline StatesT::iterator removeState(const StatesT::const_iterator it) { return _states.erase(it); }

            bool            matchTrackIds(std::vector<int> &trackIds, const std::vector<Eigen::Vector2f> &corners2D, const Tracks2D &tracks2d, const FrameId frameId) const;

            inline std::string getName() const { return _name; }

            inline void setName(const std::string name) { _name = name; }

            inline void clearStates() { _states.clear(); }

            inline unsigned getObjId() const { return _objId; }

            inline void setObjId(const unsigned objId) { _objId = objId; }

            template<typename _Scalar>
            void getIFromMass(Eigen::Matrix<_Scalar, 3, 1> &I, const Scalar mass) const;

            template<typename _Scalar>
            void getInverseIFromMass(Eigen::Matrix<_Scalar, 3, 1> &I, const Scalar mass) const;

            Scalar getMass() const;

            void   setMass(Scalar mass);

            bool   isMassFinite() const;

            inline const SHAPE &getShape() const { return _shape; }

            inline void setShape(const SHAPE &shape) { _shape = shape; }

        protected:
            Vector3                          _size;
            StatesT                          _states;
            std::string                      _name;
            static const EdgesT              _edges;
            static const std::vector<EdgesT> _parallelEdges;
            static CornersPtrT               _corners;
            unsigned                         _objId;
            Scalar                           _mass;
            SHAPE                            _shape;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    template<typename _VisT, typename _Vector3, typename _TransformationT>
    void drawCuboid(_VisT &vis, const _TransformationT &T, const _Vector3 &color, const std::string &name, std::vector<_Vector3> *outCorners = nullptr);

    static inline void getCorners2D(std::vector<Eigen::Vector2f> &corners2D, TransformationT const& T, const tracking::Mapper &mapper) {
        corners2D.resize(8);
        for (int col = 0; col != Cuboid::getCorners()->cols(); ++col) {
            corners2D[col] = mapper.to2D((T * Cuboid::getCorners()->col(col)).template head<3>(), 0, false).template head<2>();
        }
    } //...getCorners2D()
  } //...ns bundle_physics
} //...ns tracking


namespace tracking {
  namespace bundle_physics {
    template <typename _Scalar>
    void Cuboid::getIFromMass( Eigen::Matrix<_Scalar,3,1>& I, const Scalar mass ) const
    {
        auto sizeSquared = this->getSize();
        sizeSquared.array() *= sizeSquared.array();

        ceres::CeresVector3 I2;
        ceres::CeresScalar  massd( mass );
        ceres::CeresVector3 sqrSized = sizeSquared.template cast<ceres::CeresScalar>();
        bundle_physics::getI( this->getShape(), &massd, sqrSized.data(), I2.data() );
        I = I2.cast<_Scalar>();

#if 0
        if ( this->getShape() == BOX )
        {
            ceres::CeresVector3 I2;
            ceres::CeresScalar massd( mass );
            ceres::CeresVector3 sqrSized = sizeSquared.template cast<ceres::CeresScalar>();
            bundle_physics::getIBox( &massd, sqrSized.data(), I2.data() );
            I = I2.cast<_Scalar>();
        }
        else
        {
            std::cerr << "todo: other shapes" << std::endl;

            auto sizeSquared = this->getSize();
            sizeSquared.array() *= sizeSquared.array();
            I = (Eigen::Matrix<_Scalar, 3, 1>() <<
                 mass * (sizeSquared(1) + sizeSquared(2)) / _Scalar(12.),
                mass * (sizeSquared(0) + sizeSquared(2)) / _Scalar(12.),
                mass * (sizeSquared(0) + sizeSquared(1)) / _Scalar(12.)).finished();
        }
#endif
    } //..getIFromMass

    template <typename _Scalar>
    void Cuboid::getInverseIFromMass( Eigen::Matrix<_Scalar,3,1>& I, const Scalar mass ) const
    {
        getIFromMass( I, mass );
        I(0) = _Scalar(1.) / I(0);
        I(1) = _Scalar(1.) / I(1);
        I(2) = _Scalar(1.) / I(2);
    }
  } //...ns bundle_physics
} //...ns tracking

#endif // TV_CUBOID_H

