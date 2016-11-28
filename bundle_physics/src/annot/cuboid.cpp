#include "tracking/annot/cuboid.h"
#include "tracking/annot/impl/cuboid.hpp"
#include "tracking/vis/visualizer.h"

//   6 -- 7
//  /    /|
// 2 -- 3 5
// |    |/
// 0 -- 1

const             tracking::bundle_physics::Cuboid::EdgesT  tracking::bundle_physics::Cuboid::_edges              = {  0,1, 2,3, 4,5, 6,7,    0,2, 1,3, 4,6, 5,7,     0,4, 1,5, 2,6, 3,7/*,   0,3,1,2, 1,7,3,5, 5,6,4,7, 2,4,0,6*/ };
const std::vector<tracking::bundle_physics::Cuboid::EdgesT> tracking::bundle_physics::Cuboid::_parallelEdges      = { {0,1, 2,3, 4,5, 6,7},  {0,2, 1,3, 4,6, 5,7},   {0,4, 1,5, 2,6, 3,7} };
std::unique_ptr<Eigen::Matrix<float,4,8> >  tracking::bundle_physics::Cuboid::_corners = nullptr;

namespace tracking {
  namespace bundle_physics {
    static Cuboid cuboid;

    template void drawCuboid(Soup::vis::Visualizer<Scalar> &vis, const TransformationT &T, const Vector3 &color, const std::string &name, std::vector<Vector3> *outCorners);

    template void drawCuboid(Soup::vis::Visualizer<Scalar> &vis, const Cuboid::Transform4x4 &T, const Vector3 &color, const std::string &name, std::vector<Vector3> *outCorners);

    /** \brief Selects a trackId for each of the 8 corners. Returns true, if unique matching */
    bool Cuboid::matchTrackIds(std::vector<int> &trackIds, const std::vector<Eigen::Vector2f> &corners2D, const Tracks2D &tracks2d, const FrameId frameId) const {
        trackIds.resize(8, -1);
        std::vector<float> dists(8, std::numeric_limits<float>::max());

        int   cornerId(0);
//        float diff;
        for (auto const &corner2D : corners2D) {
            float minDist(std::numeric_limits<float>::max());
            int   minId(-1);
            int   trackId(0);
            for (auto const &track : tracks2d) {
                if (track.hasPoint(frameId)) {
                    float diff;
                    if ((diff = (corner2D - track.getPoint(frameId)).norm()) < minDist) {
                        minDist = diff;
                        minId   = trackId;
                    } //...if smaller
                } //...if has frame

                // next track
                ++trackId;
            } //...for each track

            // store, if valid
            if ((minId >= 0) && ((trackIds[cornerId] == -1) || (dists[cornerId] > minDist))) {
                trackIds[cornerId] = minId;
                dists[cornerId]    = minDist;
            }

            // next corner
            ++cornerId;
        } //...for each corner

        // check unique
        std::set<int> uniqueIds;
        uniqueIds.insert(trackIds.begin(), trackIds.end());
        std::cout << "trackIds:";
        for (size_t vi = 0; vi != trackIds.size(); ++vi)std::cout << trackIds[vi] << " ";
        std::cout << "\n";
        std::cout << "dists:";
        for (size_t vi = 0; vi != dists.size(); ++vi)std::cout << dists[vi] << " ";
        std::cout << "\n";
        if (uniqueIds.size() != trackIds.size()) {
            // find occluded corner
            if (uniqueIds.size() == trackIds.size() - 1) {
                // find missing
                int id = 0;
                while (std::find(trackIds.begin(), trackIds.end(), id) != trackIds.end())
                    ++id;
                if (id >= int(trackIds.size()))
                    throw new Cuboid_MatchTrackIds_LogicErrorException("mistake");
                int missingId = id;
                std::cout << "missingId:" << missingId << std::endl;

                // find duplicate
                id = 0;
                while (std::count(trackIds.begin(), trackIds.end(), id) < 2)
                    ++id;
                if (id >= int(trackIds.size()))
                    throw new Cuboid_MatchTrackIds_LogicErrorException("mistake");
                int dupliId = id;
                std::cout << "dupliId:" << dupliId << std::endl;

                // find larger dist among duplicates
                int      maxId   = -1;
                float    maxDist = 0.;
                for (int i       = 0; i != int(trackIds.size()); ++i)
                    if (trackIds[i] == dupliId) if (dists[i] > maxDist) {
                        maxDist = dists[i];
                        maxId   = i;
                    }
                if (!(maxDist > 0.) || (maxId == -1))
                    throw new Cuboid_MatchTrackIds_LogicErrorException("mistake");
                // replace larger dist duplicate by missing Id
                std::cout << "trackIds[" << maxId << "] := " << missingId << std::endl;
                trackIds[maxId] = missingId;
                std::cout << "trackIds:";
                for (size_t vi = 0; vi != trackIds.size(); ++vi)std::cout << trackIds[vi] << " ";
                std::cout << "\n";
                return true;
            }
            else
                std::cout << "uniqueIds.size() : " << uniqueIds.size() << " != " << trackIds.size() - 1 << std::endl;
            return false;
        }

        return true;
    } //...matchTrackIds

    void Cuboid::init() {
        if (!_corners) {
            _corners.reset(new typename CornersPtrT::element_type);
            int      id(0);
            for (int z = 0; z != 2; ++z)
                for (int y = 0; y != 2; ++y)
                    for (int x = 0; x != 2; ++x, ++id) {
                        _corners->col(id) = Eigen::Matrix<Scalar, 4, 1>(x - 0.5, y - 0.5, z - 0.5, 1.);
                    }
        } //...if corners need init

        // unique name
        {
            static int uid = 0;
            char       name[255];
            sprintf(name, "Cuboid%02d", uid);
            _name  = name;
            _objId = uid;
            ++uid;
        }

        _shape = BOX;
    } //...init()

    Cuboid::Transform4x4 Cuboid::getTransform4x4(const FrameId &frameId) const {
        Transform4x4 T(Transform4x4::Identity());
        T.topLeftCorner(3, 3) = this->getPose(frameId).toRotationMatrix();
        T.block<3, 1>(0, 3)   = this->getPosition(frameId);
        Transform4x4 scale(Transform4x4::Identity());
        scale.topLeftCorner(3, 3) = this->getSize().asDiagonal();
        T = T * scale;
        return T;
    } //...getTransform4x4()

    TransformationT Cuboid::getTransformation(const FrameId &frameId) const {
        return TranslationT(this->getPosition(frameId)) * TransformationT(this->getPose(frameId).toRotationMatrix()) * this->getSizeTransform();
    }

    Scalar Cuboid::getMass() const { return _mass; }

    void           Cuboid::setMass(Scalar mass) { _mass = mass; }

    bool Cuboid::isMassFinite() const { return _mass > 1.e-3; }
  } //...ns bundle_physics
} //...ns tracking
