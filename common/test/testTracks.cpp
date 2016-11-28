//
// Created by bontius on 16/07/16.
//

#include <stdlib.h>
#include "tracking/common/groupedTracks.h"
#include "gtest/gtest.h"

#include <vector>

class TracksTestFixture : public ::testing::Test {
    public:
        template <typename T, typename _FunctorT>
        inline T genUnique(std::vector<T> const& container, _FunctorT const& generator) {
            std::set<T> uniqueContainer(container.begin(), container.end());
            T value;
            do { value = generator(); } while (uniqueContainer.find(value) != uniqueContainer.end());
            return value;
        }

        TracksTestFixture(int64_t nTracks = 100, int64_t nPoints = 10)
            : _nTracks(nTracks), _nPoints(nPoints), _nGroups(4) {
            using namespace tracking;
            std::set<GroupId> uniqueGids;
            std::generate_n(std::inserter(uniqueGids, uniqueGids.end()), _nGroups, [](void){ return GroupId{static_cast<GroupId>(rand())}; });
            _allGroupIds.insert(_allGroupIds.end(), uniqueGids.begin(), uniqueGids.end());
            for (GroupId const& gid : _allGroupIds)
                std::cout << "GroupId:" << gid << ",";
            std::cout << std::endl;

            for (int64_t i = 0; i != _nTracks; ++i) {
                _trackIds.push_back(genUnique(_trackIds, [](){ return TrackId{static_cast<TrackId>(rand())};} ));
                _groupIds.push_back(_allGroupIds.at(rand() % _allGroupIds.size()));
                _frameIds.push_back({});
                Track3D  track3D{_trackIds.back()};
                for (int j = 0; j != _nPoints; ++j) {
                    FrameId frameId {static_cast<FrameId>(rand())};
                    do {
                        frameId = rand();
                        _frameIds.back().insert(frameId);
                    } while (static_cast<int>(_frameIds.back().size()) != j+1);
                    track3D.addPoint(frameId, TrackPoint3D{Eigen::Vector3f{1.f, 2.f, 3.f},
                                                           Eigen::Vector3f{4.f, 5.f, 6.f},
                                                           Eigen::Vector3f{7.f, 8.f, 9.f}});
                }
                std::cout << "adding " << track3D.getTrackId() << " to group " << _groupIds.back() << std::endl;
                _tracks3d.addTrack(track3D, _groupIds.back());
            }
        }
    protected:
        tracking::GroupedTracks3d _tracks3d;
        // GT
        std::vector<tracking::TrackId>      _trackIds; //!< GT trackIds for tracks
        std::vector<tracking::GroupId>      _groupIds; //!< GT groupId of tracks in order
        std::vector<std::set<tracking::FrameId> > _frameIds; //!< GT frameIds for points per track
        std::vector<tracking::GroupId> _allGroupIds;
        int64_t _nTracks, _nPoints, _nGroups;

};

using namespace tracking;

TEST(GroupedTracks3d, Constructor) {
    GroupedTracks3d tracks3d;
    EXPECT_EQ(tracks3d.size(), 0);
    EXPECT_EQ(tracks3d.getSequenceLength(), -1);
    EXPECT_TRUE(tracks3d.isUpdated());
}

TEST_F(TracksTestFixture, FixtureValidation) {
    EXPECT_EQ(_trackIds.size(), _nTracks);
    EXPECT_EQ(_groupIds.size(), _nTracks);
    EXPECT_EQ(_frameIds.size(), _nTracks);
    EXPECT_EQ(_allGroupIds.size(), _nGroups);
}

TEST_F(TracksTestFixture, Cache) {
    EXPECT_EQ(_tracks3d.size(), _trackIds.size());
    EXPECT_TRUE(_tracks3d.isUpdated());
    for (size_t i = 0; i != _trackIds.size(); ++i) {
        EXPECT_EQ(_trackIds[i], _tracks3d.getTrack(i).getTrackId());
        EXPECT_EQ(_trackIds[i], _tracks3d.getTrackByTrackId(_trackIds[i])->getTrackId());
        EXPECT_EQ(_trackIds[i], _tracks3d.getTrackByLabel(_trackIds[i]).getTrackId());
        EXPECT_EQ(_groupIds[i], _tracks3d.getGroupId(_trackIds[i]));
        EXPECT_EQ(_nPoints, _tracks3d.getTrack(i).size());
    }
}

TEST_F(TracksTestFixture, CopyConstructor) {
    GroupedTracks3d tracks3d(_tracks3d);

    EXPECT_EQ(tracks3d.size(), _trackIds.size());
    EXPECT_TRUE(tracks3d.isUpdated());
    for (size_t i = 0; i != _trackIds.size(); ++i) {
        EXPECT_EQ(_trackIds[i], tracks3d.getTrack(i).getTrackId());
        EXPECT_EQ(_trackIds[i], tracks3d.getTrackByTrackId(_trackIds[i])->getTrackId());
        EXPECT_EQ(_trackIds[i], tracks3d.getTrackByLabel(_trackIds[i]).getTrackId());
        EXPECT_EQ(_groupIds[i], tracks3d.getGroupId(_trackIds[i]));
        EXPECT_EQ(_nPoints, tracks3d.getTrack(i).size());
    }
}

// getTrack(TrackId)
// getGroupId
