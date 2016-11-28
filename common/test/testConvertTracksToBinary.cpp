//
// Created by bontius on 15/07/16.
//

#if 0
#include <tracking/common/track.h>
#include <tracking/common/tracks.h>
#include <tracking/common/io/os.h>
#include "gtest/gtest.h"
#include "tracking/common/io/broxIO.h"
#include "tracking/common/io/sequentialReader.h"
class ConvertTracksToBinaryTestFixture : public ::testing::Test {
    public:
        ConvertTracksToBinaryTestFixture()
            : fPath("data/listTracks32.dat"),
              ofPath("_tracks.bin") {
            io::removeFile(ofPath);
            EXPECT_FALSE(io::exists(ofPath));
        } //...FlowIOTestFixture()

    protected:
        virtual void TearDown() override final {
            ::testing::Test::TearDown();
            io::removeFile(ofPath);
            EXPECT_FALSE(io::exists(ofPath));
        }

    protected:
        std::string const fPath;
        std::string const ofPath;
};

/** \brief Makes sure reading .dat files works. */
TEST_F(ConvertTracksToBinaryTestFixture,ReadTracks) {
    using namespace tracking;
    using _Scalar = Tracks2D::Scalar;

    Tracks2D tracks;
    BroxIO::readTracks(fPath, tracks);
    ASSERT_GT(tracks.size(),0);

    EXPECT_EQ(tracks.getSequenceLength(), 32);
    EXPECT_EQ(tracks.size(),119721);

    auto const& track = *tracks.begin();
    EXPECT_EQ(track.size(), 3);
    ASSERT_TRUE(track.hasPoint(0));
    EXPECT_EQ(track.getPoint(0).getPoint().coeff(0), _Scalar(309.f));
    EXPECT_EQ(track.getPoint(0).getPoint().coeff(1), _Scalar(8.f));
    ASSERT_TRUE(track.hasPoint(1));
    EXPECT_EQ(track.getPoint(1).getPoint().coeff(0), _Scalar(308.862f));
    EXPECT_EQ(track.getPoint(1).getPoint().coeff(1), _Scalar(8.96245));
    ASSERT_TRUE(track.hasPoint(2));
    EXPECT_EQ(track.getPoint(2).getPoint().coeff(0), _Scalar(308.682f));
    EXPECT_EQ(track.getPoint(2).getPoint().coeff(1), _Scalar(9.66767f));

    EXPECT_EQ(tracks.back().size(), 2);
    ASSERT_TRUE(tracks.back().hasPoint(30));
    EXPECT_EQ(tracks.back().getPoint(30).getPoint().coeff(0), _Scalar(529.f));
    EXPECT_EQ(tracks.back().getPoint(30).getPoint().coeff(1), _Scalar(224.f));
    ASSERT_TRUE(tracks.back().hasPoint(31));
    EXPECT_EQ(tracks.back().getPoint(31).getPoint().coeff(0), _Scalar(529.f));
    EXPECT_EQ(tracks.back().getPoint(31).getPoint().coeff(1), _Scalar(224.f));
}

/** \brief Makes sure converting .dat (ascii) to .bin and reading .bin at once. */
TEST_F(ConvertTracksToBinaryTestFixture,BatchBinary) {
    using namespace tracking;

    ASSERT_FALSE(io::exists(ofPath));
    BroxIO::convertDatToBin(fPath, ofPath, 0);
    ASSERT_TRUE(io::exists(ofPath));

    Tracks2D tracks;
    BroxIO::readTracks(fPath, tracks);

    Tracks2D tracksBinary;
    BroxIO::readTracks(ofPath, tracksBinary);

    EXPECT_EQ(tracksBinary.getSequenceLength(), tracks.getSequenceLength());
    EXPECT_EQ(tracksBinary.size()             , tracks.size());

    for (size_t i = 0; i != tracksBinary.getTrackCount(); ++i) {
        Track2D const& trackBinary = tracksBinary.getTrack(i);
        for (auto const& pair : tracks.getTrack(i).getPoints()) {
            EXPECT_NEAR(pair.second.getPoint().coeff(0), trackBinary.getPoint(pair.first).getPoint().coeff(0), 1e-3f);
            EXPECT_NEAR(pair.second.getPoint().coeff(1), trackBinary.getPoint(pair.first).getPoint().coeff(1), 1e-3f);
        }
    }
}

/** \brief Makes sure converting reading .bin sequentially works. */
TEST_F(ConvertTracksToBinaryTestFixture,SequentialBinary) {
    using namespace tracking;

    ASSERT_FALSE(io::exists(ofPath));
    BroxIO::convertDatToBin(fPath, ofPath, 0);
    ASSERT_TRUE(io::exists(ofPath));

    Tracks2D tracks;
    BroxIO::readTracks(fPath, tracks);

    SequentialReader reader(ofPath, SequentialReader::BINARY);
    ASSERT_TRUE(reader.init());
    EXPECT_EQ(reader.getSequenceLength(), tracks.getSequenceLength());

    TrackId trackId{0};
    Track2D track;
    while (reader.readNext(track)) {
        for (auto const& pair : tracks.getTrack(trackId).getPoints()) {
            EXPECT_NEAR(pair.second.getPoint().coeff(0), track.getPoint(pair.first).getPoint().coeff(0), 1e-3f);
            EXPECT_NEAR(pair.second.getPoint().coeff(1), track.getPoint(pair.first).getPoint().coeff(1), 1e-3f);
        }
        ++trackId;
    }
    EXPECT_EQ(trackId, tracks.getTrackCount());
}
#endif
