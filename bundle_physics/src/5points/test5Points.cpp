#include "tracking/5points/test5Points.h"
#include <vector>
#include "bundler_sfm/src/Epipolar.h"
#include "tracking/common/tracks.h"
#include "Eigen/Core"
#include <algorithm>

#define SQR(a) ((a)*(a))

namespace tracking
{
    int normalizePoints( std::vector<Keypoint>& k1 )
    {

//           c = mean(pts(1:2,finiteind)')';            % Centroid of finite points
        Keypoint mean = std::accumulate( k1.begin(), k1.end(), Keypoint(0.,0.), []( Keypoint&init, const Keypoint&next ) { return Keypoint(init.m_x+next.m_x, init.m_y+next.m_y); } );
        mean.m_x /= static_cast<float>(k1.size());
        mean.m_y /= static_cast<float>(k1.size());
        std::cout << "[" << __func__ << "]: " << "mean: "<< mean.m_x << "," << mean.m_y << std::endl;

//           newp(1,finiteind) = pts(1,finiteind)-c(1); % Shift origin to centroid.
//           newp(2,finiteind) = pts(2,finiteind)-c(2);
//           dist = sqrt(newp(1,finiteind).^2 + newp(2,finiteind).^2);
        Eigen::Matrix<float, 1, Eigen::Dynamic> dist( 1, k1.size() );
        {
            int i = 0;
            std::for_each( k1.begin(), k1.end(), [&dist,&i,&mean]( const Keypoint& pnt) {
                dist(i) = std::sqrt(SQR(pnt.m_x - mean.m_x) + SQR(pnt.m_y - mean.m_y));
                ++i;
            });
            std::cout << "dist: " << dist << std::endl;
        }

        // meandist = mean(dist(:));  % Ensure dist is a column vector for Octave 3.0.1
        // scale    = sqrt(2)/meandist;
        const float scale = std::sqrt(2.) / dist.mean();
        // T = [scale   0    -scale*c(1)
        //       0     scale -scale*c(2)
        //       0       0      1       ];
        Eigen::Matrix3f T; T <<  scale,     0., -scale*mean.m_x,
                                    0.,  scale, -scale*mean.m_y,
                                    0.,     0.,              1.;
        std::cout << "T:\n" << T << std::endl;
        // newpts = T * pts;
        int i = 0;
        std::transform( k1.begin(), k1.end(), k1.begin(), [&i,&T](Keypoint& pnt ) {
            Keypoint tmp;
            tmp.m_x = T(0,0) * pnt.m_x + T(0,1) * pnt.m_y + T(0,2);
            tmp.m_y = T(1,0) * pnt.m_x + T(1,1) * pnt.m_y + T(1,2);
            return tmp;
        });

    } //normalizePts

    int test5Points( const Tracks2D& tracks2d, const FrameIdsT& frameIds, const Mapper& mapper )
    {
        int m_fmatrix_rounds = 1000;
        float m_fmatrix_threshold = 1.0;

        std::vector<Keypoint> k1, k2;

        FrameId next = frameIds.at(0);
        ++next;
        for ( FrameId prev = frameIds.at(0); next <= frameIds.back(); ++next )
        {
            k1.clear();
            k2.clear();
            for ( auto const& track : tracks2d )
            {
                // if has both points
                if ( track.hasPoint(prev) && track.hasPoint(next) )
                {
                    const TrackPoint2D& pnt0( track.getPoint(prev) );
                    k1.push_back( Keypoint(pnt0(0), pnt0(1)) );
                    const TrackPoint2D& pnt1( track.getPoint(next) );
                    k2.push_back( Keypoint(pnt1(0), pnt1(1)) );
                } //...has both points
            } //...tracks2d

            assert( k1.size() == k2.size() );
            //normalizePoints( k1 );
            //normalizePoints( k2 );

            Eigen::Matrix<double,3,3> F, R0;
            Eigen::Matrix<double,4,1> t0;

            std::vector<KeypointMatch> matches( k1.size() );
            for ( size_t i = 0; i != matches.size(); ++i )
                matches[i] = KeypointMatch(i,i);
#if 1
            Eigen::Matrix3d K1( Eigen::Matrix3d::Zero() );
            //K1(0,0) = mapper.getIntrinsics()(0,0);
            //K1(1,1) = mapper.getIntrinsics()(1,1);
            K1 = mapper.getIntrinsics();
            Eigen::Matrix3d K2 = K1;
            int num_inliers = EstimatePose5Point(k1, k2, matches,
                               1024, /*512*/ /* m_fmatrix_rounds, 8 * m_fmatrix_rounds */
                               0.0001, // 0.25 * m_fmatrix_threshold,
                               K1.data(), K2.data(), R0.data(), t0.data());
            std::cout << "R0:\n" << R0 << ",\nt0: " << t0.transpose()
                      << ",\nK1\n" << K1 << ",\nK2:\n" << K2
                      << std::endl;
            // F = T2'*F*T1;
#elif 0
            std::vector<int> inliers =
                    EstimateFMatrix(k1,
                                    k2,
                                    matches,
                                    m_fmatrix_rounds,
                                    m_fmatrix_threshold /* 20.0 */ /* 9.0 */,
                                    /* out: */ F.data(), true );
            int num_inliers = (int) inliers.size();
            std::cout << "F:\n" << F << std::endl;
#endif
            printf("Inliers[%u,%u] = %d out of %d\n", static_cast<unsigned>(prev), static_cast<unsigned>(next), num_inliers,(int) matches.size() );
        }

        return EXIT_SUCCESS;
    }

} //...ns tracking

#undef SQR
