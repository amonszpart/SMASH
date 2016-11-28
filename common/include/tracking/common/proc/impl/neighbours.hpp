//
// Created by bontius on 06/04/16.
//

#ifndef TRACKVIDEO_COMMON_NEIGHBOURS_HPP
#define TRACKVIDEO_COMMON_NEIGHBOURS_HPP

#include "tracking/common/eigen.h"
#include "tracking/common/proc/neighbours.h"
#include "nanoflann/nanoflann.hpp"
#include <iostream>

namespace tracking {
namespace common {
template <typename _TrackT>
NeighboursT buildNeighbourhood(GroupedTracks<_TrackT> const& tracks2, std::set<FrameId> const& allFrameIds, int const K,
                               std::function<bool (TrackId const&)>* useTrack ) {
    std::cout << "[" << __func__ << "] " << "starting..." << std::endl;

    using Scalar = typename _TrackT::Scalar;
    using MatrixT = typename  Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
    using NanoTreeT = nanoflann::KDTreeEigenMatrixAdaptor<MatrixT>;

    FrameId const dim = allFrameIds.size() * 2;
    std::map<LinId, std::pair<TrackId,LinId> > revMap;
    int N = useTrack ?
            std::count_if(std::begin(tracks2),std::end(tracks2), [&useTrack](auto const& entry){
                return (*useTrack)(entry.getTrackId());
            })
                     : tracks2.size();
    MatrixT m(N,dim);
    LinId localId(0);
    for (LinId linId = 0; linId != tracks2.size(); ++linId) {
        auto const& track = tracks2.at(linId);
        if (useTrack && !(*useTrack)(track.getTrackId()))
            continue;

        int d = 0;
//        for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId), d += 2) {
        for (auto it = allFrameIds.begin(); it != allFrameIds.end(); ++it, d+=2) {
            auto iter = track.findPoint(*it);
            if (iter != track.end()) {
                m(localId, d)     = iter->second(0);
                m(localId, d + 1) = iter->second(1);
            } else {
                m(localId, d)     = 0.; //randf(0.);
                m(localId, d + 1) = 0.; //randf(0.);
            }
        }
        revMap[localId] = std::make_pair(track.getTrackId(), linId);
        ++localId;
    }

    NanoTreeT tree(dim, m, /* max_leaf: */ 10);
    tree.index->buildIndex();

    std::vector<size_t>   ret_indexes(K);
    std::vector<Scalar> out_dists_sqr(K);

    nanoflann::KNNResultSet<Scalar> resultSet(K);

    std::map< std::pair<TrackId,TrackId>, double> dists;
    NeighboursT neighbours;
    //for (LinId linId = 0; linId != tracks2.size(); ++linId) {
    for (std::pair<LinId,std::pair<TrackId, LinId> > entry  : revMap) {
        LinId   const localId = entry.first;
        TrackId const currTrackId = entry.second.first;
        LinId   const linId = entry.second.second;
        resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
        tree.index->findNeighbors(resultSet, m.row(localId).data(), nanoflann::SearchParams(20));

        for (size_t i = 0; i < resultSet.size(); ++i) {
            TrackId const& neighId = revMap.at(ret_indexes[i]).first;
            if (currTrackId != neighId) {
                double dist = std::sqrt(out_dists_sqr[i]) / static_cast<Scalar>(tracks2.getTrack(linId).size());
                if (dist > 5.)
                    continue;
                std::pair<TrackId,TrackId> key = {currTrackId, neighId};
                if (key.first > key.second)
                    std::swap(key.first,key.second);
                dists[key] = dist;
                neighbours[currTrackId].insert(neighId);
            } //...if not same point
        } //...for each neighbour
    } //...for each used entry
#if 0
    auto drawTrack = [](cv::Mat &img,Track2D const& track,cv::Scalar const& color, double const dist) {
            bool inited(false);
            cv::Point2i p0;
            for ( auto const& pair : track.getPoints()) {
                cv::Point2i p1(pair.second(0),pair.second(1));
                cv::circle(img,p1,1,color);
                if (inited)
                    cv::line(img,p0,p1,color);
                p0 = p1;
                if (!inited && !(dist<0.))
                    cv::putText(img,std::to_string(dist),p0 - cv::Point2i(randf(20.),randf(20.)),1,2.,color);
                inited = true;
            }
        };

        io::my_mkdir("neighs");
        cv::Mat img(480,640,CV_8UC3);
        for (auto const& entry : neighbours){
            if (randf() > 0.02)
                continue;
            img.setTo(0.);
            TrackId const& trackId = entry.first;
            Track2D const& track = tracks2.getTrackByLabel(trackId);
            drawTrack(img, track,{255.,255.,255.}, -1.);
            for (auto const& neighTrackId : entry.second) {
                double dist = -1;
                std::pair<TrackId,TrackId> key = {trackId,neighTrackId};
                if (key.first > key.second)
                    std::swap(key.first,key.second);
                if (dists.find(key) != dists.end()) {
                    dist = dists.at(key);
                    std::cout << "dist: " << dist << std::endl;
                }
                drawTrack(img, tracks2.getTrackByLabel(neighTrackId), {randf(200.), randf(200.), randf(200.)}, -1.);
            }
            cv::imshow("neighs",img);
            cv::imwrite("neighs/" + std::to_string(trackId) + ".jpg",img);
            //char c(0); while ((c = cv::waitKey(50)) != 27);
            cv::waitKey(50);
        }
        cv::imshow("neighs",img);
        char c(0); while ((c = cv::waitKey()) != 27);
#endif
    std::cout << "[" << __func__ << "] " << "finished..." << std::endl;
    return neighbours;
} //...buildNeighbourhood
} //...ns common
} //...ns tracking

#endif //TRACKVIDEO_COMMON_NEIGHBOURS_HPP
