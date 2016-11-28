//
// Created by bontius on 22/04/16.
//

#ifndef TRACKVIDEO_COMMON_ORDEREDPAIR_HPP
#define TRACKVIDEO_COMMON_ORDEREDPAIR_HPP

#include <utility>
#include <algorithm>

template <typename T>
class OrderedPair : public std::pair<T,T> {
    public:
        typedef std::pair<T,T> Base;
        OrderedPair(T const p_first, T const p_second)
            : Base(std::min(p_first,p_second), std::max(p_first,p_second))
        {} //...OrderedPair()

        OrderedPair() = delete;
        OrderedPair(OrderedPair&& other) = default;
        OrderedPair(OrderedPair const& other) = default;
}; //...class OrderedPair

#endif //TRACKVIDEO_COMMON_ORDEREDPAIR_HPP
