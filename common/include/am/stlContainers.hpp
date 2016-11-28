//
// Created by bontius on 30/07/16.
//

#ifndef TRACKVIDEO_STLCONTAINERS_H
#define TRACKVIDEO_STLCONTAINERS_H

#include <iterator>
#include <algorithm>

namespace am {

template <typename _ContainerInT, typename _ContainerOutT, typename _UnaryOperation>
std::insert_iterator<_ContainerOutT>
append(_ContainerInT const& in, _ContainerOutT &__result, _UnaryOperation __unary_op) {
    return std::transform(in.cbegin(), in.cend(), std::inserter(__result, __result.end()), __unary_op);
} //...append()

} //...ns am

#endif //TRACKVIDEO_STLCONTAINERS_H
