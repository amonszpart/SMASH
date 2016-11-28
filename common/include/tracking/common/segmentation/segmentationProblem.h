//
// Created by bontius on 25/04/16.
//

#ifndef TRACKVIDEO_COMMON_SEGMENTATIONPROBLEM_H
#define TRACKVIDEO_COMMON_SEGMENTATIONPROBLEM_H

#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include <stdlib.h> // rand()
#include <iostream>
#include <map>
#include <set>

namespace tracking {

/** \brief Class, that maps our variables to linear ids for a segmentation algorithm. */
template<typename _NodeT, typename _LabelT>
class SegmentationProblem {
public:
    typedef _NodeT  NodeT;
    typedef _LabelT LabelT;
    typedef size_t  _LinId; //!< Different LinId than \ref tracking::LinId

    inline _LinId addNode(_NodeT const node) {
        if (_nodesToLinIds.find(node) != _nodesToLinIds.end()) {
            std::cerr << "[" << __func__ << "] " << "already has node"/* << node*/ << std::endl;
        }
        _LinId linId = _nodesToLinIds.size();
        _nodesToLinIds.emplace(node, linId);
        _linIdsToNodes.emplace(linId, node);
        return _nodesToLinIds.at(node);
    }

    inline _LinId addLabel(_LabelT const label) {
        if (_labelsToLinIds.find(label) != _labelsToLinIds.end()) {
            std::cerr << "[" << __func__ << "] " << "already has label" << label << std::endl;
        }
        _LinId linId = _labelsToLinIds.size();
        _labelsToLinIds.emplace(label, linId);
        _linIdsToLabels.emplace(linId, label);
        return _labelsToLinIds.at(label);
    }

    inline _LinId getIdFromNode(NodeT const& value) const { return _nodesToLinIds.at(value); }

    inline _LinId getIdFromLabel(LabelT const& label) const { return _labelsToLinIds.at(label); }

    inline NodeT getNodeFromId(_LinId const& linId) const { return _linIdsToNodes.at(linId); }

    inline LabelT getLabelFromId(_LinId const& linId) const { return _linIdsToLabels.at(linId); }

    inline _LinId getNumNodes() const { return _nodesToLinIds.size(); }

    inline _LinId getNumLabels() const { return _labelsToLinIds.size(); }

    inline bool hasNode(NodeT const& node) const
    { return _nodesToLinIds.find(node) != std::end(_nodesToLinIds); }

    inline bool hasLabel(LabelT const& label) const
    { return _labelsToLinIds.find(label) != std::end(_labelsToLinIds); }

    inline std::pair<_LinId, bool> findNode(NodeT const& value) const {
        auto iter = _nodesToLinIds.find(value);
        if (iter != std::end(_nodesToLinIds)) {
            return {iter->second, true};
        } else {
            return {_LinId(-1), false};
        }
    }

    inline std::pair<_LinId, bool> findLabel(LabelT const& label) const {
        auto iter = _labelsToLinIds.find(label);
        if (iter != std::end(_labelsToLinIds)) {
            return {iter->second, true};
        } else {
            return {_LinId(-1), false};
        }
    } //...findNode()

    inline std::pair<_LinId, bool> findLabelFromId(_LinId const& linId) const {
        auto iter = _linIdsToLabels.find(linId);
        if (iter != std::end(_linIdsToLabels)) {
            return {iter->second, true};
        } else {
            return {_LinId(-1), false};
        }
    } //...findNode()

    inline std::map<NodeT, _LinId> const& getNodesToLinIds() const { return _nodesToLinIds; }

    inline std::map<LabelT, _LinId> const& getLabelsToLinIds() const { return _labelsToLinIds; }

protected:
    // storage:
    std::map<NodeT, _LinId>  _nodesToLinIds;
    std::map<LabelT, _LinId> _labelsToLinIds;
    // cache:
    std::map<_LinId, NodeT>  _linIdsToNodes;
    std::map<_LinId, LabelT> _linIdsToLabels;
}; //...struct SegmentationProblem

//DEFINE_EXCEPTION(AddPairwise_NodesNotFound)

template<typename _NodeT, typename _LabelT, typename _Scalar>
class CostsSegmentationProblem : public SegmentationProblem<_NodeT, _LabelT> {
public:
    using Base = SegmentationProblem<_NodeT, _LabelT>;
    using NodeT = typename Base::NodeT;
    using LabelT = typename Base::LabelT;
    using _LinId = typename Base::_LinId;
    using Scalar = _Scalar;

    bool                    addUnary (NodeT const& node, LabelT const& label, Scalar const cost);
    Scalar                  getUnary (NodeT const& node, LabelT const& label) const;
    std::pair<Scalar, bool> findUnary(NodeT const& node, LabelT const& label) const;
    void addPairwise(NodeT const& node0, NodeT const& node1, Scalar const cost);
    std::map<std::pair<_LinId, _LinId>, Scalar> const& getPws() const { return _pws; }
    void setStartingLabel(NodeT const& node, LabelT const& label);
    _LinId randLabel() const;
    bool hasStartingLabels() const;
    std::vector<_LinId> const& getStartingLabels() const { return _startingLabels; }

    using Base::getIdFromNode;
    using Base::getIdFromLabel;

protected:
    using Base::_linIdsToLabels;
    std::map<std::pair<_LinId, _LinId>, Scalar> _unaries;
    std::map<std::pair<_LinId, _LinId>, Scalar> _pws;
    std::vector<_LinId> _startingLabels;

    _LinId _getMaxLinLabel() const;
}; //...class CostsSegmentationProblem

template <typename _A, typename _B>
std::ostream& operator<<( std::ostream &os, std::pair<_A,_B> const& elem) {
    os << elem.first << "," << elem.second;
    return os;
}

template <typename _NodeT, typename _LabelT, typename _Scalar> inline bool
CostsSegmentationProblem<_NodeT, _LabelT, _Scalar>::addUnary(NodeT const& node, LabelT const& label, Scalar const cost) {
    if (!this->hasNode(node)) {
        this->addNode(node);
    }
    if (!this->hasLabel(label)) {
        this->addLabel(label);
    }
    std::pair<_LinId, _LinId> key {this->getIdFromNode(node), this->getIdFromLabel(label)};
    if (_unaries.find(key) != _unaries.end()) {
        std::cerr << "[" << __func__ << "] " << "already have unary cost" << ", key: " << key.first << ","
                  << key.second << std::endl;
    }
    auto const ret = _unaries.insert({key, cost});
    return ret.second;
} //...addUnary()

template <typename _NodeT, typename _LabelT, typename _Scalar> inline _Scalar
CostsSegmentationProblem<_NodeT, _LabelT, _Scalar>::getUnary(NodeT const& node, LabelT const& label) const
{ return _unaries.at({this->getIdFromNode(node), this->getIdFromLabel(label)}); } //...getUnary()

template<typename _NodeT, typename _LabelT, typename _Scalar> std::pair<_Scalar, bool>
inline CostsSegmentationProblem<_NodeT,_LabelT,_Scalar>::findUnary(NodeT const& node, LabelT const& label) const {
    auto iter = _unaries.find({this->getIdFromNode(node), this->getIdFromLabel(label)});
    if (iter != std::end(_unaries)) {
        return {iter->second, true};
    } else {
        return {std::numeric_limits<Scalar>::max(), false};
    }
} //...getUnary()

template<typename _NodeT, typename _LabelT, typename _Scalar> inline void
CostsSegmentationProblem<_NodeT,_LabelT,_Scalar>::addPairwise(NodeT const& node0, NodeT const& node1,
                                                              Scalar const cost) {
    if (!this->hasNode(node0) || !this->hasNode(node1))
        throw new LogicException("Node(s) not found", __FILE__, __LINE__);

    _pws.insert({{this->getIdFromNode(node0), this->getIdFromNode(node1)}, cost});
} //...addPairwise()

template<typename _NodeT, typename _LabelT, typename _Scalar> inline void
CostsSegmentationProblem<_NodeT,_LabelT,_Scalar>::setStartingLabel(NodeT const& node, LabelT const& label) {
    if (!this->hasNode(node) || !this->hasLabel(label))
        throw new LogicException("Unknown node or label...", __FILE__, __LINE__);

    _LinId const linNode = getIdFromNode(node);
    while (_startingLabels.size() <= linNode)
        _startingLabels.emplace_back(randLabel());
    _startingLabels[linNode] = getIdFromLabel(label);
} //...setStartingLabel()

template<typename _NodeT, typename _LabelT, typename _Scalar> inline auto
CostsSegmentationProblem<_NodeT,_LabelT,_Scalar>::randLabel() const -> _LinId {
    _LinId linMaxLabel = _getMaxLinLabel();
    if (linMaxLabel == 0)
        return 0;
    else
        return std::rand() % (linMaxLabel+1);
} //...randLabel()

template<typename _NodeT, typename _LabelT, typename _Scalar>
inline auto CostsSegmentationProblem<_NodeT, _LabelT,_Scalar>::_getMaxLinLabel() const -> _LinId
{ return _linIdsToLabels.rbegin()->first; }

template<typename _NodeT, typename _LabelT, typename _Scalar>
inline bool CostsSegmentationProblem<_NodeT, _LabelT, _Scalar>::hasStartingLabels() const
{ return _getMaxLinLabel() < _startingLabels.size(); }

} //...ns tracking
#endif //TRACKVIDEO_COMMON_SEGMENTATIONPROBLEM_H
