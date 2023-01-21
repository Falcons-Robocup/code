// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#ifndef SEQUENCESTARASYNCNODE_H
#define SEQUENCESTARASYNCNODE_H

#include "behaviortree_cpp_v3/control_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

void registerNode_SequenceStarAsyncNode(BT::BehaviorTreeFactory& btFactory);

class SequenceStarAsyncNode : public BT::ControlNode
{
  public:
    SequenceStarAsyncNode(const std::string& name);

    virtual ~SequenceStarAsyncNode() override = default;

    virtual void halt() override;

  private:

    size_t current_child_idx_;

    virtual BT::NodeStatus tick() override;
};

#endif