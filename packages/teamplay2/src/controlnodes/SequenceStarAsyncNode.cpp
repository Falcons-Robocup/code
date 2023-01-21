// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#include "int/controlnodes/SequenceStarAsyncNode.hpp"

void registerNode_SequenceStarAsyncNode(BT::BehaviorTreeFactory& btFactory)
{
    btFactory.registerNodeType<SequenceStarAsyncNode>("SequenceStarAsync");
}

SequenceStarAsyncNode::SequenceStarAsyncNode(const std::string& name)
    : BT::ControlNode::ControlNode(name, {} )
  , current_child_idx_(0)
{
    setRegistrationID("SequenceStarAsyncNode");
}

BT::NodeStatus SequenceStarAsyncNode::tick()
{
    const size_t children_count = children_nodes_.size();

    setStatus(BT::NodeStatus::RUNNING);

    BT::TreeNode* current_child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = current_child_node->executeTick();

    switch (child_status)
    {
        case BT::NodeStatus::RUNNING:
        {
            return child_status;
        }
        case BT::NodeStatus::FAILURE:
        {
            // DO NOT reset current_child_idx_ on failure
            for(size_t i=current_child_idx_; i < childrenCount(); i++)
            {
                haltChild(i);
            }

            return child_status;
        }
        case BT::NodeStatus::SUCCESS:
        {
            current_child_idx_++;

            // A child returned SUCCESS.
            // If this was the last child, return SUCCESS.
            // If not, continue on next tick
            if (current_child_idx_ == children_count)
            {
                haltChildren();
                current_child_idx_ = 0;
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                return BT::NodeStatus::RUNNING;
            }
            
        }
        break;

        case BT::NodeStatus::IDLE:
        {
            throw BT::LogicError("A child node must never return IDLE");
        }
    }   // end switch

    // The entire while loop completed. This means that all the children returned SUCCESS.
    return BT::NodeStatus::SUCCESS;
}

void SequenceStarAsyncNode::halt()
{
    current_child_idx_ = 0;
    BT::ControlNode::halt();
}