// Copyright 2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#ifndef AVOIDPOI_HPP
#define AVOIDPOI_HPP

#include "behaviortree_cpp_v3/decorator_node.h"

#include "polygon2D.hpp"

class AvoidPOI : public BT::DecoratorNode
{
  public:
    AvoidPOI(const std::string& name, const BT::NodeConfiguration& config)
      : BT::DecoratorNode(name, config)
    {
    }

    virtual ~AvoidPOI() override = default;

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Point2D>("poi") };
    }


    private:
        Point2D _poi;

        void parseArguments();
        virtual BT::NodeStatus tick() override;

};

#endif