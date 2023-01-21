// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#ifndef POSITIONWITHHEIGHTMAP_HPP
#define POSITIONWITHHEIGHTMAP_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include "HeightmapNames.hpp"

#include "int/actionnodes/AbstractAction.hpp"

class PositionWithHeightmap : public BT::StatefulActionNode, public AbstractAction
{
  public:
    PositionWithHeightmap(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<CompositeHeightmapName>("heightmap") };
    }

    /// method to be called at the beginning.
    /// If it returns RUNNING, this becomes an asychronous node.
    virtual BT::NodeStatus onStart();

    /// method invoked by a RUNNING action.
    virtual BT::NodeStatus onRunning();

    /// when the method halt() is called and the action is RUNNING, this method is invoked.
    /// This is a convenient place todo a cleanup, if needed.
    virtual void onHalted();

    private:
        CompositeHeightmapName _heightmap;

        void parseArguments();
        behTreeReturnEnum execute(bool onStarted);

};

#endif