// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef ADVANCEDQUERIES_HPP
#define ADVANCEDQUERIES_HPP

#include "behaviortree_cpp_v3/bt_factory.h"

#include "int/types/BehaviorTreeTypes.hpp"



void registerNodes_AdvancedQueries(BT::BehaviorTreeFactory& btFactory);

////////////////////////////
// IsOwnRobotClosestToPOI //
////////////////////////////

class IsOwnRobotClosestToPOI : public BT::ControlNode
{
  public:
    IsOwnRobotClosestToPOI(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Point2D>("poi") };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:

    bool _previousResult = false;

};

/////////////////
// IsPOIInArea //
/////////////////

class IsPOIInArea : public BT::ControlNode
{
  public:
    IsPOIInArea(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Point2D>("poi"),
                    BT::InputPort<Area2D>("area") };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:

    bool _previousResult = false;

};

////////////
// IsRole //
////////////

class IsRole : public BT::ControlNode
{
  public:
    IsRole(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<teamplay::RoleEnum>("role") };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:

    bool _previousResult = false;

};

////////////////
// IsSetpiece //
////////////////

bool isSetpiece(SetpieceTreeTypeEnum setpiece);

class IsSetpiece : public BT::ControlNode
{
  public:
    IsSetpiece(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<SetpieceTreeTypeEnum>("setpiece") };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:

    bool _previousResult = false;

};

/////////////////////////
// IsShootToPOIBlocked //
/////////////////////////

bool isShootToPOIBlocked(const Point2D& poi, tpShootTypeEnum shootType);

class IsShootToPOIBlocked : public BT::ControlNode
{
  public:
    IsShootToPOIBlocked(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ControlNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<Point2D>("poi"),
                    BT::InputPort<tpShootTypeEnum>("shootType") };
    }

    virtual void halt() override final
    {
        ControlNode::halt();
    }

    BT::NodeStatus tick() override;

  private:

    bool _previousResult = false;

};

#endif
