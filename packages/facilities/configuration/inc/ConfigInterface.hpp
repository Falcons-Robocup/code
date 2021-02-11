// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigInterface.hpp
 *
 *  Created on: December 2019
 *      Author: Jan Feitsma
 */

#ifndef CONFIGINTERFACE_HPP_
#define CONFIGINTERFACE_HPP_


// use this interface to keep RTDB cleanly isolated from a package library,
// see for example PathPlanning, BallHandling

// (the configuration management code from PathPlanning was factored out 
// and templatized for reuse in other components)

template <typename T>
class ConfigInterface
{
public:
    ConfigInterface() {};
    virtual ~ConfigInterface() {};

    // return state change
    virtual bool get(T &config) { config = _config; return true; }

    virtual void set(T const &config) { _config = config; }

protected:
    T _config;
};

#endif

