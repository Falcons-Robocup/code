// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * templatedSubject.hppA
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#ifndef TEMPLATEDSUBJECT_HPP_
#define TEMPLATEDSUBJECT_HPP_

#include <algorithm>
#include <vector>

#include <boost/function.hpp>

template <class T>
class templatedSubject
{
   public:
      templatedSubject(){};
      virtual ~templatedSubject(){};

      void setUpdateFunction(boost::function<void(T)> func)
      {
    	  _function = func;
      }

      void notify(T arg)
      {
    	  _function(arg);
      }
   private:
      boost::function<void(T)> _function;
};

#endif /* TEMPLATEDSUBJECT_HPP_ */
