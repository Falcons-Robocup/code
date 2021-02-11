// Copyright 2015 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/* File:      convert.hpp
 * Author:    JC Feitsma
 * Descr:     To convert between strings and stuff.
 *            Original code was created during the C++-course of Frank Brokken (2003?).
 * Created:   23-09-2006
 * Changelog:
 * 23-09-2006 renamed A2x to fromStr, added exception mechanism (based on http://www.parashift.com/c++-faq-lite/misc-technical-issues.html)
 *            added toStr from same source
 * 05-11-2007 removed use of logdebug
 *
 * TODO:      move implementations to end of header
 *
 */

#ifndef _INCLUDED_CONVERT_HPP_
#define _INCLUDED_CONVERT_HPP_


#include <iostream>
#include <sstream>
#include <string>
#include <stdexcept>
#include <typeinfo>
#include <cstdlib>
// #include "logdebug.hpp"


#ifndef FROMSTR_DEFAULT_FAIL
#define FROMSTR_DEFAULT_FAIL false
#endif


template <typename T>
inline std::string toStr(T const &x)
{
   std::ostringstream o;
   if (!(o << x)) {
      std::cerr << std::string("conversion of ") + typeid(x).name() + " to string failed" << std::endl;
      exit(1);
   }
   return o.str();
}


class fromStr: public std::istringstream
{
 private:
  bool d_failIfLeftoverChars;

 public:
  // constructors
  fromStr();
  fromStr(fromStr const &other, bool b = FROMSTR_DEFAULT_FAIL);
  fromStr(char const *s, bool b = FROMSTR_DEFAULT_FAIL);
  fromStr(std::string &s, bool b = FROMSTR_DEFAULT_FAIL);

  // assignment operators
  fromStr &operator=(fromStr const &other)
  {
    this->clear();
    this->str(other.str());
    return *this;
  }
  fromStr &operator=(char const *s)
  {
    this->clear();
    this->str(s);
    return *this;
  }
  fromStr &operator=(std::string const &s)
  {
    this->clear();
    this->str(s);
    return *this;
  }

  // conversion operators for assignment to any type for which the insertion
  // operator is defined on istreams
  template <typename T>
  operator T()
  {
    T result;
    char c;
    if ( !(*this >> result) || ( d_failIfLeftoverChars && this->get(c) ) ) {
      std::cerr << std::string("conversion of ") + str() + " to " + typeid(result).name() + " failed" << std::endl;
      exit(1);
    }     
    return result;
  }

}; // class fromStr
 

/* inline implementations */
inline fromStr::fromStr()
{}

/*
inline fromStr::fromStr(fromStr const &other, bool b)
:
  std::istringstream(other.str()),
  d_failIfLeftoverChars(b)
{}*/ // this generates a warning JFEI does not understand

inline fromStr::fromStr(char const *s, bool b)
:
  std::istringstream(s),
  d_failIfLeftoverChars(b)
{}

inline fromStr::fromStr(std::string &s, bool b)
:
  std::istringstream(s),
  d_failIfLeftoverChars(b)
{}

#endif
// #ifndef _INCLUDED_CONVERT_HPP_
