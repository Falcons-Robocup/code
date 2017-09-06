/*********************************************************************
 *
 * $Id: yocto_tilt.h 19606 2015-03-05 10:35:57Z seb $
 *
 * Declares yFindTilt(), the high-level API for Tilt functions
 *
 * - - - - - - - - - License information: - - - - - - - - - 
 *
 *  Copyright (C) 2011 and beyond by Yoctopuce Sarl, Switzerland.
 *
 *  Yoctopuce Sarl (hereafter Licensor) grants to you a perpetual
 *  non-exclusive license to use, modify, copy and integrate this
 *  file into your software for the sole purpose of interfacing
 *  with Yoctopuce products.
 *
 *  You may reproduce and distribute copies of this file in
 *  source or object form, as long as the sole purpose of this
 *  code is to interface with Yoctopuce products. You must retain
 *  this notice in the distributed source file.
 *
 *  You should refer to Yoctopuce General Terms and Conditions
 *  for additional information regarding your rights and
 *  obligations.
 *
 *  THE SOFTWARE AND DOCUMENTATION ARE PROVIDED 'AS IS' WITHOUT
 *  WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING 
 *  WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS
 *  FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO
 *  EVENT SHALL LICENSOR BE LIABLE FOR ANY INCIDENTAL, SPECIAL,
 *  INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 *  COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR 
 *  SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT 
 *  LIMITED TO ANY DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR
 *  CONTRIBUTION, OR OTHER SIMILAR COSTS, WHETHER ASSERTED ON THE
 *  BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE), BREACH OF
 *  WARRANTY, OR OTHERWISE.
 *
 *********************************************************************/


#ifndef YOCTO_TILT_H
#define YOCTO_TILT_H

#include "yocto_api.h"
#include <cfloat>
#include <cmath>
#include <map>

//--- (YTilt return codes)
//--- (end of YTilt return codes)
//--- (YTilt definitions)
class YTilt; // forward declaration

typedef void (*YTiltValueCallback)(YTilt *func, const string& functionValue);
class YMeasure; // forward declaration
typedef void (*YTiltTimedReportCallback)(YTilt *func, YMeasure measure);
#ifndef _Y_AXIS_ENUM
#define _Y_AXIS_ENUM
typedef enum {
    Y_AXIS_X = 0,
    Y_AXIS_Y = 1,
    Y_AXIS_Z = 2,
    Y_AXIS_INVALID = -1,
} Y_AXIS_enum;
#endif
//--- (end of YTilt definitions)

//--- (YTilt declaration)
/**
 * YTilt Class: Tilt function interface
 *
 * The YSensor class is the parent class for all Yoctopuce sensors. It can be
 * used to read the current value and unit of any sensor, read the min/max
 * value, configure autonomous recording frequency and access recorded data.
 * It also provide a function to register a callback invoked each time the
 * observed value changes, or at a predefined interval. Using this class rather
 * than a specific subclass makes it possible to create generic applications
 * that work with any Yoctopuce sensor, even those that do not yet exist.
 * Note: The YAnButton class is the only analog input which does not inherit
 * from YSensor.
 */
class YOCTO_CLASS_EXPORT YTilt: public YSensor {
#ifdef __BORLANDC__
#pragma option push -w-8022
#endif
//--- (end of YTilt declaration)
protected:
    //--- (YTilt attributes)
    // Attributes (function value cache)
    Y_AXIS_enum     _axis;
    YTiltValueCallback _valueCallbackTilt;
    YTiltTimedReportCallback _timedReportCallbackTilt;

    friend YTilt *yFindTilt(const string& func);
    friend YTilt *yFirstTilt(void);

    // Function-specific method for parsing of JSON output and caching result
    virtual int     _parseAttr(yJsonStateMachine& j);

    // Constructor is protected, use yFindTilt factory function to instantiate
    YTilt(const string& func);
    //--- (end of YTilt attributes)

public:
    ~YTilt();
    //--- (YTilt accessors declaration)

    static const Y_AXIS_enum AXIS_X = Y_AXIS_X;
    static const Y_AXIS_enum AXIS_Y = Y_AXIS_Y;
    static const Y_AXIS_enum AXIS_Z = Y_AXIS_Z;
    static const Y_AXIS_enum AXIS_INVALID = Y_AXIS_INVALID;

    Y_AXIS_enum         get_axis(void);

    inline Y_AXIS_enum  axis(void)
    { return this->get_axis(); }

    /**
     * Retrieves a tilt sensor for a given identifier.
     * The identifier can be specified using several formats:
     * <ul>
     * <li>FunctionLogicalName</li>
     * <li>ModuleSerialNumber.FunctionIdentifier</li>
     * <li>ModuleSerialNumber.FunctionLogicalName</li>
     * <li>ModuleLogicalName.FunctionIdentifier</li>
     * <li>ModuleLogicalName.FunctionLogicalName</li>
     * </ul>
     *
     * This function does not require that the tilt sensor is online at the time
     * it is invoked. The returned object is nevertheless valid.
     * Use the method YTilt.isOnline() to test if the tilt sensor is
     * indeed online at a given time. In case of ambiguity when looking for
     * a tilt sensor by logical name, no error is notified: the first instance
     * found is returned. The search is performed first by hardware name,
     * then by logical name.
     *
     * @param func : a string that uniquely characterizes the tilt sensor
     *
     * @return a YTilt object allowing you to drive the tilt sensor.
     */
    static YTilt*       FindTilt(string func);

    /**
     * Registers the callback function that is invoked on every change of advertised value.
     * The callback is invoked only during the execution of ySleep or yHandleEvents.
     * This provides control over the time when the callback is triggered. For good responsiveness, remember to call
     * one of these two functions periodically. To unregister a callback, pass a null pointer as argument.
     *
     * @param callback : the callback function to call, or a null pointer. The callback function should take two
     *         arguments: the function object of which the value has changed, and the character string describing
     *         the new advertised value.
     * @noreturn
     */
    virtual int         registerValueCallback(YTiltValueCallback callback);
    using YSensor::registerValueCallback;

    virtual int         _invokeValueCallback(string value);

    /**
     * Registers the callback function that is invoked on every periodic timed notification.
     * The callback is invoked only during the execution of ySleep or yHandleEvents.
     * This provides control over the time when the callback is triggered. For good responsiveness, remember to call
     * one of these two functions periodically. To unregister a callback, pass a null pointer as argument.
     *
     * @param callback : the callback function to call, or a null pointer. The callback function should take two
     *         arguments: the function object of which the value has changed, and an YMeasure object describing
     *         the new advertised value.
     * @noreturn
     */
    virtual int         registerTimedReportCallback(YTiltTimedReportCallback callback);
    using YSensor::registerTimedReportCallback;

    virtual int         _invokeTimedReportCallback(YMeasure value);


    inline static YTilt* Find(string func)
    { return YTilt::FindTilt(func); }

    /**
     * Continues the enumeration of tilt sensors started using yFirstTilt().
     *
     * @return a pointer to a YTilt object, corresponding to
     *         a tilt sensor currently online, or a null pointer
     *         if there are no more tilt sensors to enumerate.
     */
           YTilt           *nextTilt(void);
    inline YTilt           *next(void)
    { return this->nextTilt();}

    /**
     * Starts the enumeration of tilt sensors currently accessible.
     * Use the method YTilt.nextTilt() to iterate on
     * next tilt sensors.
     *
     * @return a pointer to a YTilt object, corresponding to
     *         the first tilt sensor currently online, or a null pointer
     *         if there are none.
     */
           static YTilt* FirstTilt(void);
    inline static YTilt* First(void)
    { return YTilt::FirstTilt();}
#ifdef __BORLANDC__
#pragma option pop
#endif
    //--- (end of YTilt accessors declaration)
};

//--- (Tilt functions declaration)

/**
 * Retrieves a tilt sensor for a given identifier.
 * The identifier can be specified using several formats:
 * <ul>
 * <li>FunctionLogicalName</li>
 * <li>ModuleSerialNumber.FunctionIdentifier</li>
 * <li>ModuleSerialNumber.FunctionLogicalName</li>
 * <li>ModuleLogicalName.FunctionIdentifier</li>
 * <li>ModuleLogicalName.FunctionLogicalName</li>
 * </ul>
 *
 * This function does not require that the tilt sensor is online at the time
 * it is invoked. The returned object is nevertheless valid.
 * Use the method YTilt.isOnline() to test if the tilt sensor is
 * indeed online at a given time. In case of ambiguity when looking for
 * a tilt sensor by logical name, no error is notified: the first instance
 * found is returned. The search is performed first by hardware name,
 * then by logical name.
 *
 * @param func : a string that uniquely characterizes the tilt sensor
 *
 * @return a YTilt object allowing you to drive the tilt sensor.
 */
inline YTilt* yFindTilt(const string& func)
{ return YTilt::FindTilt(func);}
/**
 * Starts the enumeration of tilt sensors currently accessible.
 * Use the method YTilt.nextTilt() to iterate on
 * next tilt sensors.
 *
 * @return a pointer to a YTilt object, corresponding to
 *         the first tilt sensor currently online, or a null pointer
 *         if there are none.
 */
inline YTilt* yFirstTilt(void)
{ return YTilt::FirstTilt();}

//--- (end of Tilt functions declaration)

#endif
