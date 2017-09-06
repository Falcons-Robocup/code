/*********************************************************************
 *
 * $Id: yocto_power.cpp 22694 2016-01-12 23:13:27Z seb $
 *
 * Implements yFindPower(), the high-level API for Power functions
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


#define _CRT_SECURE_NO_DEPRECATE //do not use windows secure crt
#include "yocto_power.h"
#include "yapi/yjson.h"
#include "yapi/yapi.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

YPower::YPower(const string& func): YSensor(func)
//--- (Power initialization)
    ,_cosPhi(COSPHI_INVALID)
    ,_meter(METER_INVALID)
    ,_meterTimer(METERTIMER_INVALID)
    ,_valueCallbackPower(NULL)
    ,_timedReportCallbackPower(NULL)
//--- (end of Power initialization)
{
    _className="Power";
}

YPower::~YPower()
{
//--- (YPower cleanup)
//--- (end of YPower cleanup)
}
//--- (YPower implementation)
// static attributes
const double YPower::COSPHI_INVALID = YAPI_INVALID_DOUBLE;
const double YPower::METER_INVALID = YAPI_INVALID_DOUBLE;

int YPower::_parseAttr(yJsonStateMachine& j)
{
    if(!strcmp(j.token, "cosPhi")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _cosPhi =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "meter")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _meter =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "meterTimer")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _meterTimer =  atoi(j.token);
        return 1;
    }
    failed:
    return YSensor::_parseAttr(j);
}


/**
 * Returns the power factor (the ratio between the real power consumed,
 * measured in W, and the apparent power provided, measured in VA).
 *
 * @return a floating point number corresponding to the power factor (the ratio between the real power consumed,
 *         measured in W, and the apparent power provided, measured in VA)
 *
 * On failure, throws an exception or returns Y_COSPHI_INVALID.
 */
double YPower::get_cosPhi(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPower::COSPHI_INVALID;
        }
    }
    return _cosPhi;
}

int YPower::set_meter(double newval)
{
    string rest_val;
    char buf[32]; sprintf(buf,"%d", (int)floor(newval * 65536.0 + 0.5)); rest_val = string(buf);
    return _setAttr("meter", rest_val);
}

/**
 * Returns the energy counter, maintained by the wattmeter by integrating the power consumption over time.
 * Note that this counter is reset at each start of the device.
 *
 * @return a floating point number corresponding to the energy counter, maintained by the wattmeter by
 * integrating the power consumption over time
 *
 * On failure, throws an exception or returns Y_METER_INVALID.
 */
double YPower::get_meter(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPower::METER_INVALID;
        }
    }
    return _meter;
}

/**
 * Returns the elapsed time since last energy counter reset, in seconds.
 *
 * @return an integer corresponding to the elapsed time since last energy counter reset, in seconds
 *
 * On failure, throws an exception or returns Y_METERTIMER_INVALID.
 */
int YPower::get_meterTimer(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPower::METERTIMER_INVALID;
        }
    }
    return _meterTimer;
}

/**
 * Retrieves $AFUNCTION$ for a given identifier.
 * The identifier can be specified using several formats:
 * <ul>
 * <li>FunctionLogicalName</li>
 * <li>ModuleSerialNumber.FunctionIdentifier</li>
 * <li>ModuleSerialNumber.FunctionLogicalName</li>
 * <li>ModuleLogicalName.FunctionIdentifier</li>
 * <li>ModuleLogicalName.FunctionLogicalName</li>
 * </ul>
 *
 * This function does not require that $THEFUNCTION$ is online at the time
 * it is invoked. The returned object is nevertheless valid.
 * Use the method YPower.isOnline() to test if $THEFUNCTION$ is
 * indeed online at a given time. In case of ambiguity when looking for
 * $AFUNCTION$ by logical name, no error is notified: the first instance
 * found is returned. The search is performed first by hardware name,
 * then by logical name.
 *
 * @param func : a string that uniquely characterizes $THEFUNCTION$
 *
 * @return a YPower object allowing you to drive $THEFUNCTION$.
 */
YPower* YPower::FindPower(string func)
{
    YPower* obj = NULL;
    obj = (YPower*) YFunction::_FindFromCache("Power", func);
    if (obj == NULL) {
        obj = new YPower(func);
        YFunction::_AddToCache("Power", func, obj);
    }
    return obj;
}

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
int YPower::registerValueCallback(YPowerValueCallback callback)
{
    string val;
    if (callback != NULL) {
        YFunction::_UpdateValueCallbackList(this, true);
    } else {
        YFunction::_UpdateValueCallbackList(this, false);
    }
    _valueCallbackPower = callback;
    // Immediately invoke value callback with current value
    if (callback != NULL && this->isOnline()) {
        val = _advertisedValue;
        if (!(val == "")) {
            this->_invokeValueCallback(val);
        }
    }
    return 0;
}

int YPower::_invokeValueCallback(string value)
{
    if (_valueCallbackPower != NULL) {
        _valueCallbackPower(this, value);
    } else {
        YSensor::_invokeValueCallback(value);
    }
    return 0;
}

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
int YPower::registerTimedReportCallback(YPowerTimedReportCallback callback)
{
    YSensor* sensor = NULL;
    sensor = this;
    if (callback != NULL) {
        YFunction::_UpdateTimedReportCallbackList(sensor, true);
    } else {
        YFunction::_UpdateTimedReportCallbackList(sensor, false);
    }
    _timedReportCallbackPower = callback;
    return 0;
}

int YPower::_invokeTimedReportCallback(YMeasure value)
{
    if (_timedReportCallbackPower != NULL) {
        _timedReportCallbackPower(this, value);
    } else {
        YSensor::_invokeTimedReportCallback(value);
    }
    return 0;
}

/**
 * Resets the energy counter.
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YPower::reset(void)
{
    return this->set_meter(0);
}

YPower *YPower::nextPower(void)
{
    string  hwid;

    if(YISERR(_nextFunction(hwid)) || hwid=="") {
        return NULL;
    }
    return YPower::FindPower(hwid);
}

YPower* YPower::FirstPower(void)
{
    vector<YFUN_DESCR>   v_fundescr;
    YDEV_DESCR             ydevice;
    string              serial, funcId, funcName, funcVal, errmsg;

    if(YISERR(YapiWrapper::getFunctionsByClass("Power", 0, v_fundescr, sizeof(YFUN_DESCR), errmsg)) ||
       v_fundescr.size() == 0 ||
       YISERR(YapiWrapper::getFunctionInfo(v_fundescr[0], ydevice, serial, funcId, funcName, funcVal, errmsg))) {
        return NULL;
    }
    return YPower::FindPower(serial+"."+funcId);
}

//--- (end of YPower implementation)

//--- (Power functions)
//--- (end of Power functions)
