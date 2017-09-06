/*********************************************************************
 *
 * $Id: yocto_pwminput.cpp 22694 2016-01-12 23:13:27Z seb $
 *
 * Implements yFindPwmInput(), the high-level API for PwmInput functions
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
#include "yocto_pwminput.h"
#include "yapi/yjson.h"
#include "yapi/yapi.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

YPwmInput::YPwmInput(const string& func): YSensor(func)
//--- (PwmInput initialization)
    ,_dutyCycle(DUTYCYCLE_INVALID)
    ,_pulseDuration(PULSEDURATION_INVALID)
    ,_frequency(FREQUENCY_INVALID)
    ,_period(PERIOD_INVALID)
    ,_pulseCounter(PULSECOUNTER_INVALID)
    ,_pulseTimer(PULSETIMER_INVALID)
    ,_pwmReportMode(PWMREPORTMODE_INVALID)
    ,_valueCallbackPwmInput(NULL)
    ,_timedReportCallbackPwmInput(NULL)
//--- (end of PwmInput initialization)
{
    _className="PwmInput";
}

YPwmInput::~YPwmInput()
{
//--- (YPwmInput cleanup)
//--- (end of YPwmInput cleanup)
}
//--- (YPwmInput implementation)
// static attributes
const double YPwmInput::DUTYCYCLE_INVALID = YAPI_INVALID_DOUBLE;
const double YPwmInput::PULSEDURATION_INVALID = YAPI_INVALID_DOUBLE;
const double YPwmInput::FREQUENCY_INVALID = YAPI_INVALID_DOUBLE;
const double YPwmInput::PERIOD_INVALID = YAPI_INVALID_DOUBLE;

int YPwmInput::_parseAttr(yJsonStateMachine& j)
{
    if(!strcmp(j.token, "dutyCycle")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _dutyCycle =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "pulseDuration")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _pulseDuration =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "frequency")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _frequency =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "period")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _period =  floor(atof(j.token) * 1000.0 / 65536.0 + 0.5) / 1000.0;
        return 1;
    }
    if(!strcmp(j.token, "pulseCounter")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _pulseCounter =  atol(j.token);
        return 1;
    }
    if(!strcmp(j.token, "pulseTimer")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _pulseTimer =  atol(j.token);
        return 1;
    }
    if(!strcmp(j.token, "pwmReportMode")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _pwmReportMode =  (Y_PWMREPORTMODE_enum)atoi(j.token);
        return 1;
    }
    failed:
    return YSensor::_parseAttr(j);
}


/**
 * Returns the PWM duty cycle, in per cents.
 *
 * @return a floating point number corresponding to the PWM duty cycle, in per cents
 *
 * On failure, throws an exception or returns Y_DUTYCYCLE_INVALID.
 */
double YPwmInput::get_dutyCycle(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::DUTYCYCLE_INVALID;
        }
    }
    return _dutyCycle;
}

/**
 * Returns the PWM pulse length in milliseconds, as a floating point number.
 *
 * @return a floating point number corresponding to the PWM pulse length in milliseconds, as a
 * floating point number
 *
 * On failure, throws an exception or returns Y_PULSEDURATION_INVALID.
 */
double YPwmInput::get_pulseDuration(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::PULSEDURATION_INVALID;
        }
    }
    return _pulseDuration;
}

/**
 * Returns the PWM frequency in Hz.
 *
 * @return a floating point number corresponding to the PWM frequency in Hz
 *
 * On failure, throws an exception or returns Y_FREQUENCY_INVALID.
 */
double YPwmInput::get_frequency(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::FREQUENCY_INVALID;
        }
    }
    return _frequency;
}

/**
 * Returns the PWM period in milliseconds.
 *
 * @return a floating point number corresponding to the PWM period in milliseconds
 *
 * On failure, throws an exception or returns Y_PERIOD_INVALID.
 */
double YPwmInput::get_period(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::PERIOD_INVALID;
        }
    }
    return _period;
}

/**
 * Returns the pulse counter value. Actually that
 * counter is incremented twice per period. That counter is
 * limited  to 1 billion
 *
 * @return an integer corresponding to the pulse counter value
 *
 * On failure, throws an exception or returns Y_PULSECOUNTER_INVALID.
 */
s64 YPwmInput::get_pulseCounter(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::PULSECOUNTER_INVALID;
        }
    }
    return _pulseCounter;
}

int YPwmInput::set_pulseCounter(s64 newval)
{
    string rest_val;
    char buf[32]; sprintf(buf, "%u", (u32)newval); rest_val = string(buf);
    return _setAttr("pulseCounter", rest_val);
}

/**
 * Returns the timer of the pulses counter (ms)
 *
 * @return an integer corresponding to the timer of the pulses counter (ms)
 *
 * On failure, throws an exception or returns Y_PULSETIMER_INVALID.
 */
s64 YPwmInput::get_pulseTimer(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::PULSETIMER_INVALID;
        }
    }
    return _pulseTimer;
}

/**
 * Returns the parameter (frequency/duty cycle, pulse width, edges count) returned by the
 * get_currentValue function and callbacks. Attention
 *
 * @return a value among Y_PWMREPORTMODE_PWM_DUTYCYCLE, Y_PWMREPORTMODE_PWM_FREQUENCY,
 * Y_PWMREPORTMODE_PWM_PULSEDURATION and Y_PWMREPORTMODE_PWM_EDGECOUNT corresponding to the parameter
 * (frequency/duty cycle, pulse width, edges count) returned by the get_currentValue function and callbacks
 *
 * On failure, throws an exception or returns Y_PWMREPORTMODE_INVALID.
 */
Y_PWMREPORTMODE_enum YPwmInput::get_pwmReportMode(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YPwmInput::PWMREPORTMODE_INVALID;
        }
    }
    return _pwmReportMode;
}

/**
 * Modifies the  parameter  type (frequency/duty cycle, pulse width, or edge count) returned by the
 * get_currentValue function and callbacks.
 * The edge count value is limited to the 6 lowest digits. For values greater than one million, use
 * get_pulseCounter().
 *
 * @param newval : a value among Y_PWMREPORTMODE_PWM_DUTYCYCLE, Y_PWMREPORTMODE_PWM_FREQUENCY,
 * Y_PWMREPORTMODE_PWM_PULSEDURATION and Y_PWMREPORTMODE_PWM_EDGECOUNT
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YPwmInput::set_pwmReportMode(Y_PWMREPORTMODE_enum newval)
{
    string rest_val;
    char buf[32]; sprintf(buf, "%d", newval); rest_val = string(buf);
    return _setAttr("pwmReportMode", rest_val);
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
 * Use the method YPwmInput.isOnline() to test if $THEFUNCTION$ is
 * indeed online at a given time. In case of ambiguity when looking for
 * $AFUNCTION$ by logical name, no error is notified: the first instance
 * found is returned. The search is performed first by hardware name,
 * then by logical name.
 *
 * @param func : a string that uniquely characterizes $THEFUNCTION$
 *
 * @return a YPwmInput object allowing you to drive $THEFUNCTION$.
 */
YPwmInput* YPwmInput::FindPwmInput(string func)
{
    YPwmInput* obj = NULL;
    obj = (YPwmInput*) YFunction::_FindFromCache("PwmInput", func);
    if (obj == NULL) {
        obj = new YPwmInput(func);
        YFunction::_AddToCache("PwmInput", func, obj);
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
int YPwmInput::registerValueCallback(YPwmInputValueCallback callback)
{
    string val;
    if (callback != NULL) {
        YFunction::_UpdateValueCallbackList(this, true);
    } else {
        YFunction::_UpdateValueCallbackList(this, false);
    }
    _valueCallbackPwmInput = callback;
    // Immediately invoke value callback with current value
    if (callback != NULL && this->isOnline()) {
        val = _advertisedValue;
        if (!(val == "")) {
            this->_invokeValueCallback(val);
        }
    }
    return 0;
}

int YPwmInput::_invokeValueCallback(string value)
{
    if (_valueCallbackPwmInput != NULL) {
        _valueCallbackPwmInput(this, value);
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
int YPwmInput::registerTimedReportCallback(YPwmInputTimedReportCallback callback)
{
    YSensor* sensor = NULL;
    sensor = this;
    if (callback != NULL) {
        YFunction::_UpdateTimedReportCallbackList(sensor, true);
    } else {
        YFunction::_UpdateTimedReportCallbackList(sensor, false);
    }
    _timedReportCallbackPwmInput = callback;
    return 0;
}

int YPwmInput::_invokeTimedReportCallback(YMeasure value)
{
    if (_timedReportCallbackPwmInput != NULL) {
        _timedReportCallbackPwmInput(this, value);
    } else {
        YSensor::_invokeTimedReportCallback(value);
    }
    return 0;
}

/**
 * Returns the pulse counter value as well as its timer.
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YPwmInput::resetCounter(void)
{
    return this->set_pulseCounter(0);
}

YPwmInput *YPwmInput::nextPwmInput(void)
{
    string  hwid;

    if(YISERR(_nextFunction(hwid)) || hwid=="") {
        return NULL;
    }
    return YPwmInput::FindPwmInput(hwid);
}

YPwmInput* YPwmInput::FirstPwmInput(void)
{
    vector<YFUN_DESCR>   v_fundescr;
    YDEV_DESCR             ydevice;
    string              serial, funcId, funcName, funcVal, errmsg;

    if(YISERR(YapiWrapper::getFunctionsByClass("PwmInput", 0, v_fundescr, sizeof(YFUN_DESCR), errmsg)) ||
       v_fundescr.size() == 0 ||
       YISERR(YapiWrapper::getFunctionInfo(v_fundescr[0], ydevice, serial, funcId, funcName, funcVal, errmsg))) {
        return NULL;
    }
    return YPwmInput::FindPwmInput(serial+"."+funcId);
}

//--- (end of YPwmInput implementation)

//--- (PwmInput functions)
//--- (end of PwmInput functions)
