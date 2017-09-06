/*********************************************************************
 *
 * $Id: yocto_audioin.cpp 22191 2015-12-02 06:49:31Z mvuilleu $
 *
 * Implements yFindAudioIn(), the high-level API for AudioIn functions
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
#include "yocto_audioin.h"
#include "yapi/yjson.h"
#include "yapi/yapi.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

YAudioIn::YAudioIn(const string& func): YFunction(func)
//--- (AudioIn initialization)
    ,_volume(VOLUME_INVALID)
    ,_mute(MUTE_INVALID)
    ,_volumeRange(VOLUMERANGE_INVALID)
    ,_signal(SIGNAL_INVALID)
    ,_noSignalFor(NOSIGNALFOR_INVALID)
    ,_valueCallbackAudioIn(NULL)
//--- (end of AudioIn initialization)
{
    _className="AudioIn";
}

YAudioIn::~YAudioIn()
{
//--- (YAudioIn cleanup)
//--- (end of YAudioIn cleanup)
}
//--- (YAudioIn implementation)
// static attributes
const string YAudioIn::VOLUMERANGE_INVALID = YAPI_INVALID_STRING;

int YAudioIn::_parseAttr(yJsonStateMachine& j)
{
    if(!strcmp(j.token, "volume")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _volume =  atoi(j.token);
        return 1;
    }
    if(!strcmp(j.token, "mute")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _mute =  (Y_MUTE_enum)atoi(j.token);
        return 1;
    }
    if(!strcmp(j.token, "volumeRange")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _volumeRange =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "signal")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _signal =  atoi(j.token);
        return 1;
    }
    if(!strcmp(j.token, "noSignalFor")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _noSignalFor =  atoi(j.token);
        return 1;
    }
    failed:
    return YFunction::_parseAttr(j);
}


/**
 * Returns audio input gain, in per cents.
 *
 * @return an integer corresponding to audio input gain, in per cents
 *
 * On failure, throws an exception or returns Y_VOLUME_INVALID.
 */
int YAudioIn::get_volume(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YAudioIn::VOLUME_INVALID;
        }
    }
    return _volume;
}

/**
 * Changes audio input gain, in per cents.
 *
 * @param newval : an integer corresponding to audio input gain, in per cents
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YAudioIn::set_volume(int newval)
{
    string rest_val;
    char buf[32]; sprintf(buf, "%d", newval); rest_val = string(buf);
    return _setAttr("volume", rest_val);
}

/**
 * Returns the state of the mute function.
 *
 * @return either Y_MUTE_FALSE or Y_MUTE_TRUE, according to the state of the mute function
 *
 * On failure, throws an exception or returns Y_MUTE_INVALID.
 */
Y_MUTE_enum YAudioIn::get_mute(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YAudioIn::MUTE_INVALID;
        }
    }
    return _mute;
}

/**
 * Changes the state of the mute function. Remember to call the matching module
 * saveToFlash() method to save the setting permanently.
 *
 * @param newval : either Y_MUTE_FALSE or Y_MUTE_TRUE, according to the state of the mute function
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YAudioIn::set_mute(Y_MUTE_enum newval)
{
    string rest_val;
    rest_val = (newval>0 ? "1" : "0");
    return _setAttr("mute", rest_val);
}

/**
 * Returns the supported volume range. The low value of the
 * range corresponds to the minimal audible value. To
 * completely mute the sound, use set_mute()
 * instead of the set_volume().
 *
 * @return a string corresponding to the supported volume range
 *
 * On failure, throws an exception or returns Y_VOLUMERANGE_INVALID.
 */
string YAudioIn::get_volumeRange(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YAudioIn::VOLUMERANGE_INVALID;
        }
    }
    return _volumeRange;
}

/**
 * Returns the detected input signal level.
 *
 * @return an integer corresponding to the detected input signal level
 *
 * On failure, throws an exception or returns Y_SIGNAL_INVALID.
 */
int YAudioIn::get_signal(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YAudioIn::SIGNAL_INVALID;
        }
    }
    return _signal;
}

/**
 * Returns the number of seconds elapsed without detecting a signal
 *
 * @return an integer corresponding to the number of seconds elapsed without detecting a signal
 *
 * On failure, throws an exception or returns Y_NOSIGNALFOR_INVALID.
 */
int YAudioIn::get_noSignalFor(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YAudioIn::NOSIGNALFOR_INVALID;
        }
    }
    return _noSignalFor;
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
 * Use the method YAudioIn.isOnline() to test if $THEFUNCTION$ is
 * indeed online at a given time. In case of ambiguity when looking for
 * $AFUNCTION$ by logical name, no error is notified: the first instance
 * found is returned. The search is performed first by hardware name,
 * then by logical name.
 *
 * @param func : a string that uniquely characterizes $THEFUNCTION$
 *
 * @return a YAudioIn object allowing you to drive $THEFUNCTION$.
 */
YAudioIn* YAudioIn::FindAudioIn(string func)
{
    YAudioIn* obj = NULL;
    obj = (YAudioIn*) YFunction::_FindFromCache("AudioIn", func);
    if (obj == NULL) {
        obj = new YAudioIn(func);
        YFunction::_AddToCache("AudioIn", func, obj);
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
int YAudioIn::registerValueCallback(YAudioInValueCallback callback)
{
    string val;
    if (callback != NULL) {
        YFunction::_UpdateValueCallbackList(this, true);
    } else {
        YFunction::_UpdateValueCallbackList(this, false);
    }
    _valueCallbackAudioIn = callback;
    // Immediately invoke value callback with current value
    if (callback != NULL && this->isOnline()) {
        val = _advertisedValue;
        if (!(val == "")) {
            this->_invokeValueCallback(val);
        }
    }
    return 0;
}

int YAudioIn::_invokeValueCallback(string value)
{
    if (_valueCallbackAudioIn != NULL) {
        _valueCallbackAudioIn(this, value);
    } else {
        YFunction::_invokeValueCallback(value);
    }
    return 0;
}

YAudioIn *YAudioIn::nextAudioIn(void)
{
    string  hwid;

    if(YISERR(_nextFunction(hwid)) || hwid=="") {
        return NULL;
    }
    return YAudioIn::FindAudioIn(hwid);
}

YAudioIn* YAudioIn::FirstAudioIn(void)
{
    vector<YFUN_DESCR>   v_fundescr;
    YDEV_DESCR             ydevice;
    string              serial, funcId, funcName, funcVal, errmsg;

    if(YISERR(YapiWrapper::getFunctionsByClass("AudioIn", 0, v_fundescr, sizeof(YFUN_DESCR), errmsg)) ||
       v_fundescr.size() == 0 ||
       YISERR(YapiWrapper::getFunctionInfo(v_fundescr[0], ydevice, serial, funcId, funcName, funcVal, errmsg))) {
        return NULL;
    }
    return YAudioIn::FindAudioIn(serial+"."+funcId);
}

//--- (end of YAudioIn implementation)

//--- (AudioIn functions)
//--- (end of AudioIn functions)
