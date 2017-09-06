/*********************************************************************
 *
 * $Id: yocto_cellular.cpp 21511 2015-09-14 16:25:19Z seb $
 *
 * Implements yFindCellular(), the high-level API for Cellular functions
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
#include "yocto_cellular.h"
#include "yapi/yjson.h"
#include "yapi/yapi.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


YCellRecord::YCellRecord(int mcc,int mnc,int lac,int cellId,int dbm,int tad,const string &oper):
    _oper(oper),_mcc(mcc),_mnc(mnc),_lac(lac),_cid(cellId),_dbm(dbm),_tad(tad)
{

}

//--- (generated code: YCellRecord implementation)
// static attributes


string YCellRecord::get_cellOperator(void)
{
    return _oper;
}

int YCellRecord::get_mobileCountryCode(void)
{
    return _mcc;
}

int YCellRecord::get_mobileNetworkCode(void)
{
    return _mnc;
}

int YCellRecord::get_locationAreaCode(void)
{
    return _lac;
}

int YCellRecord::get_cellId(void)
{
    return _cid;
}

int YCellRecord::get_signalStrength(void)
{
    return _dbm;
}

int YCellRecord::get_timingAdvance(void)
{
    return _tad;
}
//--- (end of generated code: YCellRecord implementation)


YCellular::YCellular(const string& func): YFunction(func)
//--- (generated code: Cellular initialization)
    ,_linkQuality(LINKQUALITY_INVALID)
    ,_cellOperator(CELLOPERATOR_INVALID)
    ,_cellIdentifier(CELLIDENTIFIER_INVALID)
    ,_imsi(IMSI_INVALID)
    ,_message(MESSAGE_INVALID)
    ,_pin(PIN_INVALID)
    ,_lockedOperator(LOCKEDOPERATOR_INVALID)
    ,_enableData(ENABLEDATA_INVALID)
    ,_apn(APN_INVALID)
    ,_apnSecret(APNSECRET_INVALID)
    ,_command(COMMAND_INVALID)
    ,_valueCallbackCellular(NULL)
//--- (end of generated code: Cellular initialization)
{
    _className="Cellular";
}

YCellular::~YCellular()
{
//--- (generated code: YCellular cleanup)
//--- (end of generated code: YCellular cleanup)
}
//--- (generated code: YCellular implementation)
// static attributes
const string YCellular::CELLOPERATOR_INVALID = YAPI_INVALID_STRING;
const string YCellular::CELLIDENTIFIER_INVALID = YAPI_INVALID_STRING;
const string YCellular::IMSI_INVALID = YAPI_INVALID_STRING;
const string YCellular::MESSAGE_INVALID = YAPI_INVALID_STRING;
const string YCellular::PIN_INVALID = YAPI_INVALID_STRING;
const string YCellular::LOCKEDOPERATOR_INVALID = YAPI_INVALID_STRING;
const string YCellular::APN_INVALID = YAPI_INVALID_STRING;
const string YCellular::APNSECRET_INVALID = YAPI_INVALID_STRING;
const string YCellular::COMMAND_INVALID = YAPI_INVALID_STRING;

int YCellular::_parseAttr(yJsonStateMachine& j)
{
    if(!strcmp(j.token, "linkQuality")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _linkQuality =  atoi(j.token);
        return 1;
    }
    if(!strcmp(j.token, "cellOperator")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _cellOperator =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "cellIdentifier")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _cellIdentifier =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "imsi")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _imsi =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "message")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _message =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "pin")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _pin =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "lockedOperator")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _lockedOperator =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "enableData")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _enableData =  (Y_ENABLEDATA_enum)atoi(j.token);
        return 1;
    }
    if(!strcmp(j.token, "apn")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _apn =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "apnSecret")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _apnSecret =  _parseString(j);
        return 1;
    }
    if(!strcmp(j.token, "command")) {
        if(yJsonParse(&j) != YJSON_PARSE_AVAIL) goto failed;
        _command =  _parseString(j);
        return 1;
    }
    failed:
    return YFunction::_parseAttr(j);
}


/**
 * Returns the link quality, expressed in percent.
 *
 * @return an integer corresponding to the link quality, expressed in percent
 *
 * On failure, throws an exception or returns Y_LINKQUALITY_INVALID.
 */
int YCellular::get_linkQuality(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::LINKQUALITY_INVALID;
        }
    }
    return _linkQuality;
}

/**
 * Returns the name of the cell operator currently in use.
 *
 * @return a string corresponding to the name of the cell operator currently in use
 *
 * On failure, throws an exception or returns Y_CELLOPERATOR_INVALID.
 */
string YCellular::get_cellOperator(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::CELLOPERATOR_INVALID;
        }
    }
    return _cellOperator;
}

/**
 * Returns the unique identifier of the cellular antenna in use: MCC, MNC, LAC and Cell ID.
 *
 * @return a string corresponding to the unique identifier of the cellular antenna in use: MCC, MNC,
 * LAC and Cell ID
 *
 * On failure, throws an exception or returns Y_CELLIDENTIFIER_INVALID.
 */
string YCellular::get_cellIdentifier(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::CELLIDENTIFIER_INVALID;
        }
    }
    return _cellIdentifier;
}

/**
 * Returns an opaque string if a PIN code has been configured in the device to access
 * the SIM card, or an empty string if none has been configured or if the code provided
 * was rejected by the SIM card.
 *
 * @return a string corresponding to an opaque string if a PIN code has been configured in the device to access
 *         the SIM card, or an empty string if none has been configured or if the code provided
 *         was rejected by the SIM card
 *
 * On failure, throws an exception or returns Y_IMSI_INVALID.
 */
string YCellular::get_imsi(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::IMSI_INVALID;
        }
    }
    return _imsi;
}

/**
 * Returns the latest status message from the wireless interface.
 *
 * @return a string corresponding to the latest status message from the wireless interface
 *
 * On failure, throws an exception or returns Y_MESSAGE_INVALID.
 */
string YCellular::get_message(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::MESSAGE_INVALID;
        }
    }
    return _message;
}

/**
 * Returns an opaque string if a PIN code has been configured in the device to access
 * the SIM card, or an empty string if none has been configured or if the code provided
 * was rejected by the SIM card.
 *
 * @return a string corresponding to an opaque string if a PIN code has been configured in the device to access
 *         the SIM card, or an empty string if none has been configured or if the code provided
 *         was rejected by the SIM card
 *
 * On failure, throws an exception or returns Y_PIN_INVALID.
 */
string YCellular::get_pin(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::PIN_INVALID;
        }
    }
    return _pin;
}

/**
 * Changes the PIN code used by the module to access the SIM card.
 * This function does not change the code on the SIM card itself, but only changes
 * the parameter used by the device to try to get access to it. If the SIM code
 * does not work immediately on first try, it will be automatically forgotten
 * and the message will be set to "Enter SIM PIN". The method should then be
 * invoked again with right correct PIN code. After three failed attempts in a row,
 * the message is changed to "Enter SIM PUK" and the SIM card PUK code must be
 * provided using method sendPUK.
 *
 * Remember to call the saveToFlash() method of the module to save the
 * new value in the device flash.
 *
 * @param newval : a string corresponding to the PIN code used by the module to access the SIM card
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::set_pin(const string& newval)
{
    string rest_val;
    rest_val = newval;
    return _setAttr("pin", rest_val);
}

/**
 * Returns the name of the only cell operator to use if automatic choice is disabled,
 * or an empty string if the SIM card will automatically choose among available
 * cell operators.
 *
 * @return a string corresponding to the name of the only cell operator to use if automatic choice is disabled,
 *         or an empty string if the SIM card will automatically choose among available
 *         cell operators
 *
 * On failure, throws an exception or returns Y_LOCKEDOPERATOR_INVALID.
 */
string YCellular::get_lockedOperator(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::LOCKEDOPERATOR_INVALID;
        }
    }
    return _lockedOperator;
}

/**
 * Changes the name of the cell operator to be used. If the name is an empty
 * string, the choice will be made automatically based on the SIM card. Otherwise,
 * the selected operator is the only one that will be used.
 *
 * @param newval : a string corresponding to the name of the cell operator to be used
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::set_lockedOperator(const string& newval)
{
    string rest_val;
    rest_val = newval;
    return _setAttr("lockedOperator", rest_val);
}

/**
 * Returns the condition for enabling IP data services (GPRS).
 * When data services are disabled, SMS are the only mean of communication.
 *
 * @return a value among Y_ENABLEDATA_HOMENETWORK, Y_ENABLEDATA_ROAMING and Y_ENABLEDATA_NEVER
 * corresponding to the condition for enabling IP data services (GPRS)
 *
 * On failure, throws an exception or returns Y_ENABLEDATA_INVALID.
 */
Y_ENABLEDATA_enum YCellular::get_enableData(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::ENABLEDATA_INVALID;
        }
    }
    return _enableData;
}

/**
 * Changes the condition for enabling IP data services (GPRS).
 * The service can be either fully deactivated, or limited to the SIM home network,
 * or enabled for all partner networks (roaming). Caution: enabling data services
 * on roaming networks may cause prohibitive communication costs !
 *
 * When data services are disabled, SMS are the only mean of communication.
 *
 * @param newval : a value among Y_ENABLEDATA_HOMENETWORK, Y_ENABLEDATA_ROAMING and Y_ENABLEDATA_NEVER
 * corresponding to the condition for enabling IP data services (GPRS)
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::set_enableData(Y_ENABLEDATA_enum newval)
{
    string rest_val;
    char buf[32]; sprintf(buf, "%d", newval); rest_val = string(buf);
    return _setAttr("enableData", rest_val);
}

/**
 * Returns the Access Point Name (APN) to be used, if needed.
 * When left blank, the APN suggested by the cell operator will be used.
 *
 * @return a string corresponding to the Access Point Name (APN) to be used, if needed
 *
 * On failure, throws an exception or returns Y_APN_INVALID.
 */
string YCellular::get_apn(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::APN_INVALID;
        }
    }
    return _apn;
}

/**
 * Returns the Access Point Name (APN) to be used, if needed.
 * When left blank, the APN suggested by the cell operator will be used.
 *
 * @param newval : a string
 *
 * @return YAPI_SUCCESS if the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::set_apn(const string& newval)
{
    string rest_val;
    rest_val = newval;
    return _setAttr("apn", rest_val);
}

/**
 * Returns an opaque string if APN authentication parameters have been configured
 * in the device, or an empty string otherwise.
 * To configure these parameters, use set_apnAuth().
 *
 * @return a string corresponding to an opaque string if APN authentication parameters have been configured
 *         in the device, or an empty string otherwise
 *
 * On failure, throws an exception or returns Y_APNSECRET_INVALID.
 */
string YCellular::get_apnSecret(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::APNSECRET_INVALID;
        }
    }
    return _apnSecret;
}

int YCellular::set_apnSecret(const string& newval)
{
    string rest_val;
    rest_val = newval;
    return _setAttr("apnSecret", rest_val);
}

string YCellular::get_command(void)
{
    if (_cacheExpiration <= YAPI::GetTickCount()) {
        if (this->load(YAPI::DefaultCacheValidity) != YAPI_SUCCESS) {
            return YCellular::COMMAND_INVALID;
        }
    }
    return _command;
}

int YCellular::set_command(const string& newval)
{
    string rest_val;
    rest_val = newval;
    return _setAttr("command", rest_val);
}

/**
 * Retrieves a cellular interface for a given identifier.
 * The identifier can be specified using several formats:
 * <ul>
 * <li>FunctionLogicalName</li>
 * <li>ModuleSerialNumber.FunctionIdentifier</li>
 * <li>ModuleSerialNumber.FunctionLogicalName</li>
 * <li>ModuleLogicalName.FunctionIdentifier</li>
 * <li>ModuleLogicalName.FunctionLogicalName</li>
 * </ul>
 *
 * This function does not require that the cellular interface is online at the time
 * it is invoked. The returned object is nevertheless valid.
 * Use the method YCellular.isOnline() to test if the cellular interface is
 * indeed online at a given time. In case of ambiguity when looking for
 * a cellular interface by logical name, no error is notified: the first instance
 * found is returned. The search is performed first by hardware name,
 * then by logical name.
 *
 * @param func : a string that uniquely characterizes the cellular interface
 *
 * @return a YCellular object allowing you to drive the cellular interface.
 */
YCellular* YCellular::FindCellular(string func)
{
    YCellular* obj = NULL;
    obj = (YCellular*) YFunction::_FindFromCache("Cellular", func);
    if (obj == NULL) {
        obj = new YCellular(func);
        YFunction::_AddToCache("Cellular", func, obj);
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
int YCellular::registerValueCallback(YCellularValueCallback callback)
{
    string val;
    if (callback != NULL) {
        YFunction::_UpdateValueCallbackList(this, true);
    } else {
        YFunction::_UpdateValueCallbackList(this, false);
    }
    _valueCallbackCellular = callback;
    // Immediately invoke value callback with current value
    if (callback != NULL && this->isOnline()) {
        val = _advertisedValue;
        if (!(val == "")) {
            this->_invokeValueCallback(val);
        }
    }
    return 0;
}

int YCellular::_invokeValueCallback(string value)
{
    if (_valueCallbackCellular != NULL) {
        _valueCallbackCellular(this, value);
    } else {
        YFunction::_invokeValueCallback(value);
    }
    return 0;
}

/**
 * Sends a PUK code to unlock the SIM card after three failed PIN code attempts, and
 * setup a new PIN into the SIM card. Only ten consecutives tentatives are permitted:
 * after that, the SIM card will be blocked permanently without any mean of recovery
 * to use it again. Note that after calling this method, you have usually to invoke
 * method set_pin() to tell the YoctoHub which PIN to use in the future.
 *
 * @param puk : the SIM PUK code
 * @param newPin : new PIN code to configure into the SIM card
 *
 * @return YAPI_SUCCESS when the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::sendPUK(string puk,string newPin)
{
    string gsmMsg;
    gsmMsg = this->get_message();
    if (!(gsmMsg == "Enter SIM PUK")) {
        _throw(YAPI_INVALID_ARGUMENT,"PUK not expected at this time");
        return YAPI_INVALID_ARGUMENT;
    }
    if (newPin == "") {
        return this->set_command(YapiWrapper::ysprintf("AT+CPIN=%s,0000;+CLCK=SC,0,0000",puk.c_str()));
    }
    return this->set_command(YapiWrapper::ysprintf("AT+CPIN=%s,%s",puk.c_str(),newPin.c_str()));
}

/**
 * Configure authentication parameters to connect to the APN. Both
 * PAP and CHAP authentication are supported.
 *
 * @param username : APN username
 * @param password : APN password
 *
 * @return YAPI_SUCCESS when the call succeeds.
 *
 * On failure, throws an exception or returns a negative error code.
 */
int YCellular::set_apnAuth(string username,string password)
{
    return this->set_apnSecret(YapiWrapper::ysprintf("%s,%s",username.c_str(),password.c_str()));
}

/**
 * Sends an AT command to the GSM module and returns the command output.
 * The command will only execute when the GSM module is in standard
 * command state, and should leave it in the exact same state.
 * Use this function with great care !
 *
 * @param cmd : the AT command to execute, like for instance: "+CCLK?".
 *
 * @return a string with the result of the commands. Empty lines are
 *         automatically removed from the output.
 */
string YCellular::_AT(string cmd)
{
    int chrPos = 0;
    int cmdLen = 0;
    int waitMore = 0;
    string res;
    string buff;
    int bufflen = 0;
    string buffstr;
    int buffstrlen = 0;
    int idx = 0;
    int suffixlen = 0;
    // quote dangerous characters used in AT commands
    cmdLen = (int)(cmd).length();
    chrPos = _ystrpos(cmd, "#");
    while (chrPos >= 0) {
        cmd = YapiWrapper::ysprintf("%s%c23%s", (cmd).substr( 0, chrPos).c_str(), 37,(cmd).substr( chrPos+1, cmdLen-chrPos-1).c_str());
        cmdLen = cmdLen + 2;
        chrPos = _ystrpos(cmd, "#");
    }
    chrPos = _ystrpos(cmd, "+");
    while (chrPos >= 0) {
        cmd = YapiWrapper::ysprintf("%s%c2B%s", (cmd).substr( 0, chrPos).c_str(), 37,(cmd).substr( chrPos+1, cmdLen-chrPos-1).c_str());
        cmdLen = cmdLen + 2;
        chrPos = _ystrpos(cmd, "+");
    }
    chrPos = _ystrpos(cmd, "=");
    while (chrPos >= 0) {
        cmd = YapiWrapper::ysprintf("%s%c3D%s", (cmd).substr( 0, chrPos).c_str(), 37,(cmd).substr( chrPos+1, cmdLen-chrPos-1).c_str());
        cmdLen = cmdLen + 2;
        chrPos = _ystrpos(cmd, "=");
    }
    cmd = YapiWrapper::ysprintf("at.txt?cmd=%s",cmd.c_str());
    res = YapiWrapper::ysprintf("");
    // max 2 minutes (each iteration may take up to 5 seconds if waiting)
    waitMore = 24;
    while (waitMore > 0) {
        buff = this->_download(cmd);
        bufflen = (int)(buff).size();
        buffstr = buff;
        buffstrlen = (int)(buffstr).length();
        idx = bufflen - 1;
        while ((idx > 0) && (((u8)buff[idx]) != 64) && (((u8)buff[idx]) != 10) && (((u8)buff[idx]) != 13)) {
            idx = idx - 1;
        }
        if (((u8)buff[idx]) == 64) {
            suffixlen = bufflen - idx;
            cmd = YapiWrapper::ysprintf("at.txt?cmd=%s",(buffstr).substr( buffstrlen - suffixlen, suffixlen).c_str());
            buffstr = (buffstr).substr( 0, buffstrlen - suffixlen);
            waitMore = waitMore - 1;
        } else {
            waitMore = 0;
        }
        res = YapiWrapper::ysprintf("%s%s", res.c_str(),buffstr.c_str());
    }
    return res;
}

/**
 * Returns the list detected cell operators in the neighborhood.
 * This function will typically take between 30 seconds to 1 minute to
 * return. Note that any SIM card can usually only connect to specific
 * operators. All networks returned by this function might therefore
 * not be available for connection.
 *
 * @return a list of string (cell operator names).
 */
vector<string> YCellular::get_availableOperators(void)
{
    string cops;
    int idx = 0;
    int slen = 0;
    vector<string> res;
    // may throw an exception
    cops = this->_AT("+COPS=?");
    slen = (int)(cops).length();
    res.clear();
    idx = _ystrpos(cops, "(");
    while (idx >= 0) {
        slen = slen - (idx+1);
        cops = (cops).substr( idx+1, slen);
        idx = _ystrpos(cops, "\"");
        if (idx > 0) {
            slen = slen - (idx+1);
            cops = (cops).substr( idx+1, slen);
            idx = _ystrpos(cops, "\"");
            if (idx > 0) {
                res.push_back((cops).substr( 0, idx));
            }
        }
        idx = _ystrpos(cops, "(");
    }
    return res;
}

/**
 * Returns a list of nearby cellular antennas, as required for quick
 * geolocation of the device. The first cell listed is the serving
 * cell, and the next ones are the neighboor cells reported by the
 * serving cell.
 *
 * @return a list of YCellRecords.
 */
vector<YCellRecord> YCellular::quickCellSurvey(void)
{
    string moni;
    vector<string> recs;
    int llen = 0;
    string mccs;
    int mcc = 0;
    string mncs;
    int mnc = 0;
    int lac = 0;
    int cellId = 0;
    string dbms;
    int dbm = 0;
    string tads;
    int tad = 0;
    string oper;
    vector<YCellRecord> res;
    // may throw an exception
    moni = this->_AT("+CCED=0;#MONI=7;#MONI");
    mccs = (moni).substr(7, 3);
    if ((mccs).substr(0, 1) == "0") {
        mccs = (mccs).substr(1, 2);
    }
    if ((mccs).substr(0, 1) == "0") {
        mccs = (mccs).substr(1, 1);
    }
    mcc = atoi((mccs).c_str());
    mncs = (moni).substr(11, 3);
    if ((mncs).substr(2, 1) == ",") {
        mncs = (mncs).substr(0, 2);
    }
    if ((mncs).substr(0, 1) == "0") {
        mncs = (mncs).substr(1, (int)(mncs).length()-1);
    }
    mnc = atoi((mncs).c_str());
    recs = _strsplit(moni,'#');
    // process each line in turn
    res.clear();
    for (unsigned ii = 0; ii < recs.size(); ii++) {
        llen = (int)(recs[ii]).length() - 2;
        if (llen >= 44) {
            if ((recs[ii]).substr(41, 3) == "dbm") {
                lac = (int)strtoul((recs[ii]).substr(16, 4).c_str(), NULL, 16);
                cellId = (int)strtoul((recs[ii]).substr(23, 4).c_str(), NULL, 16);
                dbms = (recs[ii]).substr(37, 4);
                if ((dbms).substr(0, 1) == " ") {
                    dbms = (dbms).substr(1, 3);
                }
                dbm = atoi((dbms).c_str());
                if (llen > 66) {
                    tads = (recs[ii]).substr(54, 2);
                    if ((tads).substr(0, 1) == " ") {
                        tads = (tads).substr(1, 3);
                    }
                    tad = atoi((tads).c_str());
                    oper = (recs[ii]).substr(66, llen-66);
                } else {
                    tad = -1;
                    oper = "";
                }
                if (lac < 65535) {
                    res.push_back(YCellRecord(mcc,mnc,lac,cellId,dbm,tad,oper));
                }
            }
        }
        ;;
    }
    return res;
}

YCellular *YCellular::nextCellular(void)
{
    string  hwid;

    if(YISERR(_nextFunction(hwid)) || hwid=="") {
        return NULL;
    }
    return YCellular::FindCellular(hwid);
}

YCellular* YCellular::FirstCellular(void)
{
    vector<YFUN_DESCR>   v_fundescr;
    YDEV_DESCR             ydevice;
    string              serial, funcId, funcName, funcVal, errmsg;

    if(YISERR(YapiWrapper::getFunctionsByClass("Cellular", 0, v_fundescr, sizeof(YFUN_DESCR), errmsg)) ||
       v_fundescr.size() == 0 ||
       YISERR(YapiWrapper::getFunctionInfo(v_fundescr[0], ydevice, serial, funcId, funcName, funcVal, errmsg))) {
        return NULL;
    }
    return YCellular::FindCellular(serial+"."+funcId);
}

//--- (end of generated code: YCellular implementation)

//--- (generated code: Cellular functions)
//--- (end of generated code: Cellular functions)
