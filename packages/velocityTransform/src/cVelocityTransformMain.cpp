// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformMain.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/cVelocityTransformMain.hpp"

#include <boost/thread/thread.hpp>

#include "int/algorithms/cTranslateFeedback.hpp"
#include "int/algorithms/cPublishFeedback.hpp"
#include "int/algorithms/cTranslateTarget.hpp"
#include "int/algorithms/cPublishTarget.hpp"

#include "cDiagnostics.hpp"


cVelocityTransformMain::cVelocityTransformMain()
{
    // Empty implementation. Defined to allow this class to be globally defined.
    _feedbackAlgorithm = NULL;
    _setpointAlgorithm = NULL;
    _vtDataClass = NULL;

    setAlgorithms();

    TRACE("INIT");
}

cVelocityTransformMain::~cVelocityTransformMain()
{
}

void cVelocityTransformMain::iterateFeedback()
{
    while (true)
    {
        try
        {

            {
                TRACE_FUNCTION("");

                // Do an execute of the algorithm
                if (_feedbackAlgorithm != NULL)
                {
                    _feedbackAlgorithm->execute();
                }

                WRITE_TRACE;
            }

            // Sleep
            boost::this_thread::sleep_for( boost::chrono::seconds(1800) );
            TRACE("Thread waking up... THIS MAY NOT HAPPEN!");
        }
        catch(boost::thread_interrupted& e)
        {
            // Thread interrupted.
            //TRACE("Thread interrupted!");
        }
        catch(...)
        {
            TRACE("Unknown exception caught!");
        }
    }
}

void cVelocityTransformMain::iterateSetpoint()
{
    while (true)
    {
        try
        {

            {
                TRACE_FUNCTION("");

                // Do an execute of the algorithm
                if (_setpointAlgorithm != NULL)
                {
                    _setpointAlgorithm->execute();
                }
            }

            // Sleep
            boost::this_thread::sleep_for( boost::chrono::seconds(1800) );
            TRACE("Thread waking up... THIS MAY NOT HAPPEN!");
        }
        catch(boost::thread_interrupted& e)
        {
            // Thread interrupted.
            //TRACE("Thread interrupted!");
        }
        catch(...)
        {
            TRACE("Unknown exception caught!");
        }
    }
}

void cVelocityTransformMain::setAlgorithms()
{
    TRACE_FUNCTION("");

    if (_feedbackAlgorithm != NULL)
    {
        // Delete all entries, then delete the algorithm
        while(!_feedbackAlgorithm->_vtBlocks.empty()) delete _feedbackAlgorithm->_vtBlocks.front(), _feedbackAlgorithm->_vtBlocks.pop_front();
        delete _feedbackAlgorithm;
    }
    if (_setpointAlgorithm != NULL)
    {
        // Delete all entries, then delete the algorithm
        while(!_setpointAlgorithm->_vtBlocks.empty()) delete _setpointAlgorithm->_vtBlocks.front(), _setpointAlgorithm->_vtBlocks.pop_front();
        delete _setpointAlgorithm;
    }

    _feedbackAlgorithm = new cAbstractVelocityTransform(this);
    _feedbackAlgorithm->_vtBlocks.push_back(new cTranslateFeedback(this));
    _feedbackAlgorithm->_vtBlocks.push_back(new cPublishFeedback(this));

    _setpointAlgorithm = new cAbstractVelocityTransform(this);
    _setpointAlgorithm->_vtBlocks.push_back(new cTranslateTarget(this));
    _setpointAlgorithm->_vtBlocks.push_back(new cPublishTarget(this));

}
