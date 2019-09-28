 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cVelocityControlMain.cpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#include "int/cVelocityControlMain.hpp"

#include <boost/thread/thread.hpp>

#include "int/algorithms/cTranslateFeedback.hpp"
#include "int/algorithms/cPublishFeedback.hpp"
#include "int/algorithms/cTranslateTarget.hpp"
#include "int/algorithms/cPublishTarget.hpp"

#include "cDiagnostics.hpp"


cVelocityControlMain::cVelocityControlMain()
{
    // Empty implementation. Defined to allow this class to be globally defined.
    _feedbackAlgorithm = NULL;
    _setpointAlgorithm = NULL;
    _vcDataClass = NULL;

    setAlgorithms();

    TRACE("INIT");
}

cVelocityControlMain::~cVelocityControlMain()
{
}

void cVelocityControlMain::iterateFeedback()
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

void cVelocityControlMain::iterateSetpoint()
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

void cVelocityControlMain::setAlgorithms()
{
    TRACE_FUNCTION("");

    if (_feedbackAlgorithm != NULL)
    {
        // Delete all entries, then delete the algorithm
        while(!_feedbackAlgorithm->_vcBlocks.empty()) delete _feedbackAlgorithm->_vcBlocks.front(), _feedbackAlgorithm->_vcBlocks.pop_front();
        delete _feedbackAlgorithm;
    }
    if (_setpointAlgorithm != NULL)
    {
        // Delete all entries, then delete the algorithm
        while(!_setpointAlgorithm->_vcBlocks.empty()) delete _setpointAlgorithm->_vcBlocks.front(), _setpointAlgorithm->_vcBlocks.pop_front();
        delete _setpointAlgorithm;
    }

    _feedbackAlgorithm = new cAbstractVelocityControl(this);
    _feedbackAlgorithm->_vcBlocks.push_back(new cTranslateFeedback(this));
    _feedbackAlgorithm->_vcBlocks.push_back(new cPublishFeedback(this));

    _setpointAlgorithm = new cAbstractVelocityControl(this);
    _setpointAlgorithm->_vcBlocks.push_back(new cTranslateTarget(this));
    _setpointAlgorithm->_vcBlocks.push_back(new cPublishTarget(this));

}
