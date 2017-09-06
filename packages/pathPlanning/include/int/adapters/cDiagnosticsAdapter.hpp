 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsAdapter.hpp
 *
 *  Created on: Sept 05, 2015
 *      Author: Erik Kouters
 */

#ifndef CDIAGNOSTICSADAPTER_HPP_
#define CDIAGNOSTICSADAPTER_HPP_

/*!
 * \brief The singleton adapter class dedicated to publishing information for diagnostics.
 */
#include <string>
#include <map>
#include <vector>
#include "rosMsgs/t_diag_pathpl.h"
#include "rosMsgs/t_pp_plotdata.h"
#include "cDiagnostics.hpp"
#include "int/cPathPlanningTypes.hpp"
#include "polygon2D.hpp"

class cDiagnosticsAdapter : public diagnostics::cDiagnosticsSender<rosMsgs::t_diag_pathpl>
{
    public:
        /*!
         * \brief The constructor of cDiagnosticsAdapter
         */
        static cDiagnosticsAdapter& getInstance()
        {
            static cDiagnosticsAdapter instance; 
            return instance;
        }
        
        void setTarget(double x, double y, double phi);
        void setSubTarget(double x, double y);
        void setObstacles(const std::vector<pp_obstacle_struct_t> obstacles, const std::vector<polygon2D>& areas, std::vector<linepoint2D>& projectedSpeedVectors);
        void setActive(bool active);

        //Data for kst plot
        void initializePlotData();
        void setPlotData(pp_plot_data_struct_t plotData );

    private:
        cDiagnosticsAdapter();
        boost::shared_ptr<ros::NodeHandle> _hROS;
        ros::Publisher  _pPlotData;
};

#endif /* CDIAGNOSTICSADAPTER_HPP_ */

