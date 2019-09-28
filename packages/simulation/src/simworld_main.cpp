 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * simworld_main.cpp
 *
 *  Created on: Feb 19, 2019
 *      Author: Coen Tempelaars
 */

#include <chrono>
#include <stdexcept>
#include <thread>
#include <iostream>

#include "int/simulationCapabilities.hpp"
#include "int/simworld.hpp"
#include "tracing.hpp"

const static int PERIOD_MS = (1000 / SIMULATION_FREQUENCY);


int main(int argc, char **argv)
{
    try
    {
        INIT_TRACE;
        TRACE_FUNCTION("");

        Simworld simworld;
        simworld.initialize();

        while(true)
        {
            std::chrono::system_clock::time_point timePoint =
                    std::chrono::system_clock::now() + std::chrono::milliseconds(PERIOD_MS);

            simworld.control();
            WRITE_TRACE;

            std::this_thread::sleep_until(timePoint);
        }
    }
    catch(std::exception& e)
    {
        std::cout << "Main loop caught exception: " << e.what() << std::endl;
    }
}
