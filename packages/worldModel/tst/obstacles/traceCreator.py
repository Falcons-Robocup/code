""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 


def writeMeasurementTraceEntry(out_file, index, robotId, uniqueId, timestamp, cameraType, confidence, 
                                azimuth, elevation, radius, cameraX, cameraY, cameraZ, cameraPhi):
    out_file.write('0.0 OM %d %d %d %f %d %f %f %f %f %f %f %f %f\n' % 
               (index, robotId, uniqueId, timestamp, cameraType, confidence, 
                azimuth, elevation, radius, cameraX, cameraY, cameraZ, cameraPhi))


def writeMissingObstacleFile():
    with open('fakeTrace.txt', 'w') as trace_file:
        writeMeasurementTraceEntry(out_file=trace_file, 
                                   index=0,
                                   robotId=0,
                                   uniqueId=0,
                                   timestamp=0.0,
                                   cameraType=0,
                                   confidence=0.0, 
                                   azimuth=0.0,
                                   elevation=0.0, 
                                   radius=2.0,
                                   cameraX=0.0,
                                   cameraY=0.0,
                                   cameraZ=0.0,
                                   cameraPhi=0.0)

        writeMeasurementTraceEntry(out_file=trace_file, 
                                   index=1,
                                   robotId=0,
                                   uniqueId=1,
                                   timestamp=0.0,
                                   cameraType=0,
                                   confidence=0.0, 
                                   azimuth=3.14,
                                   elevation=0.0, 
                                   radius=2.0,
                                   cameraX=0.0,
                                   cameraY=0.0,
                                   cameraZ=0.0,
                                   cameraPhi=0.0)


        for t in range(1, 1000):
            writeMeasurementTraceEntry(out_file=trace_file, 
                                       index=0,
                                       robotId=0,
                                       uniqueId=0,
                                       timestamp=t*0.01,
                                       cameraType=0,
                                       confidence=0.0, 
                                       azimuth=0.0,
                                       elevation=0.0, 
                                       radius=2.0,
                                       cameraX=0.0,
                                       cameraY=0.0,
                                       cameraZ=0.0,
                                       cameraPhi=0.0)


writeMissingObstacleFile()
