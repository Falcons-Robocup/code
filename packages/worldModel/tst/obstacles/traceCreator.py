# Copyright 2018 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0



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
