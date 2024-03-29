# Copyright 2019-2022 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
import struct
import msgpack
import zstd
from collections import defaultdict
from copy import copy

import falconspy
from rtdb2 import RtDBTime, RtDBItem, RtDBFrameItem

class RDLFrame:
    def __init__(self, raw_frame, is_compressed):
        self.raw_frame = raw_frame

        self.age = raw_frame[0]
        try:
            self.framedata = self.readFrameData(raw_frame[1], is_compressed)
        except Exception as e:
            raise RuntimeError('Unexpected error while trying to read frame data: {}, raw_frame={}'.format(str(e)), str(raw_frame))

        self.data = {}

        # As final step, deserialize the values of all rtdb keys in this frame
        for raw_item in self.framedata:
            item = RtDBFrameItem()
            item.fromArray(raw_item)
            # create agent dict if not existing
            if not item.agent in self.data.keys():
                self.data[item.agent] = {}
            # store item
            self.data[item.agent][item.key] = item

    def readFrameData(self, raw_data, is_compressed):
        if is_compressed:
            raw_data = zstd.decompress(raw_data)
        data = msgpack.unpackb(raw_data)
        return data[0] # TODO why the nesting? problem in logFileWriter?

    def __str__(self):
        return 'age=%7.3fs data=%s' % (self.age, str(self.data))


class RDLHeader:
    def __init__(self, raw_header):
        self.raw_header = raw_header

        self.hostname = str(raw_header[0], "utf-8")
        self.creation = RtDBTime(*raw_header[1])
        self.compression = raw_header[2]
        self.duration = raw_header[3]
        self.filename = str(raw_header[4], "utf-8")
        self.frequency = raw_header[5]

        if (len(raw_header) >= 8):
            self.commitCode = str(raw_header[6], "utf-8")
            self.commitTeamplayData = str(raw_header[7], "utf-8")
        else:
            self.commitCode = 'old'
            self.commitTeamplayData = 'old'


    def serialize(self):
        return [    self.hostname, 
                    [   int(self.creation.tv_sec), 
                        int(self.creation.tv_usec) ], 
                    self.compression, 
                    self.duration, 
                    self.filename, 
                    self.frequency,
                    self.commitCode,
                    self.commitTeamplayData ]

class RDLFile:
    def __init__(self, filename):
        self.filename = filename

        # Filled in by parseRDL()
        self.header = None
        self.frames = []

        self.rdl_file = None

    def parseRDL(self, ageMin = None, ageMax = None):
        # Parse the RDL
        
        if ageMin is None:
            ageMin = 0
        if ageMax is None:
            ageMax = 1e9
        
        # TODO: do not load all at once, not very memory-efficient (try rdlDump on a large file)
        with open(self.filename, "rb") as f:
            self.header = RDLHeader(self.readFrame(f))
            self.frames = []

            while True:
                raw_frame = self.readFrame(f)
                if raw_frame is not None:
                    frame_age = raw_frame[0]
                    if frame_age >= ageMin and frame_age <= ageMax:
                        frame = RDLFrame(raw_frame, self.header.compression)
                        self.frames.append(frame)
                else:
                    break

    def openRDL(self):
        self.rdl_file = open(self.filename, "rb")
        self.header = RDLHeader(self.readFrame(self.rdl_file))

    def readNextFrame(self):
        raw_frame = self.readFrame(self.rdl_file)
        if raw_frame is not None:
            frame = RDLFrame(raw_frame, self.header.compression)
            return frame
        return None

    def write(self):
        with open(self.filename, "w+b") as f:
            header_serialized = self.header.serialize()
            self.writeFrame(f, header_serialized)
            for frame in self.frames:
                raw_frame = frame.raw_frame # assume unchanged -- TODO!
                self.writeFrame(f, raw_frame)

    def readFrame(self, f):
        framesize = f.read(8)
        if framesize == b"":
            return None
        else:
            framesize = struct.unpack('L', framesize)[0]
            if framesize > 1e6:
                raise RuntimeError('Unexpected large frame size: {}'.format(str(framesize)))
            if framesize < 0:
                raise RuntimeError('Unexpected negative frame size: {}'.format(str(framesize)))
            framedata = f.read(framesize)
            unpacked_frame = msgpack.unpackb(framedata)
            return unpacked_frame

    def writeFrame(self, fp, frame):
        # option use_single_float is required to not get 8-byte floats, which C++ doesn't like
        # furthermore, unicode() and option use_bin_type are needed for consistent string encoding, see
        # https://github.com/msgpack/msgpack-python/issues/162
        packed_frame = msgpack.packb(frame, use_single_float=True, use_bin_type=True)
        framesize = len(packed_frame)
        fp.write(struct.pack('L', framesize))
        fp.write(packed_frame)


def RDLResample(rdl, frequency):
    """
    Given an RDL object, resample frames to given frequency.
    Useful for analysis when condensing high-frequent data into a low-frequent dataframe.
    """
    # hack for stimulation usecase, where item timestamps are much fresher than frame timestamps (age)
    useItemTimestamps = "_stim" not in rdl.filename
    # prepare resulting object
    r = RDLFile(rdl.filename)
    r.header = rdl.header
    r.header.frequency = frequency
    t0 = r.header.creation
    age = 0.0
    dt = 1.0 / frequency
    n = 0
    # helper functions
    def resetFrame(f, age):
        f.raw_frame = None
        f.framedata = None
        f.age = age
        f.data = defaultdict(lambda : {})
    def getFrameItems(f):
        return [(a,k,f.data[a][k]) for a in f.data.keys() for k in f.data[a].keys()]
    def mergeFrame(f, items):
        # merge items in frame, remaining items probably belong to next frame
        rem = []
        for item in items:
            (a,k,i) = item
            if useItemTimestamps:
                t = i.timestamp[0] + 1e-6 * i.timestamp[1]
            else:
                t = f.age # more inaccurate, but needed for instance for stimulated RDL ...
            if t <= f.age + float(t0):
                f.data[a][k] = i
            else:
                rem.append(item)
        return rem
    def storeFrame():
        r.frames.append(copy(newFrame))
    # iterate
    newFrame = copy(rdl.frames[0])
    resetFrame(newFrame, 0.0)
    items = []
    for oldFrame in rdl.frames:
        # get new items
        items += getFrameItems(oldFrame)
        # merge items into running frame if their timestamp matches, return remainder
        items = mergeFrame(newFrame, items)
        # check if we can close running frame
        if oldFrame.age > age:
            # write new frame
            storeFrame()
            # advance to next timestamp
            n += 1
            age = n * dt
            # reset frame
            resetFrame(newFrame, age)
    # finish
    storeFrame()
    return r

