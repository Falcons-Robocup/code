""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import struct
import msgpack
import zstd

import sys # TODO nasty hack, should move this library into rtdb toolchain
sys.path.append("/home/robocup/falcons/code/packages/facilities/rtdb3/src/tools/rtdb2Tools")
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
            if not self.data.has_key(item.agent):
                self.data[item.agent] = {}
            # store item
            self.data[item.agent][item.key] = item

    def readFrameData(self, raw_data, is_compressed):
        if is_compressed:
            raw_data = zstd.decompress(raw_data)
        data = msgpack.unpackb(raw_data)
        return data[0] # TODO why the nesting? problem in logFileWriter?

    def __str__(self):
        return 'agent=%d age=%7.3fs shared=%d data=%s' % (self.agent, self.age, self.shared, str(self.data))


class RDLHeader:
    def __init__(self, raw_header):
        self.raw_header = raw_header

        self.hostname = unicode(raw_header[0])
        self.creation = RtDBTime(*raw_header[1])
        self.compression = raw_header[2]
        self.duration = raw_header[3]
        self.filename = unicode(raw_header[4])
        self.frequency = raw_header[5]

    def serialize(self):
        return [self.hostname, [int(self.creation.tv_sec), int(self.creation.tv_usec)], self.compression, self.duration, self.filename, self.frequency]

class RDLFile:
    def __init__(self, filename):
        self.filename = filename

        # Filled in by parseRDL()
        self.header = None
        self.frames = []

    def parseRDL(self):
        # Parse the RDL

        # TODO: do not load all at once, not very memory-efficient (try rdlDump on a large file)
        with open(self.filename, "rb") as f:
            self.header = RDLHeader(self.readFrame(f))
            self.frames = []

            while True:
                raw_frame = self.readFrame(f)
                if raw_frame is not None:
                    frame = RDLFrame(raw_frame, self.header.compression)
                    self.frames.append(frame)
                else:
                    break

    def write(self):
        with open(self.filename, "w+b") as f:
            header_serialized = self.header.serialize()
            self.writeFrame(f, header_serialized)
            for frame in self.frames:
                raw_frame = frame.raw_frame # assume unchanged -- TODO!
                self.writeFrame(f, raw_frame)

    def readFrame(self, f):
        framesize = f.read(8)
        if framesize == "":
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

