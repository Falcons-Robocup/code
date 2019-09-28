""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 import lmdb
import msgpack
import math
import sys
import struct
import datetime, time
import os
import shm

RTDB2_DEFAULT_PATH = "/tmp/rtdb2_storage"


class RtDBTime():
    # TODO: behave as (or inherit) datetime and add serialize/deserialize?
    def __init__(self, s=0, u=0):
        self.tv_sec = int(s)
        self.tv_usec = int(u)
    def now(self):
        t = time.time()
        self.tv_sec = int(math.floor(t))
        self.tv_usec = int(round(1e6 * (t - self.tv_sec)))
    def __add__(self, other):
        t = datetime.datetime.fromtimestamp(self.tv_sec + 1e-6*self.tv_usec)
        t = t + datetime.timedelta(seconds=other)
        self.tv_sec = int(time.mktime(t.timetuple()))
        self.tv_usec = int(t.microsecond)
        return self

def now():
    t = RtDBTime()
    t.now()
    return [t.tv_sec, t.tv_usec]


class RtDB2Store():
    def __init__(self, path, readonly=True):
        self.path = path
        self.readonly = readonly
        self.rtdb_instances = {}
        # initialize
        self.refresh_rtdb_instances()

    def closeAll(self):
        for rtdbInstance in self.rtdb_instances.values():
            rtdbInstance.close()

    def refresh_rtdb_instances(self):

        # Check if path exists
        if not os.path.isdir(self.path):
            self.rtdb_instances.clear()
            return

        dbsOnDisk = os.listdir(self.path)

        # agent{,_sync}{0-9}
        for db in dbsOnDisk:

            # Skip if already exists
            if db in self.rtdb_instances.keys():
                continue

            # Open database connection and add to dict
            self.rtdb_instances[db] = RtDB2Instance(self, self.path, db, self.readonly)

        # Remove environments that no longer exist
        for old in self.rtdb_instances.keys():
            if old not in dbsOnDisk:
                self.rtdb_instances.pop(old)

    def get(self, agent, key):
        # Dispatch to appropriate database
        db = "agent" + str(agent)
        if db in self.rtdb_instances.keys():
            return self.rtdb_instances[db].get(key)
        return None

    def put(self, agent, key, value):
        # Dispatch to appropriate database
        db = "agent" + str(agent)
        if db in self.rtdb_instances.keys():
            return self.rtdb_instances[db].put(key, value)
        return None

    def getAllRtDBItems(self):
        # Refresh all RtDB instances
        self.refresh_rtdb_instances()

        # Get all data from all RtDB instances in the form of RtDBFrameItems
        list_items = []
        for rtdbInstance in self.rtdb_instances.values():
            list_items.extend( rtdbInstance.getRtDBItems() )

        return list_items

    def wakeWaitingConsumers(self, agent, key):
        # Get sync database
        agent = "agent_sync" + str(agent)
        if agent in self.rtdb_instances.keys():

            syncDB = self.rtdb_instances[agent]

            # Get list of RtDB2SyncPoint
            data = syncDB.get_and_clear(key)

            if data is not None:

                # Free the list of semaphores
                for rtdb2SyncPoint in data:
                    sem = shm.semaphore(rtdb2SyncPoint["sem_ID"])
                    sem.V()

class RtDB2Instance():
    def __init__(self, store, path, name, readonly):

        self.store = store
        self.path = path
        self.readonly = readonly

        # in rtdb v2 we would have: agent_{local,shared,sync}{0-9}
        # now: local/shared has become attribute of items in agentX
        # sync database remains separated though
        self.name = name
        self.agent = int(name[-1])

        # Open database connection
        try:
            lock = not self.readonly # ?!? vague: if not using this flag, rtop would interfere with running SW, causing LMDB error -30783 (bad RSLOT))
            self.conn = lmdb.open(os.path.join(path, name), readonly=readonly, lock=lock)
        except:
            print "RtDB Opening Error: %s does not seem a valid storage or missing permissions! (name=%s)" % (self.path, name)
            sys.exit(0)

    def close(self):
        self.conn.close()

    def put(self, key, value):
        # A bit of extra effort: get current item, try to preserve its attributes
        item = self.get(key)
        if item == None:
            item = RtDBItem()
            # TODO determine shared attribute based on configuration
            if key == "JOYSTICK_CONTROL":
                item.shared = True
        # Overwrite value and update timestamp
        item.value = msgpack.packb(value)
        item.timestamp = now()
        # Store
        self.conn.reader_check()
        txn = self.conn.begin(write=True)
        cursor = txn.cursor()
        cursor.put(key, item.serialize())
        cursor.close()
        txn.commit()

        # Wake up all ITEM "consumers"
        self.store.wakeWaitingConsumers(self.agent, key)

    def get(self, key, consume=False):
        self.conn.reader_check()
        txn = self.conn.begin(write=consume)
        value = txn.get(key)
        if consume:
            txn.delete(key)
            txn.commit()
        else:
            txn.abort()

        if value != None:
            if self.isSync():
                return RtDBSyncPoint.get_data(value)
            else:
                r = RtDBItem()
                r.deserialize(value) # with RtDB3 some properties (local/shared) are encoded in value
                return r
        return None

    def get_and_clear(self, key):
        return self.get(key, consume=True)

    def isSync(self):
        if 'sync' in self.name:
            return True
        else:
            return False

    def getType(self):
        if self.isSync():
            return 'sync'
        return 'normal'

    def getRtDBItems(self):

        # Skip RtDBInstances of type 'sync'
        if self.isSync():
            return []

        list_items = []

        self.conn.reader_check()
        txn = self.conn.begin()
        cursor = txn.cursor()
        for key, value in cursor:
            r = None
            rf = None
            try:
                r = RtDBItem()
                r.deserialize(value) # with RtDB3 all properties (agent, local/shared) are encoded in value
                rf = RtDBFrameItem(self.agent, key)
                rf.value = r.value
                rf.timestamp = r.timestamp # TODO: find better 'pythonic' way to assign subclass?
                rf.shared = r.shared
                rf.list = r.list
                rf.size = r.size
                list_items.append(rf)
            except:
                print key
                print r
                print rf
                raise Exception("failed to create RtDBFrameItem for dbname %s, key '%s', #value=%d\n" % (self.name, key, len(value)))

        cursor.close()
        txn.abort()

        return list_items

    def getSyncItems(self):
        # Only allow RtDBInstances of type 'sync'
        if not self.isSync():
            return []

        list_items = []

        self.conn.reader_check()
        txn = self.conn.begin()
        cursor = txn.cursor()
        for key, value in cursor:
            try:
                r = RtDBSyncPoint(self.agent, key, value, self.getType())
                list_items.append(r)
            except:
                raise Exception("failed to create RtDBSyncPoint for agent %s, key %s, #value=%d\n" % (self.agent, key, len(value)))

        cursor.close()
        txn.abort()

        return list_items


class RtDBItem():
    def __init__(self):
        self.value = None
        self.timestamp = now()
        self.shared = False
        self.list = False
        self.size = None

    def deserialize(self, serialized):
        self.size = len(serialized)
        self.fromArray(msgpack.unpackb(serialized))

    def fromArray(self, data):
        # data is a deserialized RtDB2Item:
        #    SERIALIZE_DATA_FIXED(data, timestamp, shared, list);
        self.value = msgpack.unpackb(data[0])
        self.timestamp = data[1] # TODO: maybe we should use datetime in this python layer?
        self.shared = data[2]
        self.list = data[3]

    def serialize(self):
        return msgpack.packb([self.value, self.timestamp, self.shared, self.list])

    def age(self):
        tv_sec = self.timestamp[0]
        tv_usec = self.timestamp[1]
        t = datetime.datetime.fromtimestamp(tv_sec + 1e-6*tv_usec)
        dt = datetime.datetime.now() - t
        return dt.total_seconds()

    def __str__(self):
        s = ''
        s += '   age: {}\n'.format(self.age())
        s += 'shared: {}\n'.format(self.shared)
        s += '  list: {}\n'.format(self.list)
        s += ' value: {}\n'.format(self.value)
        return s


class RtDBFrameItem(RtDBItem):
    def __init__(self, agent = None, key = None):
        RtDBItem.__init__(self)
        self.key = key
        self.agent = agent

    def fromArray(self, data):
        # data is a deserialized RtDB2FrameItem:
        #    SERIALIZE_DATA_FIXED(key, agent, data, timestamp, shared, list);
        self.key = data[0]
        self.agent = data[1]
        self.value = msgpack.unpackb(data[2])
        self.timestamp = data[3]
        self.shared = data[4]
        self.list = data[5]

    def serialize(self):
        return msgpack.packb([self.key, self.agent, self.value, self.timestamp, self.shared, self.list])

    def __str__(self):
        s = ''
        s += '   key: {}\n'.format(self.key)
        s += ' agent: {}\n'.format(self.agent)
        s += '   age: {}\n'.format(self.age())
        s += 'shared: {}\n'.format(self.shared)
        s += ' value: {}\n'.format(self.value)
        s += '  list: {}\n'.format(self.list)
        return s


class RtDBSyncPoint():
    def __init__(self, agent, key, raw_data, data_type):
        self.agent = agent
        self.key = key
        self.raw_data = raw_data
        self.data = msgpack.unpackb(raw_data)
        self.type = data_type
        self.size = len(self.raw_data)
        self.keys = self.data.keys() if type(self.data) == dict else self.data
        self.nokeys = len(self.data) if type(self.data) == dict or type(self.data) == list else 1

    # TODO create_item -- needed to implement wait_for_put in Python

    @staticmethod
    def get_data(raw_data):
        data = msgpack.unpackb(raw_data)
        return data

