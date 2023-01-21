# Copyright 2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/python

import sys
import os
import shutil


# this tool applies the database linking trick,
# which is useful to be able to run a test suite / simulation without having to use comm
# WARNING: run this before RTDB clients start to read/write


# user should specify database strings, for example
# /tmp/rtdb_MTP_A and /tmp/rtdb_MTP_B
databases = sys.argv[1:]
if len(databases) == 0:
    raise Exception("please provide database folder(s) as argument")

# iterate over given databases
for database in databases:
    # wipe if existing
    if os.path.isdir(database):
        # sanity check, we don't want to accidentally remove 
        # too much if user makes a mistake when giving arguments
        if not database.startswith('/tmp/rtdb'):
            raise Exception("WARNING: refuse to wipe " + database)
        shutil.rmtree(database)
    # create fresh database
    os.mkdir(database)
    # designate 0 to be the main one
    main_database = "0"
    os.mkdir(os.path.join(database, main_database))
    # link all others
    os.chdir(database)
    for client in range(1, 11): # TODO: this is a bit clumsy and not very scalable, but OK for now
        os.symlink(main_database, str(client))

