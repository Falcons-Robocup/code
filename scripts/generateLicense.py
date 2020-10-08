#!/usr/bin/python

import os
import sys
import traceback
import datetime
from shutil import copy2

filenames = []

HEADER_PYTHON = """\"\"\" \n 2014 - ${CURRENTYEAR} ASML Holding N.V. All Rights Reserved. \n \n NOTICE: \n \n IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. \n \n NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. \n \n NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES \n \"\"\" \n """

HEADER_C = """ /*** \n 2014 - ${CURRENTYEAR} ASML Holding N.V. All Rights Reserved. \n \n NOTICE: \n \n IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. \n \n NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. \n \n NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES \n ***/ \n """

def prependText(filename, text):
    f = open(filename,'r+')
    lines = f.readlines() # read old content
    f.seek(0) # go back to the beginning of the file
    f.write(text) # write new content at the beginning
    for line in lines: # write old content after new
        f.write(line)
    f.close()

for path, subdirs, files in os.walk("/home/robocup/falconsExported/code/packages/"):
    for name in files:
        truncated_file, ext = os.path.splitext(os.path.basename(name))
        if ("c" in ext) or ("cpp" in ext) or ("h" in ext) or ("hpp" in ext) or ("py" in ext):
            filenames.append(path + "/" + name)

now = datetime.datetime.now()

for ArchivesFiles in filenames:
    truncated_file, ext = os.path.splitext(os.path.basename(ArchivesFiles))
    try:
        if (ext == '.c') or (ext == '.cpp') or (ext == '.h') or (ext == '.hpp'):
            prependText(ArchivesFiles, HEADER_C.replace("${CURRENTYEAR}", str(now.year)))
        elif (ext == '.py'):
            prependText(ArchivesFiles, HEADER_PYTHON.replace("${CURRENTYEAR}", str(now.year)))
        else:
            print("Skipping file: %s" % (ArchivesFiles))
    except:
        print("failed to prepend to file: %s\n" % (ArchivesFiles))
        print(traceback.format_exc())
        print(sys.exc_info()[0])

