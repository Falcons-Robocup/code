#!/usr/bin/python

import os
import sys
import traceback
import datetime
import git # sudo pip3 install GitPython
from shutil import copy2

filenames = []

HEADER_PYTHON = """# Copyright ${YEARRANGE} ${CREATOR} (Falcons)
# SPDX-License-Identifier: Apache-2.0
"""

HEADER_C = """// Copyright ${YEARRANGE} ${CREATOR} (Falcons)
// SPDX-License-Identifier: Apache-2.0
"""

def prependText(filename, text):
    f = open(filename,'r+')
    lines = f.readlines() # read old content
    f.seek(0) # go back to the beginning of the file
    f.write(text) # write new content at the beginning
    for line in lines: # write old content after new
        f.write(line)
    f.close()

def applyVariablesToText(text, variables):
    result = text
    for (k, v) in variables.items():
        result = result.replace("${" + k + "}", str(v))
    return result

for path, subdirs, files in os.walk(os.getenv("HOME") + "/falconsExported/code/packages/"):
    for name in files:
        truncated_file, ext = os.path.splitext(os.path.basename(name))
        if ("c" in ext) or ("cpp" in ext) or ("h" in ext) or ("hpp" in ext) or ("py" in ext):
            filenames.append(path + "/" + name)

now = datetime.datetime.now()
repo = git.Repo("~/falcons/code")

def determineVariables(filename):
    filename = filename.replace('/falconsExported/', '/falcons/') # look in source context
    variables = {"CURRENTYEAR": str(now.year)}
    commits = list(repo.iter_commits(paths=filename))
    firstCommit = commits[-1]
    lastCommit = commits[0]
    variables["CREATOR"] = firstCommit.author.name
    variables["CREATIONYEAR"] = str(firstCommit.committed_datetime.year)
    variables["LASTMODIFIEDYEAR"] = str(lastCommit.committed_datetime.year)
    variables["YEARRANGE"] = variables["CREATIONYEAR"] + "-" + variables["LASTMODIFIEDYEAR"]
    if variables["CREATIONYEAR"] == variables["LASTMODIFIEDYEAR"]:
        variables["YEARRANGE"] = variables["CREATIONYEAR"] # 2016-2016 just looks silly
    return variables
    

for f in filenames:
    bf, ext = os.path.splitext(os.path.basename(f))
    variables = determineVariables(f)
    if (ext == '.c') or (ext == '.cpp') or (ext == '.h') or (ext == '.hpp'):
        headerTemplate = HEADER_C
    elif (ext == '.py'):
        headerTemplate = HEADER_PYTHON
    else:
        continue
    # prepend
    prependText(f, applyVariablesToText(headerTemplate, variables))

