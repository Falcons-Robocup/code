# Copyright 2022 Erik Kouters (Falcons)
# SPDX-License-Identifier: Apache-2.0
from glob import glob
from keyword import iskeyword
from os.path import dirname, join, split, splitext
import logging
import traceback
logging.basicConfig()

basedir = dirname(__file__)

__all__ = []
for name in glob(join(basedir, '*.py')):
    module = splitext(split(name)[-1])[0]
    if not module.startswith('_') and not iskeyword(module):
        try:
            __import__(__name__+'.'+module)
        except:
            import logging
            logger = logging.getLogger(__name__)
            logger.warning('Ignoring exception while loading the %r plug-in:', module)
            print(traceback.format_exc())
        else:
            __all__.append(module)
__all__.sort()

