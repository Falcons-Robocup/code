""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
#
# Copyright 2008, Google Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.
#     * Neither the name of Google Inc. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Verifies that Google Test correctly determines whether to use colors."""

__author__ = 'wan@google.com (Zhanyong Wan)'

import os
import gtest_test_utils


IS_WINDOWS = os.name = 'nt'

COLOR_ENV_VAR = 'GTEST_COLOR'
COLOR_FLAG = 'gtest_color'
COMMAND = gtest_test_utils.GetTestExecutablePath('gtest_color_test_')


def SetEnvVar(env_var, value):
  """Sets the env variable to 'value'; unsets it when 'value' is None."""

  if value is not None:
    os.environ[env_var] = value
  elif env_var in os.environ:
    del os.environ[env_var]


def UsesColor(term, color_env_var, color_flag):
  """Runs gtest_color_test_ and returns its exit code."""

  SetEnvVar('TERM', term)
  SetEnvVar(COLOR_ENV_VAR, color_env_var)

  if color_flag is None:
    args = []
  else:
    args = ['--%s=%s' % (COLOR_FLAG, color_flag)]
  p = gtest_test_utils.Subprocess([COMMAND] + args)
  return not p.exited or p.exit_code


class GTestColorTest(gtest_test_utils.TestCase):
  def testNoEnvVarNoFlag(self):
    """Tests the case when there's neither GTEST_COLOR nor --gtest_color."""

    if not IS_WINDOWS:
      self.assert_(not UsesColor('dumb', None, None))
      self.assert_(not UsesColor('emacs', None, None))
      self.assert_(not UsesColor('xterm-mono', None, None))
      self.assert_(not UsesColor('unknown', None, None))
      self.assert_(not UsesColor(None, None, None))
    self.assert_(UsesColor('linux', None, None))
    self.assert_(UsesColor('cygwin', None, None))
    self.assert_(UsesColor('xterm', None, None))
    self.assert_(UsesColor('xterm-color', None, None))
    self.assert_(UsesColor('xterm-256color', None, None))

  def testFlagOnly(self):
    """Tests the case when there's --gtest_color but not GTEST_COLOR."""

    self.assert_(not UsesColor('dumb', None, 'no'))
    self.assert_(not UsesColor('xterm-color', None, 'no'))
    if not IS_WINDOWS:
      self.assert_(not UsesColor('emacs', None, 'auto'))
    self.assert_(UsesColor('xterm', None, 'auto'))
    self.assert_(UsesColor('dumb', None, 'yes'))
    self.assert_(UsesColor('xterm', None, 'yes'))

  def testEnvVarOnly(self):
    """Tests the case when there's GTEST_COLOR but not --gtest_color."""

    self.assert_(not UsesColor('dumb', 'no', None))
    self.assert_(not UsesColor('xterm-color', 'no', None))
    if not IS_WINDOWS:
      self.assert_(not UsesColor('dumb', 'auto', None))
    self.assert_(UsesColor('xterm-color', 'auto', None))
    self.assert_(UsesColor('dumb', 'yes', None))
    self.assert_(UsesColor('xterm-color', 'yes', None))

  def testEnvVarAndFlag(self):
    """Tests the case when there are both GTEST_COLOR and --gtest_color."""

    self.assert_(not UsesColor('xterm-color', 'no', 'no'))
    self.assert_(UsesColor('dumb', 'no', 'yes'))
    self.assert_(UsesColor('xterm-color', 'no', 'auto'))

  def testAliasesOfYesAndNo(self):
    """Tests using aliases in specifying --gtest_color."""

    self.assert_(UsesColor('dumb', None, 'true'))
    self.assert_(UsesColor('dumb', None, 'YES'))
    self.assert_(UsesColor('dumb', None, 'T'))
    self.assert_(UsesColor('dumb', None, '1'))

    self.assert_(not UsesColor('xterm', None, 'f'))
    self.assert_(not UsesColor('xterm', None, 'false'))
    self.assert_(not UsesColor('xterm', None, '0'))
    self.assert_(not UsesColor('xterm', None, 'unknown'))


if __name__ == '__main__':
  gtest_test_utils.Main()
