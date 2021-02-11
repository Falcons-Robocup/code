# Copyright 2015-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Define colors for output to console.

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[00m'
    BOLD = '\033[01m'
    UNDERLINE = '\033[04m'


if __name__ == '__main__':
   # Argument parsing.
   parser     = argparse.ArgumentParser(description='falcons color settings')
