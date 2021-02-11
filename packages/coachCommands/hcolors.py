# Copyright 2015-2020 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
#
# Define colors for html formatting.

class hcolors:
    OKGREEN = '<color="0,255,0">'
    FAIL = '<color="255,0,0">'
    ENDC = '</color>'


if __name__ == '__main__':
   # Argument parsing.
   parser     = argparse.ArgumentParser(description='falcons color settings')
