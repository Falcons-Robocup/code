# Copyright 2021 Jan Feitsma (Falcons)
# SPDX-License-Identifier: Apache-2.0
#!/usr/bin/env python3
# wrapper around catapult trace2html


import argparse
import os



JSON_FILE_SIZE_LIMIT = 300e6


def parse_args():
    # Argument parsing.
    descriptionTxt = 'Convert tracing JSON file to HTML and optionally launch browser. Check if size is not too large, because if so, things might go wrong (memory consumption, browser not rendering anything).\n'
    exampleTxt = 'Examples:\n   trace2html.py --view /tmp/uftrace.json /tmp/uftrace.html\n\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-f', '--force', help='ignore size check', action='store_true')
    parser.add_argument('-v', '--view', help='automatically launch browser', action='store_true')
    parser.add_argument('jsonfile', help='input tracing JSON file', type=str)
    parser.add_argument('htmlfile', help='output tracing HTML file', type=str)
    return parser.parse_args()



def run(args):
    # check file size
    if os.path.getsize(args.jsonfile) > JSON_FILE_SIZE_LIMIT:
        raise Exception('size of {} exceeds given limit ({}), consider to reduce size via uftrace options'.format(args.jsonfile, JSON_FILE_SIZE_LIMIT))

    # find the conversion tool from Google catapult toolchain
    # if tooling is not installed, then display instructions how to achieve this manually
    # TODO: be more user-friendly? locate -r trace2html$
    trace2html = '/home/jan/falcons/data/external/catapult_py3/tracing/bin/trace2html'
    if not os.path.isfile(trace2html):
        print('echo open google-chromium, browse to chrome://tracing, click load and browse to {}'.format(args.jsonfile))
        return

    # convert
    os.system('{} {} --output={}'.format(trace2html, args.jsonfile, args.htmlfile))

    # view?
    if args.view:
        os.system('google-chrome ' + args.htmlfile)



if __name__ == '__main__':
    # run
    run(parse_args())

