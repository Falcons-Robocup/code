#!/usr/bin/env python3
#
# Jan Feitsma, June 2020
#
# Laptop management: display and manage information relevant for this laptop.



import argparse
import collections
import contextlib
import json
import logging
import math
import netifaces
import os
import random
import re
import requests
import secrets
import shlex
import socket
import subprocess
import sys
import tempfile
import textwrap
from datetime import datetime
from pathlib import Path
from logging import handlers
from subprocess import PIPE, STDOUT

import falconspy


# ----------------------------------------------------------------
# Global variables
#

toolname = Path(__file__).stem

# Get path to the code root, assuming this scripts is:
#   code/scripts/setupLaptopInfo.py
code_root = Path(__file__).parent.parent

logger = logging.getLogger()
log_max_size = (10 * (1024 ** 2)) # 10 MiB
log_max_count = 3

authorized_keys_location = 'config/authorized_keys'

available_hostnames_url = 'https://api.falcons-robocup.nl/v1/assets/info/hostnames'



# ----------------------------------------------------------------
# Manage the laptop's information
#

def get_hostname():
    return socket.gethostname()

def cmd_output(cmd):
    return subprocess.check_output(cmd).strip().decode('utf-8')

def get_mac_addresses():
    mac_eth = {}
    mac_wifi = {}

    for interface in netifaces.interfaces():
        # guess type based on first character
        if interface[0] == 'w':
            mac_wifi[interface] = netifaces.ifaddresses(interface)[netifaces.AF_LINK][0]['addr']
        elif interface[0] == 'e':
            mac_eth[interface] = netifaces.ifaddresses(interface)[netifaces.AF_LINK][0]['addr']

    return mac_eth, mac_wifi



# ----------------------------------------------------------------
# Manage the laptop's hostname
#

def read_hostnames():
    """Parse the hosts file to get a mapping of laptop hostnames to their IP address."""

    hostnames = {}

    # Parse the hosts file to find all laptop hostnames
    with open(falconspy.FALCONS_CONFIG_PATH + '/hosts') as hosts:
        in_laptop_hosts_section = False

        for line in hosts:

            if line.startswith('#'):
                if line.startswith('##'):
                    in_laptop_hosts_section = line.strip() == '## Laptop hostnames'
                continue

            if in_laptop_hosts_section:
                # We need to match the hostname entries with different spacing, e.g,
                #   172.16.74.1	kroonsteentje
                #   172.16.74.10	fapper
                #   172.16.74.45    dalek
                pattern = r'^(\S+)\s+(\S+)\s*'
                match = re.search(pattern, line)

                if match:
                    ip = match.group(1)
                    hostname = match.group(2)
                    hostnames[hostname] = ip

    return hostnames


def find_available_hostnames(hostnames):
    """Figure out which hostnames are not yet taken."""

    hostnames = hostnames.copy()

    # Remove hostnames from the list which are already in use
    # TODO: remove this once we can rely on the nest API, which is after all ubuntu 16 laptops have
    # been turned in (and their public keys got removed from authorized_keys on the master branch)
    authorized_keys_path = code_root / authorized_keys_location

    with authorized_keys_path.open() as authorized_keys_file:
        for line in authorized_keys_file:
            for hostname in hostnames:
                if hostname in line:
                    hostnames.remove(hostname)
                    break

    # Figure out which hostnames are still available by asking nest
    hostnames = requests.post(available_hostnames_url, json=hostnames).json()

    # We're left with hostnames that should be suitable for the laptop
    logger.debug("Found available hostnames:")
    for hostname in hostnames:
        logger.debug(f" - {hostname}")

    return hostnames


def prompt_is_replacement():
    """
    Prompt the user whether this laptop is a replacement of their previous (Ubuntu 16) laptop.
    """

    answer = None

    while not answer:
        answer = input('Are you replacing your previous laptop? [yes/no]: ')

    return answer.lower().startswith('y')


def prompt_new_hostname(hostnames):
    """
    Prompt the user with a menu to let them choose a new hostname for this laptop,
    e.g.:
        Enter the number of the hostname you'd like to use:

        Page 1/2:
          0: kroonsteentje
          1: dalek
          2: pear
          3: mario
          4: wario
          5: wicky
          6: bowser
          7: vegeta
          8: toad
          9: link

        Enter your choice [q, n, 0-9, /, ?]:
    """

    lines = 10
    page = 0

    search = ''

    # We're printing here instead of using the logger, because this is interactive
    # and does not yield useful information to end up in the logfile
    print('')
    print("Enter the number of the hostname you'd like to use:")

    # Keep prompting until the user has selected a hostname
    # This will redraw the choices when, for example, navigating to the next page
    while True:

        print()

        # Switch the list of hostnames we're showing based on whether
        # we're in search mode or not
        if search:
            pagetype = 'Search'
            selection = [h for h in hostnames if search in h]
        else:
            pagetype = 'Page'
            selection = hostnames

        # Calculate a few convenience variables
        count = len(selection)
        pages = math.ceil(count / lines)

        start = min(page * lines, count)
        end = min((page+1) * lines, count)

        # Print the available options
        print(f'{pagetype} {page+1}/{pages}:')

        sublist = selection[start:end]
        for i in range(len(sublist)):
            item = sublist[i]
            if str(item).isdigit():
                item = chr(97+sublist[i])
            print(f'  {i}: {item}')


        print()

        # Create a list of choices the user currently has
        choices = {}

        choices['q'] = 'Quit'

        if page:
            choices['p'] = 'Previous page'

        if end < count:
            choices['n'] = 'Next page'

        if count:
            max_choice = min(lines, count) - 1
            choices[f'0-{max_choice}'] = f'Select choice (number between 0 and {max_choice})'

        if search:
            choices['c'] = 'Cancel search'

        choices['/'] = 'Search for an option'
        choices['?'] = 'This help text'

        choices_str = ', '.join(choices.keys())

        # Prompt for an answer
        choice = input(f"Enter your choice [{choices_str}]: ")

        # Return their answer
        if choice.isdigit() and int(choice) < len(sublist) :
            return sublist[int(choice)]

        # Validate the chocie
        if not choice:
            print("Please enter a choice")
            continue

        command = choice[0]

        if command not in choices:
            print("Invalid choice")
            continue

        # Process command
        if command == 'q':
            return None

        if command == 'p':
            page = max(page - 1, 0)

        if command == 'n':
            page = min(page + 1, pages)

        if command == '/':
            page = 0
            search = choice[1:]

            if not search:
                search = input("Search for: ")

        if command == 'c':
            search = ''

        if command == '?':
            print()
            print("Available choices:")

            for command, help_text in choices.items():
                print(f'  {command:3} : {help_text}')

            print()
            input("Press Enter to continue...")


def set_new_hostname(hostname):
    """Set the specified hostname on this laptop"""
    return cmd_output(['sudo', 'hostnamectl', 'set-hostname', hostname])


def configure_hostname(preferred_hostname=None, allow_existing_hostname=False):
    """
    Determine whether a new hostname is required. If yes, prompt the user to pick one from the
    hosts file. Only hostnames which are not taken are allowed when allow_existing_hostname is
    not specified. Also first tries to pick the preferred hostname if provided.

    Returns whether this laptop has a registered hostname.
    """

    current_hostname = os.uname().nodename

    hostnames = list(read_hostnames())

    # We don't need to change the hostname if we already have a listed hostname
    if current_hostname in hostnames:
        logger.info("Laptop already has a registered hostname")
        return True

    available_hostnames = find_available_hostnames(hostnames)

    if not allow_existing_hostname and not available_hostnames:
        logger.error("No suitable hostname is available")
        return False

    # Now make a selection of the hostnames
    new_hostname = None

    if allow_existing_hostname is None:
        allow_existing_hostname = prompt_is_replacement()

    if allow_existing_hostname:
        hostname_selection = hostnames
    else:
        hostname_selection = available_hostnames

    if preferred_hostname:
        if preferred_hostname in hostname_selection:
            new_hostname = preferred_hostname
        else:
            # Fall-through to prompting for a new hostname
            logger.info("The preferred hostname does not appear to be available")

    # Let the user select a new hostname unless they quit or abort the prompt
    if not new_hostname:
        with contextlib.suppress(KeyboardInterrupt):
            new_hostname = prompt_new_hostname(hostname_selection)

    # Choose a random hostname if no hostname has been chosen by the user
    if not new_hostname:
        new_hostname = random.choice(available_hostnames)

    # Configure the chosen hostname
    set_new_hostname(new_hostname)

    logger.info("Sucessfully configured hostname to %s", new_hostname)

    return True


def configure_wifi():
    """Create the NetworkManager configuration file to connect to the Falcons_A_a network"""

    config_file = '/etc/NetworkManager/system-connections/Falcons_A_a.nmconnection'

    if os.path.exists(config_file):
        return

    hostname = get_hostname()
    _, mac_wifi  = get_mac_addresses()

    ip_address = read_hostnames().get(hostname)
    mac_address = next(iter(mac_wifi.values())).upper()

    # TODO: In theory, the setup-wifi script should also be capable of configuring the WiFi, but
    # this yet didn't work. Perhaps it could be converted to a more readeble Python version
    template = textwrap.dedent(f"""
        [connection]
        id=Falcons_A_a
        uuid=9f92c9db-b621-4b15-b828-05161b309629
        type=802-11-wireless

        [802-11-wireless]
        ssid=Falcons_A_a
        mode=infrastructure
        mac-address={mac_address}

        [802-11-wireless-security]
        key-mgmt=wpa-psk
        psk=WIFI_access

        [ipv6]
        method=ignore

        [ipv4]
        method=manual
        dns=172.16.74.1;
        addresses1={ip_address};24;172.16.74.1;
    """)

    # We only set capture_output to prevent the tee output from ending up in
    # the terminal. Tee just happens to be an easy tool to write the config
    # to a file as root.
    subprocess.run(['sudo', 'tee', config_file], capture_output=True, input=template.encode('utf-8'))
    subprocess.run(['sudo', 'chmod', '600', config_file])


# ----------------------------------------------------------------
# Main
#

def init(args):
    """Setup the root logger to both print to the console and log to a file"""

    # Set the root logger to debug to allow debug statements to reach the log file
    logger.setLevel(logging.DEBUG)

    # Only set the console to debug if the vebose option was specifieds
    consoleLevel = logging.DEBUG if args.verbose else logging.INFO

    # Add a file logger
    fileFormatter = logging.Formatter("%(asctime)s [%(levelname)-7.7s] %(message)s")
    fileHandler = handlers.RotatingFileHandler(f'/var/tmp/{toolname}.log', maxBytes=log_max_size, backupCount=log_max_count)
    fileHandler.setLevel(logging.DEBUG)
    fileHandler.setFormatter(fileFormatter)
    logger.addHandler(fileHandler)

    # Add a console logger
    consoleHandler = logging.StreamHandler(sys.stdout)
    consoleHandler.setLevel(consoleLevel)
    logger.addHandler(consoleHandler)



if __name__ == '__main__':
    # argument parsing
    descriptionTxt = 'Display configuration info for current laptop, optionally manage.\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt)
    parser.add_argument('-v', '--verbose', help='use verbose logging', action='store_true')
    parser.add_argument('-p', '--preferred-hostname', help='specify the preferred hostname for this laptop', default=None)
    parser.add_argument('--replace-laptop', help='allow specifying a hostname which is already in use', action='store_true')
    parser.add_argument('--no-replace-laptop', help="don't ask whether this laptop is a replacement", action='store_true')
    args       = parser.parse_args()

    init(args)

    # When None, the user will be prompted for this question
    allow_existing_hostname = True if args.replace_laptop else False if args.no_replace_laptop else None

    # Configure new hostname
    configure_hostname(args.preferred_hostname, allow_existing_hostname)
    logger.info('')

    # Configure the Falcons_A_a Wi-Fi network
    configure_wifi()
