// Copyright 2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <errno.h>
#include <stdio.h>
#include <unistd.h> // getopt
#include <stdlib.h>

#include "raspiControl.hpp"

int main(int argc, char** argv) {
    int opt = 0;
    while ((opt = getopt(argc, argv, "h")) != -1) {
        switch (opt) {
        case 'h':
            printf("INFO    : no arguments\n");
            exit(EXIT_SUCCESS);
            break;
        }
    }

    raspiControl *client = new raspiControl();
    client->multiCastSetup();

    client->update();

    client->multiCastClose();

    return 0;
}
