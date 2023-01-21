// Copyright 2022 Lucas Catabriga (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef STACKTRACER_HPP
#define STACKTRACER_HPP

#ifdef __linux__

#include <stdio.h>
#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>

#endif

class StackTracer
{
public:
    StackTracer()
    {
    #ifdef __linux__
        signal(SIGSEGV, handler);
        signal(SIGFPE, handler);
        signal(SIGABRT, handler);
        signal(SIGILL, handler);
        signal(SIGINT, handler);
        signal(SIGTERM, handler);
    #endif
    }

    virtual ~StackTracer()
    {

    }

#ifdef __linux__
    static void handler(int sig)
    {
        int32_t max_size = 100;
        void* array[max_size];
        size_t size;

        // get void*'s for all entries on the stack
        size = backtrace(array, max_size);

        // print out all the frames to stderr
        fprintf(stderr, "Error: signal %d:\n", sig);
        backtrace_symbols_fd(array, size, STDERR_FILENO);
        exit(1);
    }
 #endif
};

#endif
