// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
// minitrace
// Copyright 2014 by Henrik Rydgård
// http://www.github.com/hrydgard/minitrace
// Released under the MIT license.

// See minitrace.h for basic documentation.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#pragma warning (disable:4996)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define __thread __declspec(thread)
#define pthread_mutex_t CRITICAL_SECTION
#define pthread_mutex_init(a, b) InitializeCriticalSection(a)
#define pthread_mutex_lock(a) EnterCriticalSection(a)
#define pthread_mutex_unlock(a) LeaveCriticalSection(a)
#define pthread_mutex_destroy(a) DeleteCriticalSection(a)
#else
#include <signal.h>
#include <pthread.h>
#include <sys/time.h>
#include <unistd.h>
#endif

#ifndef MTR_ENABLED
#define MTR_ENABLED 1
#endif
#include "minitrace.h"

#ifdef __GNUC__
#define ATTR_NORETURN __attribute__((noreturn))
#else
#define ATTR_NORETURN
#endif

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

// Ugh, this struct is already pretty heavy.
// Will probably need to move arguments to a second buffer to support more than one.
typedef struct raw_event {
	char name[255];
	const char *cat;
	void *id;
	int64_t ts;
	uint32_t pid;
	uint32_t tid;
	char ph;
	mtr_arg_type arg_type;
	const char *arg_name;
	union {
		const char *a_str;
		int a_int;
		double a_double;
	};
} raw_event_t;

static raw_event_t *buffer[2];
static volatile int count;
static int is_tracing = 0;
static int64_t time_offset;
static int first_line = 1;
static FILE *f;
static __thread int cur_thread_id;	// Thread local storage
static int cur_process_id;
static pthread_mutex_t mutex;
static pthread_mutex_t flush_mutex;

#define STRING_POOL_SIZE 100
static char *str_pool[100];

// Tiny portability layer.
// Exposes:
//	 get_cur_thread_id()
//	 get_cur_process_id()
//	 mtr_time_s()
//	 pthread basics
#ifdef _WIN32
static int get_cur_thread_id() {
	return (int)GetCurrentThreadId();
}
static int get_cur_process_id() {
	return (int)GetCurrentProcessId();
}

static uint64_t _frequency = 0;
static uint64_t _starttime = 0;
double mtr_time_s() {
	if (_frequency == 0) {
		QueryPerformanceFrequency((LARGE_INTEGER*)&_frequency);
		QueryPerformanceCounter((LARGE_INTEGER*)&_starttime);
	}
	__int64 time;
	QueryPerformanceCounter((LARGE_INTEGER*)&time);
	return ((double) (time - _starttime) / (double) _frequency);
}

// Ctrl+C handling for Windows console apps
static BOOL WINAPI CtrlHandler(DWORD fdwCtrlType) {
	if (is_tracing && fdwCtrlType == CTRL_C_EVENT) {
		printf("Ctrl-C detected! Flushing trace and shutting down.\n\n");
		mtr_flush();
		mtr_shutdown();
	}
	ExitProcess(1);
}

void mtr_register_sigint_handler() {
	// For console apps:
	SetConsoleCtrlHandler(&CtrlHandler, TRUE);
}

#else

static inline int get_cur_thread_id() {
	return (int)(intptr_t)pthread_self();
}
static inline int get_cur_process_id() {
	return (int)getpid();
}

#if defined(BLACKBERRY)
double mtr_time_s() {
	struct timespec time;
	clock_gettime(CLOCK_MONOTONIC, &time); // Linux must use CLOCK_MONOTONIC_RAW due to time warps
	return time.tv_sec + time.tv_nsec / 1.0e9;
}
#else
double mtr_time_s() {
	//static time_t start;
	struct timeval tv;
	gettimeofday(&tv, NULL);
	//if (start == 0) {
	//	start = tv.tv_sec;
	//}
	//tv.tv_sec -= start;
	return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}
#endif	// !BLACKBERRY

static void termination_handler(int signum) ATTR_NORETURN;
static void termination_handler(int signum) {
	(void) signum;
	if (is_tracing) {
		printf("Ctrl-C detected! Flushing trace and shutting down.\n\n");
		mtr_flush();
		fwrite("\n]}\n", 1, 4, f);
		fclose(f);
	}
	exit(1);
}

void mtr_register_sigint_handler() {
#ifndef MTR_ENABLED
	return;
#endif
	// Avoid altering set-to-be-ignored handlers while registering.
	if (signal(SIGINT, &termination_handler) == SIG_IGN)
		signal(SIGINT, SIG_IGN);
}

#endif

void mtr_init_from_stream(void *stream) {
#ifndef MTR_ENABLED
	return;
#endif
	buffer[0] = (raw_event_t *)malloc(INTERNAL_MINITRACE_BUFFER_SIZE * sizeof(raw_event_t));
	buffer[1] = (raw_event_t *)malloc(INTERNAL_MINITRACE_BUFFER_SIZE * sizeof(raw_event_t));
	is_tracing = 1;
	count = 0;
	f = (FILE *)stream;
	const char *header = "{\"traceEvents\":[\n";
	fwrite(header, 1, strlen(header), f);
	//time_offset = (uint64_t)(mtr_time_s() * 1000000);
	time_offset = 0;
	first_line = 1;
	pthread_mutex_init(&mutex, 0);
	pthread_mutex_init(&flush_mutex, 0);
}

void mtr_init(const char *json_file) {
#ifndef MTR_ENABLED
	return;
#endif
	mtr_init_from_stream(fopen(json_file, "wb"));
}

void mtr_shutdown() {
	int i;
#ifndef MTR_ENABLED
	return;
#endif
	is_tracing = 0;
	mtr_flush();
	fwrite("\n]}\n", 1, 4, f);
	fclose(f);
	pthread_mutex_destroy(&mutex);
	pthread_mutex_destroy(&flush_mutex);
	f = 0;
	free(buffer[0]);
	buffer[0] = 0;
	free(buffer[1]);
	buffer[1] = 0;
	for (i = 0; i < STRING_POOL_SIZE; i++) {
		if (str_pool[i]) {
			free(str_pool[i]);
			str_pool[i] = 0;
		}
	}
}

const char *mtr_pool_string(const char *str) {
	int i;
	for (i = 0; i < STRING_POOL_SIZE; i++) {
		if (!str_pool[i]) {
			str_pool[i] = (char*)malloc(strlen(str) + 1);
			strcpy(str_pool[i], str);
			return str_pool[i];
		} else {
			if (!strcmp(str, str_pool[i]))
				return str_pool[i];
		}
	}
	return "string pool full";
}

void mtr_start() {
#ifndef MTR_ENABLED
	return;
#endif
	is_tracing = 1;
}

void mtr_stop() {
#ifndef MTR_ENABLED
	return;
#endif
	is_tracing = 0;
}

// TODO: fwrite more than one line at a time.
void mtr_flush() {
#ifndef MTR_ENABLED
    printf("MTR_ENABLED not defined");
	return;
#endif
	int i = 0;
	char linebuf[1024];
	char arg_buf[1024];
	char id_buf[256];

	// use double buffering for flushing:
	// we create two buffers and copy data from the main buffer over to the second on flush.
	// thus flushing and tracing can happen concurrently without them intermingeling
	// only one flush may be active at all times, which is ensured by flush_mutex
	pthread_mutex_lock(&flush_mutex);
	pthread_mutex_lock(&mutex);
	int flush_count = count;
	count = 0;
	memcpy(buffer[1], buffer[0], flush_count * sizeof(raw_event_t));
	// release the main mutex now, as tracing now operates on the second buffer (this function may no longer touch buffer[0])
	pthread_mutex_unlock(&mutex);

	for (i = 0; i < flush_count; i++) {
		raw_event_t *raw = &buffer[1][i];
		int len;
		switch (raw->arg_type) {
		case MTR_ARG_TYPE_INT:
			snprintf(arg_buf, ARRAY_SIZE(arg_buf), "\"%s\":%i", raw->arg_name, raw->a_int);
			break;
		case MTR_ARG_TYPE_STRING_CONST:
			snprintf(arg_buf, ARRAY_SIZE(arg_buf), "\"%s\":\"%s\"", raw->arg_name, raw->a_str);
			break;
		case MTR_ARG_TYPE_STRING_COPY:

            // EKPC 2020-01-30
            // a_str is sometimes a null ptr
            if (raw->a_str)
            {
                if (strlen(raw->a_str) > 700) {
                    snprintf(arg_buf, ARRAY_SIZE(arg_buf), "\"%s\":\"%.*s\"", raw->arg_name, 700, raw->a_str);
                } else {
                    snprintf(arg_buf, ARRAY_SIZE(arg_buf), "\"%s\":\"%s\"", raw->arg_name, raw->a_str);
                }
            }
            else
            {
                snprintf(arg_buf, ARRAY_SIZE(arg_buf), "\"%s\":\"%s\"", raw->arg_name, "");
            }
			break;
		case MTR_ARG_TYPE_NONE:
			arg_buf[0] = '\0';
			break;
		}
		if (raw->id) {
			switch (raw->ph) {
			case 'b':
			case 'n':
			case 'e':
				// TODO: Support full 64-bit pointers
				snprintf(id_buf, ARRAY_SIZE(id_buf), ",\"id\":\"0x%08x\"", (uint32_t)(uintptr_t)raw->id);
				break;
			case 'X':
				snprintf(id_buf, ARRAY_SIZE(id_buf), ",\"dur\":%i", (int)raw->a_double);
				break;
			}
		} else {
			id_buf[0] = 0;
		}
		const char *cat = raw->cat;
#ifdef _WIN32
		// On Windows, we often end up with backslashes in category.
		char temp[256];
		{
			int len = (int)strlen(cat);
			int i;
			if (len > 255) len = 255;
			for (i = 0; i < len; i++) {
				temp[i] = cat[i] == '\\' ? '/' : cat[i];
			}
			temp[len] = 0;
			cat = temp;
		}
#endif

        // EKPC 2021-12-28
        // Convert timestamp to human-readable timestamp
        time_t ts_time_t = (time_t)(raw->ts / 1E6);
        struct tm* ts = gmtime( &ts_time_t );
        char ts_str[26];
        strftime(ts_str, sizeof ts_str, "%Y-%m-%d %H:%M:%S", ts);
        char us_str[20];
        snprintf(us_str, sizeof us_str, ".%ld", (raw->ts % (int64_t)1E6));
        strcat(ts_str, us_str);

        char ts_json[128];
        snprintf(ts_json, sizeof ts_json, "\"ts%c\":\"%s\"", raw->ph, ts_str);

        // If argument given, add a comma to the argument
        if (strlen(arg_buf) > 0)
        {
            strcat(arg_buf, ",");
        }

		len = snprintf(linebuf, ARRAY_SIZE(linebuf), "%s{\"cat\":\"%s\",\"pid\":%i,\"tid\":%i,\"ts\":%" PRId64 ",\"ph\":\"%c\",\"name\":\"%s\",\"args\":{%s%s}%s}",
				first_line ? "" : ",\n",
				cat, raw->pid, raw->tid, raw->ts - time_offset, raw->ph, raw->name, arg_buf, ts_json, id_buf);
		fwrite(linebuf, 1, len, f);
		first_line = 0;
	}
	pthread_mutex_unlock(&flush_mutex);
}

void mtr_flush_to_disk() {
#ifndef MTR_ENABLED
    printf("MTR_ENABLED not defined");
	return;
#endif
    mtr_flush();
	fflush(f);
}

long mtr_file_pos()
{
    if (f)
    {
        return ftell(f);
    }
    else
    {
        return 0;
    }
}

void internal_mtr_raw_event(const char *category, const char *name, char ph, void *id) {
#ifndef MTR_ENABLED
	return;
#endif
	if (!is_tracing || count >= INTERNAL_MINITRACE_BUFFER_SIZE)
		return;
	double ts = mtr_time_s();
	if (!cur_thread_id) {
		cur_thread_id = get_cur_thread_id();
	}
	if (!cur_process_id) {
		cur_process_id = get_cur_process_id();
	}

#if 0 && _WIN32	// This should work, feel free to enable if you're adventurous and need performance.
	int bufPos = InterlockedExchangeAdd((LONG volatile *)&count, 1);
	raw_event_t *ev = &buffer[0][bufPos];
#else
	pthread_mutex_lock(&mutex);
	raw_event_t *ev = &buffer[0][count];
	count++;
	pthread_mutex_unlock(&mutex);
#endif

	ev->cat = category;
	snprintf(ev->name, sizeof(ev->name), "%s", name);
	ev->id = id;
	ev->ph = ph;
	if (ev->ph == 'X') {
		double x;
		memcpy(&x, id, sizeof(double));
		ev->ts = (int64_t)(x * 1000000);
		ev->a_double = (ts - x) * 1000000;
	} else {
		ev->ts = (int64_t)(ts * 1000000);
	}
	ev->tid = cur_thread_id;
	ev->pid = cur_process_id;
	ev->arg_type = MTR_ARG_TYPE_NONE;
}

void internal_mtr_raw_event_arg(const char *category, const char *name, char ph, void *id, mtr_arg_type arg_type, const char *arg_name, void *arg_value) {
#ifndef MTR_ENABLED
	return;
#endif
	if (!is_tracing || count >= INTERNAL_MINITRACE_BUFFER_SIZE)
		return;
	if (!cur_thread_id) {
		cur_thread_id = get_cur_thread_id();
	}
	if (!cur_process_id) {
		cur_process_id = get_cur_process_id();
	}
	double ts = mtr_time_s();

#if 0 && _WIN32	// This should work, feel free to enable if you're adventurous and need performance.
	int bufPos = InterlockedExchangeAdd((LONG volatile *)&count, 1);
	raw_event_t *ev = &buffer[0][bufPos];
#else
	pthread_mutex_lock(&mutex);
	raw_event_t *ev = &buffer[0][count];
	count++;
	pthread_mutex_unlock(&mutex);
#endif

	ev->cat = category;
	snprintf(ev->name, sizeof(ev->name), "%s", name);
	ev->id = id;
	ev->ts = (int64_t)(ts * 1000000);
	ev->ph = ph;
	ev->tid = cur_thread_id;
	ev->pid = cur_process_id;
	ev->arg_type = arg_type;
	ev->arg_name = arg_name;
	switch (arg_type) {
	case MTR_ARG_TYPE_INT: ev->a_int = (int)(uintptr_t)arg_value; break;
	case MTR_ARG_TYPE_STRING_CONST:	ev->a_str = (const char*)arg_value; break;
	case MTR_ARG_TYPE_STRING_COPY: ev->a_str = strdup((const char*)arg_value); break;
	case MTR_ARG_TYPE_NONE: break;
	}
}

