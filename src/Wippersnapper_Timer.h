/*!
 * @file Wippersnapper_Timer.h
 *
 * This is a modified version of the SimpleTimer Arduino library.
 * 
 * SimpleTimer - A timer library for Arduino.
 * Author: mromani@ottotecnica.com
 * Copyright (c) 2010 OTTOTECNICA Italy
 *
 * This library is free software; you can redistribute it
 * and/or modify it under the terms of the GNU Lesser
 * General Public License as published by the Free Software
 * Foundation; either version 2.1 of the License, or (at
 * your option) any later version.
 *
 * This library is distributed in the hope that it will
 * be useful, but WITHOUT ANY WARRANTY; without even the
 * implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU Lesser General Public
 * License for more details.
 *
 * You should have received a copy of the GNU Lesser
 * General Public License along with this library; if not,
 * write to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */


#ifndef WIPPERSNAPPER_TIMER_H
#define WIPPERSNAPPER_TIMER_H

#include "Arduino.h"

#define MAX_TIMERS 20

// deferred call constants
#define DEFCALL_DONTRUN     0   // don't call the callback function
#define DEFCALL_RUNONLY     1   // call the callback function but don't delete the timer
#define DEFCALL_RUNANDDEL   2   // call the callback function and delete the timer


typedef void (*timer_callback)(void);

class WSTimer {

public:

    const static int RUN_FOREVER = 0; // setTimer()

    // constructor
    WSTimer();

    // this function must be called inside loop()
    void run();

    // call function f every d milliseconds
    int setInterval(long d, timer_callback f);

    // call function f every d milliseconds for n times
    int setTimer(long d, timer_callback f, int n);

    // destroy the specified timer
    void deleteTimer(int numTimer);

    // returns true if the specified timer is enabled
    bool isEnabled(int numTimer);

    // enables the specified timer
    void enable(int numTimer);

    // disables the specified timer
    void disable(int numTimer);

    // returns the number of used timers
    int getNumTimers();

    // returns the number of available timers
    int getNumAvailableTimers() { return MAX_TIMERS - numTimers; };

private:

    // find the first available slot
    int findFirstFreeSlot();

    // value returned by the millis() function
    // in the previous run() call
    unsigned long prev_millis[MAX_TIMERS];

    // pointers to the callback functions
    timer_callback callbacks[MAX_TIMERS];

    // delay values
    long delays[MAX_TIMERS];

    // number of runs to be executed for each timer
    int maxNumRuns[MAX_TIMERS];

    // number of executed runs for each timer
    int numRuns[MAX_TIMERS];

    // which timers are enabled
    bool enabled[MAX_TIMERS];

    // deferred function call (sort of) - N.B.: this array is only used in run()
    int toBeCalled[MAX_TIMERS];

    // actual number of timers in use
    int numTimers;
};

#endif