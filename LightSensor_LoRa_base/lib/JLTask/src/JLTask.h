// A modification of T2Task to distinguish infinite 
// repetition from an ended process. This allows 
// external control (switching on/off) of 1-time processes. 

/*
 * Talk2 AVR Task Scheduler Library
 * http://talk2.wisen.com.au
 *
 * This file is part of Talk2 AVR Task Scheduler Library.

 * Talk2 AVR Task Scheduler Library
 * is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Talk2 AVR Task Scheduler Library
 * is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef JLTASK_H_
#define JLTASK_H_

/* External Libraries */
#include <Arduino.h>

/* Additional Libraries */

#define JLTASK_INFINITE -1

#define JLTASK_STATUS_STOPPED   0
#define JLTASK_STATUS_SCHEDULED 1
#define JLTASK_STATUS_RUNNING   2
#define JLTASK_STATUS_PAUSED    3
#define JLTASK_STATUS_ABORTED   4
#define JLTASK_STATUS_FINISHED  5

class JLTask
{
    typedef void (*cbFunction_t)(JLTask*);
    typedef void (*cbFunctionObj_t)(void*, JLTask*);

  public:
    JLTask(cbFunction_t cbTask);
    JLTask(cbFunctionObj_t cbTask, void *cbObject);
    virtual ~JLTask();

    void run();

    void status(uint8_t status);
    uint8_t status();

    void runInterval(uint32_t interval);
    uint32_t runInterval();

    void runCount(uint32_t count);
    uint32_t runCount();

    uint32_t lastRunTime();
    uint32_t statsCounter();
    uint32_t statsExecTime();

  private:
    uint32_t _lastRunTime;
    uint32_t _runInterval;
    uint32_t _runCount;
    uint32_t _statsCounter;
    uint32_t _statsExecTime;   // Execution Time in Microseconds
    uint8_t _status;
    cbFunction_t _cbTask;
    cbFunctionObj_t _cbTaskObj;
    void *_cbObject;

};

#endif /* JLTASK_H_ */
