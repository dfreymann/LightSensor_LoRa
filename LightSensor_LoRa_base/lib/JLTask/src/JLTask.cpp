/*
 * Talk2 AVR Task Scheduler Library
 * http://talk2.wisen.com.au
 *
 *
 *  Copyright 2015-2016 by Mike Musskopf.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "JLTask.h"

JLTask::JLTask(cbFunction_t cbTask)
  :
  _lastRunTime(0),
  _runInterval(0),
  _runCount(0),
  _statsCounter(0),
  _statsExecTime(0),
  _status(JLTASK_STATUS_STOPPED),
  _cbTask(cbTask),
  _cbTaskObj(NULL),
  _cbObject(NULL)
{
}

JLTask::JLTask(cbFunctionObj_t cbTaskObj, void *cbObject)
  :
  _lastRunTime(0),
  _runInterval(0),
  _runCount(0),
  _statsCounter(0),
  _statsExecTime(0),
  _status(JLTASK_STATUS_STOPPED),
  _cbTask(NULL),
  _cbTaskObj(cbTaskObj),
  _cbObject(cbObject)
{
}

JLTask::~JLTask()
{
  //Destructor
}


void JLTask::run()
{
  if((millis() - this->_lastRunTime) >= this->_runInterval && this->_status == JLTASK_STATUS_SCHEDULED)
  {
    this->_lastRunTime = millis();
    this->_statsCounter++;
    this->_status = JLTASK_STATUS_RUNNING;
    uint32_t startMicros = micros();

    if(this->_cbObject != NULL)
    {
      this->_cbTaskObj(_cbObject, this);
    } else {
      this->_cbTask(this);
    }

    this->_statsExecTime = micros() - startMicros;

    if(this->_runCount == JLTASK_INFINITE)
    {
      //This is an infinite task, reschedule it
      this->_status = JLTASK_STATUS_SCHEDULED;
    } else {
      if(--this->_runCount == 0)
      {
        //Last execution, finish it!
        this->_status = JLTASK_STATUS_FINISHED;
      } else {
        //Still more executions to go, reschedule it
        this->_status = JLTASK_STATUS_SCHEDULED;
      }
    }
  }
}

void JLTask::status(uint8_t status)
{
  this->_status = status;
}

uint8_t JLTask::status()
{
  return this->_status;
}

void JLTask::runInterval(uint32_t interval)
{
  this->_runInterval = interval;
}

uint32_t JLTask::runInterval()
{
  return this->_runInterval;
}

void JLTask::runCount(uint32_t count)
{
  // for backwards compatibility. runCount 
  // initiailized with '0' was used to indicate 
  // infinite repeating tasks
  if (count == 0) count = JLTASK_INFINITE; 

  this->_runCount = count;
}

uint32_t JLTask::runCount()
{
  return this->_runCount;
}

uint32_t JLTask::lastRunTime()
{
  return this->_lastRunTime;
}

uint32_t JLTask::statsCounter()
{
  return this->_statsCounter;
}

uint32_t JLTask::statsExecTime()
{
  return this->_statsExecTime;
}
