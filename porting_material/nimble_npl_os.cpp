/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <stddef.h>
#include "nimble/nimble_npl.h"

#include <circle/timer.h>
#include <circle/synchronize.h>
#include "LinkedList.h"



static struct ble_npl_eventq g_eventq_dflt;
extern "C" {
struct ble_npl_eventq* nimble_port_get_dflt_eventq(void) {return &g_eventq_dflt;}
}



bool
ble_npl_os_started(void)
{
    return true;
}

void *
ble_npl_get_current_task_id(void)
{
    return nullptr; // seems OK...
}

template<> void deleteListElement(ble_npl_event* e) {} // so that we don't delete the events when removing them

void
ble_npl_eventq_init(struct ble_npl_eventq *evq)
{
	evq->opaque = new LinkedList<ble_npl_event>();
}

struct ble_npl_event *
ble_npl_eventq_get(struct ble_npl_eventq *evq, ble_npl_time_t timeout)  // don't honor the timeout otherwise we block the whole app
{
	LinkedList<ble_npl_event>* l = (LinkedList<ble_npl_event>*) evq->opaque;
	ble_npl_event* ev = l->takeFirst();
	if (ev) ev->queued = false;
	return ev;
}

void
ble_npl_eventq_put(struct ble_npl_eventq *evq, struct ble_npl_event *ev)
{
	((LinkedList<ble_npl_event>*)evq->opaque)->add(ev);
	ev->queued = true;
}

void
ble_npl_eventq_remove(struct ble_npl_eventq *evq,
                      struct ble_npl_event *ev)
{
	((LinkedList<ble_npl_event>*)evq->opaque)->remove(ev);
	ev->queued = false;
}

void
ble_npl_event_run(struct ble_npl_event *ev)
{
	if (ev) ev->fn(ev);
}

void
ble_npl_event_init(struct ble_npl_event *ev, ble_npl_event_fn *fn,
                   void *arg)
{
	ev->fn = fn;
	ev->arg = arg;
	ev->queued = false;
}

bool ble_npl_event_is_queued(struct ble_npl_event *ev)
{
    return ev->queued;
}

void *
ble_npl_event_get_arg(struct ble_npl_event *ev)
{
    return ev->arg;
}

void
ble_npl_event_set_arg(struct ble_npl_event *ev, void *arg)
{
	ev->arg = arg;
}

ble_npl_error_t
ble_npl_mutex_init(struct ble_npl_mutex *mu)
{
	mu->locked = false;
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_mutex_pend(struct ble_npl_mutex *mu, ble_npl_time_t timeout)
{
	// todo this is ok if we only use 1 core, but what with multiple?
	while (mu->locked) {} // don't bother with the timeout because it's always huge anyway
	mu->locked = true;
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_mutex_release(struct ble_npl_mutex *mu)
{
	// todo this is ok if we only use 1 core, but what with multiple?
	mu->locked = false;
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_sem_init(struct ble_npl_sem *sem, uint16_t tokens)
{
	sem->count = tokens;
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_sem_pend(struct ble_npl_sem *sem, ble_npl_time_t timeout)
{
	// todo this is ok if we only use 1 core, but what with multiple?
	ble_npl_time_t now = ble_npl_time_get();
	while (sem->count==0) {
		if  (ble_npl_time_get()-now>timeout) return BLE_NPL_TIMEOUT; // we need to honor the (always 2s) timeout here
	}
	sem->count--;
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_sem_release(struct ble_npl_sem *sem)
{
	// todo this is ok if we only use 1 core, but what with multiple?
	sem->count++;
    return BLE_NPL_OK;
}

uint16_t
ble_npl_sem_get_count(struct ble_npl_sem *sem)
{
    return sem->count;
}


static LinkedList<struct ble_npl_callout> callouts;
template<> void deleteListElement(ble_npl_callout* e) {} // so that we don't delete the callouts when removing them todo really?

void ble_npl_callout_poll() {
	if (callouts.size()==0) return;
	ble_npl_time_t t = ble_npl_time_get();
	callouts.walkWithDeletion([](struct ble_npl_callout* co,void* _t) {
		ble_npl_time_t& t = *(ble_npl_time_t*)_t;
		if (co->expiresAt<=t) {
		    if (co->evq) ble_npl_eventq_put(co->evq,&co->ev);
		    else ble_npl_event_run(&co->ev);
		    return true;
		}
		return false;
	},&t);
}

void
ble_npl_callout_init(struct ble_npl_callout *co, struct ble_npl_eventq *evq,
                     ble_npl_event_fn *ev_cb, void *ev_arg)
{
	co->evq = evq;
    ble_npl_event_init(&co->ev, ev_cb, ev_arg);
	ble_npl_callout_stop(co);
	callouts.add(co);
}

ble_npl_error_t
ble_npl_callout_reset(struct ble_npl_callout *co, ble_npl_time_t ticks)
{
	co->expiresAt = ble_npl_time_get() + ticks;
    return BLE_NPL_OK;
}

void
ble_npl_callout_stop(struct ble_npl_callout *co)
{
	co->expiresAt = 0;
}

bool
ble_npl_callout_is_active(struct ble_npl_callout *co)
{
	return co->expiresAt > 0;
}

ble_npl_time_t
ble_npl_callout_get_ticks(struct ble_npl_callout *co)
{
	return co->expiresAt - ble_npl_time_get();
}

uint32_t
ble_npl_time_get(void)
{
    return CTimer::Get()->GetClockTicks();
}

ble_npl_error_t
ble_npl_time_ms_to_ticks(uint32_t ms, ble_npl_time_t *out_ticks)
{
	*out_ticks = ble_npl_time_ms_to_ticks32(ms);
    return BLE_NPL_OK;
}

ble_npl_error_t
ble_npl_time_ticks_to_ms(ble_npl_time_t ticks, uint32_t *out_ms)
{
	*out_ms = ble_npl_time_ticks_to_ms32(ticks);
    return BLE_NPL_OK;
}

ble_npl_time_t
ble_npl_time_ms_to_ticks32(uint32_t ms)
{
    return ms*(CLOCKHZ/1000);
}

uint32_t
ble_npl_time_ticks_to_ms32(ble_npl_time_t ticks)
{
    return ticks/(CLOCKHZ/1000);
}

uint32_t
ble_npl_hw_enter_critical(void)
{
	EnterCritical();
    return 0;
}

void
ble_npl_hw_exit_critical(uint32_t ctx)
{
	LeaveCritical();
}
