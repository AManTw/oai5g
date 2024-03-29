/*
    Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
    contributor license agreements.  See the NOTICE file distributed with
    this work for additional information regarding copyright ownership.
    The OpenAirInterface Software Alliance licenses this file to You under
    the OAI Public License, Version 1.1  (the "License"); you may not use this file
    except in compliance with the License.
    You may obtain a copy of the License at

        http://www.openairinterface.org/?page_id=698

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
    -------------------------------------------------------------------------------
    For more information about the OpenAirInterface (OAI) Software Alliance:
        contact@openairinterface.org
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "OctetString.h"

#ifndef GPRS_TIMER_H_
#define GPRS_TIMER_H_

#define GPRS_TIMER_MINIMUM_LENGTH 2
#define GPRS_TIMER_MAXIMUM_LENGTH 2

typedef struct GprsTimer_tag
{
#define GPRS_TIMER_UNIT_2S  0b000 /* 2 seconds  */
#define GPRS_TIMER_UNIT_60S 0b001 /* 1 minute */
#define GPRS_TIMER_UNIT_360S  0b010 /* decihours  */
#define GPRS_TIMER_UNIT_0S  0b111 /* deactivated  */
    uint8_t  unit: 3;
    uint8_t  timervalue: 5;
} GprsTimer;

int encode_gprs_timer(GprsTimer *gprstimer, uint8_t iei, uint8_t *buffer, uint32_t len);

void dump_gprs_timer_xml(GprsTimer *gprstimer, uint8_t iei);

int decode_gprs_timer(GprsTimer *gprstimer, uint8_t iei, uint8_t *buffer, uint32_t len);

long gprs_timer_value(GprsTimer *gprstimer);

#endif /* GPRS TIMER_H_ */

