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
#ifndef __RFSIM__H__
#define __RFSIM__H__
#include "lte-softmodem.h"
#include "openair1/SIMULATION/TOOLS/sim.h"
#include "platform_constants.h"
#include "common/ran_context.h"
#include "PHY/defs_UE.h"
#include "PHY/defs_eNB.h"

void init_ocm(double snr_dB, double sinr_dB);

void update_ocm(double snr_dB, double sinr_dB);

//extern pthread_mutex_t async_server_lock;
//extern pthread_cond_t async_server_notify;
//extern int async_server_shutdown;

void init_channel_vars(void);

#endif
