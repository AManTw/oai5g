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


#include "TLVEncoder.h"
#include "TLVDecoder.h"
#include "MsNetworkFeatureSupport.h"

int decode_ms_network_feature_support(MsNetworkFeatureSupport *msnetworkfeaturesupport, uint8_t iei, uint8_t *buffer, uint32_t len)
{
    int decoded = 0;


    if(iei > 0)
    {
        CHECK_IEI_DECODER(iei, (*buffer & 0xc0));
    }
    msnetworkfeaturesupport->spare_bits = (*(buffer + decoded) >> 3) & 0x7;
    msnetworkfeaturesupport->extended_periodic_timers = *(buffer + decoded) & 0x1;
    decoded++;

#if defined (NAS_DEBUG)
    dump_ms_network_feature_support_xml(msnetworkfeaturesupport, iei);
#endif
    return decoded;
}
int encode_ms_network_feature_support(MsNetworkFeatureSupport *msnetworkfeaturesupport, uint8_t iei, uint8_t *buffer, uint32_t len)
{
    //uint8_t *lenPtr;
    uint32_t encoded = 0;
    //int encode_result;
    /* Checking IEI and pointer */
    CHECK_PDU_POINTER_AND_LENGTH_ENCODER(buffer, MS_NETWORK_FEATURE_SUPPORT_MINIMUM_LENGTH, len);
#if defined (NAS_DEBUG)
    dump_ms_network_feature_support_xml(msnetworkfeaturesupport, iei);
#endif


    *(buffer + encoded) = 0x00 | ((msnetworkfeaturesupport->spare_bits & 0x7) << 3)
                          | (msnetworkfeaturesupport->extended_periodic_timers & 0x1);
    encoded++;
    return encoded;
}

void dump_ms_network_feature_support_xml(MsNetworkFeatureSupport *msnetworkfeaturesupport, uint8_t iei)
{
    printf("<Ms Network Feature Support>\n");

    if(iei > 0)
        /* Don't display IEI if = 0 */
    {
        printf("    <IEI>0x%X</IEI>\n", iei);
    }

    printf("    <spare_bits>%u<spare_bits>\n", msnetworkfeaturesupport->spare_bits);
    printf("    <extended_periodic_timer>%u<extended_periodic_timer>\n", msnetworkfeaturesupport->extended_periodic_timers);
    printf("</Ms Network Feature Support>\n");
}

