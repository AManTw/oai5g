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
#include <string.h>
#include <stdint.h>


#include "TLVEncoder.h"
#include "TLVDecoder.h"
#include "ServiceRequest.h"

int decode_service_request(service_request_msg *service_request, uint8_t *buffer, uint32_t len)
{
    uint32_t decoded = 0;
    int decoded_result = 0;

    LOG_FUNC_IN;
    // Check if we got a NULL pointer and if buffer length is >= minimum length expected for the message.
    CHECK_PDU_POINTER_AND_LENGTH_DECODER(buffer, SERVICE_REQUEST_MINIMUM_LENGTH, len);

    /* Decoding mandatory fields */
    if((decoded_result = decode_ksi_and_sequence_number(&service_request->ksiandsequencenumber, 0, buffer + decoded, len - decoded)) < 0)
    {
        LOG_FUNC_RETURN(decoded_result);
    }
    else
    {
        decoded += decoded_result;
    }

    if((decoded_result = decode_short_mac(&service_request->messageauthenticationcode, 0, buffer + decoded, len - decoded)) < 0)
    {
        LOG_FUNC_RETURN(decoded_result);
    }
    else
    {
        decoded += decoded_result;
    }

    LOG_FUNC_RETURN(decoded);
}

int encode_service_request(service_request_msg *service_request, uint8_t *buffer, uint32_t len)
{
    int encoded = 0;
    int encode_result = 0;

    LOG_FUNC_IN;
    /* Checking IEI and pointer */
    CHECK_PDU_POINTER_AND_LENGTH_ENCODER(buffer, SERVICE_REQUEST_MINIMUM_LENGTH, len);

    if((encode_result =
                encode_ksi_and_sequence_number(&service_request->ksiandsequencenumber,
                        0, buffer + encoded, len - encoded)) < 0)        //Return in case of error
    {
        LOG_FUNC_RETURN(encode_result);
    }
    else
    {
        encoded += encode_result;
    }

    if((encode_result =
                encode_short_mac(&service_request->messageauthenticationcode, 0,
                                 buffer + encoded, len - encoded)) < 0)        //Return in case of error
    {
        LOG_FUNC_RETURN(encode_result);
    }
    else
    {
        encoded += encode_result;
    }

    LOG_FUNC_RETURN(encoded);
}

