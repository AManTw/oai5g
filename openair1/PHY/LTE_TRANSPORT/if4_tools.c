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

/*! \file PHY/LTE_TRANSPORT/if4_tools.c
    \brief
    \author S. Sandeep Kumar, Raymond Knopp
    \date 2016
    \version 0.1
    \company Eurecom
    \email: ee13b1025@iith.ac.in, knopp@eurecom.fr
    \note
    \warning
*/

#include "PHY/defs_eNB.h"
#include "PHY/TOOLS/alaw_lut.h"
#include "PHY/phy_extern.h"
#include "SCHED/sched_eNB.h"

//#include "targets/ARCH/ETHERNET/USERSPACE/LIB/if_defs.h"
#include "targets/ARCH/ETHERNET/USERSPACE/LIB/ethernet_lib.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

const uint8_t lin2alaw_if4p5[65536] = {213, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, 161, };

void send_IF4p5(RU_t *ru, int frame, int subframe, uint16_t packet_type)
{

    LTE_DL_FRAME_PARMS *fp     = &ru->frame_parms;
    int32_t **txdataF          = ru->common.txdataF_BF;
    int32_t **rxdataF          = ru->common.rxdataF;
    int16_t **prach_rxsigF     = ru->prach_rxsigF;
#if (LTE_RRC_VERSION >= MAKE_VERSION(14, 0, 0))
    int16_t ***prach_rxsigF_br = ru->prach_rxsigF_br;
#endif
    void *tx_buffer            = ru->ifbuffer.tx[subframe & 1];
    void *tx_buffer_prach      = ru->ifbuffer.tx_prach;


    uint16_t symbol_id = 0, element_id = 0;
    uint16_t db_fulllength, db_halflength;
    int slotoffsetF = 0, blockoffsetF = 0;

    uint16_t *data_block = NULL, *i = NULL, *d = NULL;

    IF4p5_header_t *packet_header = NULL;
    eth_state_t *eth = (eth_state_t *)(ru->ifdevice.priv);
    int nsym = fp->symbols_per_tti;

    if(ru->idx == 0)
    {
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_SEND_IF4, 1);
    }

    if(packet_type == IF4p5_PDLFFT)
    {
        //LOG_D(PHY,"send DL_IF4p5: RU %d frame %d, subframe %d\n",ru->idx,frame,subframe);

        if(subframe_select(fp, subframe) == SF_S)
        {
            nsym = fp->dl_symbols_in_S_subframe;
        }


        db_fulllength = 12 * fp->N_RB_DL;
        db_halflength = (db_fulllength) >> 1;
        slotoffsetF = 1;//(subframe)*(fp->ofdm_symbol_size)*((fp->Ncp==1) ? 12 : 14) + 1;
        blockoffsetF = slotoffsetF + fp->ofdm_symbol_size - db_halflength - 1;


        if(eth->flags == ETH_RAW_IF4p5_MODE)
        {
            packet_header = (IF4p5_header_t *)(tx_buffer + MAC_HEADER_SIZE_BYTES);
            data_block = (uint16_t *)(tx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t);
        }
        else
        {
            packet_header = (IF4p5_header_t *)(tx_buffer);
            data_block = (uint16_t *)(tx_buffer + sizeof_IF4p5_header_t);
        }
        gen_IF4p5_dl_header(packet_header, frame, subframe);

        AssertFatal(txdataF[0] != NULL, "txdataF_BF[0] is null\n");
        for(symbol_id = 0; symbol_id < nsym; symbol_id++)
        {
            for(element_id = 0; element_id < db_halflength; element_id++)
            {
                i = (uint16_t *) &txdataF[0][blockoffsetF + element_id];
                data_block[element_id] = ((uint16_t) lin2alaw_if4p5[*i]) | (lin2alaw_if4p5[*(i + 1)] << 8);

                i = (uint16_t *) &txdataF[0][slotoffsetF + element_id];
                data_block[element_id + db_halflength] = ((uint16_t) lin2alaw_if4p5[*i]) | (lin2alaw_if4p5[*(i + 1)] << 8);
            }
            VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 0);

            packet_header->frame_status &= ~(0x000f << 26);
            packet_header->frame_status |= (symbol_id & 0x000f) << 26;
            VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1);
            if((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                            symbol_id,
                                            &tx_buffer,
                                            db_fulllength,
                                            1,
                                            IF4p5_PDLFFT)) < 0)
            {
                perror("ETHERNET write for IF4p5_PDLFFT\n");
            }
            VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0);
            slotoffsetF  += fp->ofdm_symbol_size;
            blockoffsetF += fp->ofdm_symbol_size;
        }
    }
    else if((packet_type == IF4p5_PULFFT) ||
            (packet_type == IF4p5_PULTICK))
    {
        db_fulllength = 12 * fp->N_RB_UL;
        db_halflength = (db_fulllength) >> 1;
        slotoffsetF = 0;
        blockoffsetF = slotoffsetF + fp->ofdm_symbol_size - db_halflength;

        if(subframe_select(fp, subframe) == SF_S)
        {
            nsym = fp->ul_symbols_in_S_subframe;
            slotoffsetF  += (fp->ofdm_symbol_size * (fp->symbols_per_tti - nsym));
            blockoffsetF += (fp->ofdm_symbol_size * (fp->symbols_per_tti - nsym));
        }

        if(eth->flags == ETH_RAW_IF4p5_MODE)
        {
            packet_header = (IF4p5_header_t *)(tx_buffer + MAC_HEADER_SIZE_BYTES);
            data_block = (uint16_t *)(tx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t);
        }
        else
        {
            packet_header = (IF4p5_header_t *)(tx_buffer);
            data_block = (uint16_t *)(tx_buffer + sizeof_IF4p5_header_t);
        }
        gen_IF4p5_ul_header(packet_header, packet_type, frame, subframe);

        if(packet_type == IF4p5_PULFFT)
        {

            for(symbol_id = fp->symbols_per_tti - nsym; symbol_id < fp->symbols_per_tti; symbol_id++)
            {

                uint32_t *rx0 = (uint32_t *) &rxdataF[0][blockoffsetF];
                uint32_t *rx1 = (uint32_t *) &rxdataF[0][slotoffsetF];

                VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME(VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF4_SYMBOL, symbol_id);
                VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 1);

                start_meas(&ru->compression);

                for(element_id = 0; element_id < db_halflength; element_id += 8)
                {
                    i = (uint16_t *) &rx0[element_id];
                    d = (uint16_t *) &data_block[element_id];
                    d[0] = ((uint16_t) lin2alaw_if4p5[i[0]])  | ((uint16_t)(lin2alaw_if4p5[i[1]] << 8));
                    d[1] = ((uint16_t) lin2alaw_if4p5[i[2]])  | ((uint16_t)(lin2alaw_if4p5[i[3]] << 8));
                    d[2] = ((uint16_t) lin2alaw_if4p5[i[4]])  | ((uint16_t)(lin2alaw_if4p5[i[5]] << 8));
                    d[3] = ((uint16_t) lin2alaw_if4p5[i[6]])  | ((uint16_t)(lin2alaw_if4p5[i[7]] << 8));
                    d[4] = ((uint16_t) lin2alaw_if4p5[i[8]])  | ((uint16_t)(lin2alaw_if4p5[i[9]] << 8));
                    d[5] = ((uint16_t) lin2alaw_if4p5[i[10]]) | ((uint16_t)(lin2alaw_if4p5[i[11]] << 8));
                    d[6] = ((uint16_t) lin2alaw_if4p5[i[12]]) | ((uint16_t)(lin2alaw_if4p5[i[13]] << 8));
                    d[7] = ((uint16_t) lin2alaw_if4p5[i[14]]) | ((uint16_t)(lin2alaw_if4p5[i[15]] << 8));

                    i = (uint16_t *) &rx1[element_id];
                    d = (uint16_t *) &data_block[element_id + db_halflength];
                    d[0] = ((uint16_t) lin2alaw_if4p5[i[0]])  | ((uint16_t)(lin2alaw_if4p5[i[1]] << 8));
                    d[1] = ((uint16_t) lin2alaw_if4p5[i[2]])  | ((uint16_t)(lin2alaw_if4p5[i[3]] << 8));
                    d[2] = ((uint16_t) lin2alaw_if4p5[i[4]])  | ((uint16_t)(lin2alaw_if4p5[i[5]] << 8));
                    d[3] = ((uint16_t) lin2alaw_if4p5[i[6]])  | ((uint16_t)(lin2alaw_if4p5[i[7]] << 8));
                    d[4] = ((uint16_t) lin2alaw_if4p5[i[8]])  | ((uint16_t)(lin2alaw_if4p5[i[9]] << 8));
                    d[5] = ((uint16_t) lin2alaw_if4p5[i[10]]) | ((uint16_t)(lin2alaw_if4p5[i[11]] << 8));
                    d[6] = ((uint16_t) lin2alaw_if4p5[i[12]]) | ((uint16_t)(lin2alaw_if4p5[i[13]] << 8));
                    d[7] = ((uint16_t) lin2alaw_if4p5[i[14]]) | ((uint16_t)(lin2alaw_if4p5[i[15]] << 8));

                }

                stop_meas(&ru->compression);
                VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 0);
                packet_header->frame_status &= ~(0x000f << 26);
                packet_header->frame_status |= (symbol_id & 0x000f) << 26;
                VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1);
                start_meas(&ru->transport);
                if((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                                symbol_id,
                                                &tx_buffer,
                                                db_fulllength,
                                                1,
                                                IF4p5_PULFFT)) < 0)
                {
                    perror("ETHERNET write for IF4p5_PULFFT\n");
                }
                stop_meas(&ru->transport);
                VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0);
                slotoffsetF  += fp->ofdm_symbol_size;
                blockoffsetF += fp->ofdm_symbol_size;
            }
        }
        else
        {
            if((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                            0,
                                            &tx_buffer,
                                            0,
                                            1,
                                            IF4p5_PULTICK)) < 0)
            {
                perror("ETHERNET write for IF4p5_PULFFT\n");
            }
        }
    }
    else if(packet_type >= IF4p5_PRACH &&
            packet_type <= IF4p5_PRACH + 4)
    {
        // FIX: hard coded prach samples length
        LOG_D(PHY, "IF4p5_PRACH: frame %d, subframe %d,packet type %x\n", frame, subframe, packet_type);
        db_fulllength = PRACH_NUM_SAMPLES;

        if(eth->flags == ETH_RAW_IF4p5_MODE)
        {
            packet_header = (IF4p5_header_t *)(tx_buffer_prach + MAC_HEADER_SIZE_BYTES);
            data_block = (uint16_t *)(tx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t);
        }
        else
        {
            packet_header = (IF4p5_header_t *)(tx_buffer_prach);
            data_block = (uint16_t *)(tx_buffer_prach + sizeof_IF4p5_header_t);
        }
        gen_IF4p5_prach_header(packet_header, frame, subframe);


        int16_t *rxF;

#if (LTE_RRC_VERSION >= MAKE_VERSION(14, 0, 0))
        if(packet_type > IF4p5_PRACH)
        {
            rxF = &prach_rxsigF_br[packet_type - IF4p5_PRACH - 1][0][0];
        }
        else
#endif
            rxF = &prach_rxsigF[0][0];

        AssertFatal(rxF != NULL, "rxF is null\n");
        if(eth->flags == ETH_RAW_IF4p5_MODE)
        {
            memcpy((void *)(tx_buffer_prach + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t),
                   (void *)rxF,
                   PRACH_BLOCK_SIZE_BYTES);
        }
        else
        {
            memcpy((void *)(tx_buffer_prach + sizeof_IF4p5_header_t),
                   (void *)rxF,
                   PRACH_BLOCK_SIZE_BYTES);
        }
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1);
        if((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                        symbol_id,
                                        &tx_buffer_prach,
                                        db_fulllength,
                                        1,
                                        packet_type)) < 0)
        {
            perror("ETHERNET write for IF4p5_PRACH\n");
        }

        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0);
    }
    else
    {
        AssertFatal(1 == 0, "send_IF4p5 - Unknown packet_type %x", packet_type);
    }

    if(ru->idx == 0)
    {
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_SEND_IF4, 0);
    }
    return;
}

void recv_IF4p5(RU_t *ru, int *frame, int *subframe, uint16_t *packet_type, uint32_t *symbol_number)
{
    LTE_DL_FRAME_PARMS *fp     = &ru->frame_parms;
    int32_t **txdataF          = ru->common.txdataF_BF;
    int32_t **rxdataF          = ru->common.rxdataF;
    int16_t **prach_rxsigF     = ru->prach_rxsigF;
#if (LTE_RRC_VERSION >= MAKE_VERSION(14, 0, 0))
    int16_t ***prach_rxsigF_br = ru->prach_rxsigF_br;
#endif
    void *rx_buffer            = ru->ifbuffer.rx;

    uint16_t element_id;
    uint16_t db_fulllength, db_halflength;
    int slotoffsetF = 0, blockoffsetF = 0;
    eth_state_t *eth = (eth_state_t *)(ru->ifdevice.priv);
    int idx;

    if(ru->idx == 0)
    {
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_RECV_IF4, 1);
    }

    if(ru->function == NGFI_RRU_IF4p5)
    {
        db_fulllength = (12 * fp->N_RB_DL);
    }
    else     // This is not an RRU
    {
        db_fulllength = (12 * fp->N_RB_UL);
    }
    db_halflength = db_fulllength >> 1;

    IF4p5_header_t *packet_header = NULL;
    uint16_t *data_block = NULL, *i = NULL;

    LOG_D(PHY, "recv IF4p5: RU %d waiting (%d samples)\n", ru->idx, db_fulllength);
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 1);
    if(ru->ifdevice.trx_read_func(&ru->ifdevice,
                                  (int64_t *) packet_type,
                                  &rx_buffer,
                                  db_fulllength,
                                  0) < 0)
    {
        perror("ETHERNET read");
    }
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 0);
    if(eth->flags == ETH_RAW_IF4p5_MODE)
    {
        packet_header = (IF4p5_header_t *)(rx_buffer + MAC_HEADER_SIZE_BYTES);
        data_block = (uint16_t *)(rx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t);
    }
    else
    {
        packet_header = (IF4p5_header_t *)(rx_buffer);
        data_block = (uint16_t *)(rx_buffer + sizeof_IF4p5_header_t);
    }



    *frame = ((packet_header->frame_status) >> 6) & 0xffff;
    *subframe = ((packet_header->frame_status) >> 22) & 0x000f;

    *packet_type = packet_header->sub_type;
    LOG_D(PHY, "recv_IF4p5: Frame %d, Subframe %d: packet_type %x\n", *frame, *subframe, *packet_type);
    if(*packet_type == IF4p5_PDLFFT)
    {
        *symbol_number = ((packet_header->frame_status) >> 26) & 0x000f;
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME(VCD_SIGNAL_DUMPER_VARIABLES_RECV_IF4_SYMBOL, *symbol_number);
        LOG_D(PHY, "DL_IF4p5: RU %d frame %d, subframe %d, symbol %d\n", ru->idx, *frame, *subframe, *symbol_number);

        slotoffsetF = (*symbol_number) * (fp->ofdm_symbol_size) + 1; // + (*subframe)*(fp->ofdm_symbol_size)*((fp->Ncp==1) ? 12 : 14) + 1;
        blockoffsetF = slotoffsetF + fp->ofdm_symbol_size - db_halflength - 1;

        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 1);
        for(element_id = 0; element_id < db_halflength; element_id++)
        {
            i = (uint16_t *) &txdataF[0][blockoffsetF + element_id];
            *i = alaw2lin_if4p5[(data_block[element_id] & 0xff) ];
            *(i + 1) = alaw2lin_if4p5[(data_block[element_id] >> 8) ];

            i = (uint16_t *) &txdataF[0][slotoffsetF + element_id];
            *i = alaw2lin_if4p5[(data_block[element_id + db_halflength] & 0xff) ];
            *(i + 1) = alaw2lin_if4p5[(data_block[element_id + db_halflength] >> 8) ];
        }
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 0);
    }
    else if(*packet_type == IF4p5_PULFFT)
    {
        *symbol_number = ((packet_header->frame_status) >> 26) & 0x000f;

        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME(VCD_SIGNAL_DUMPER_VARIABLES_RECV_IF4_SYMBOL, *symbol_number);
        if(ru->idx == 0)
        {
            LOG_D(PHY, "UL_IF4p5: RU %d : frame %d, subframe %d, symbol %d\n", ru->idx, *frame, *subframe, *symbol_number);
        }


        slotoffsetF = (*symbol_number) * (fp->ofdm_symbol_size);
        blockoffsetF = slotoffsetF + fp->ofdm_symbol_size - db_halflength;
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 1);
        for(element_id = 0; element_id < db_halflength; element_id++)
        {
            i = (uint16_t *) &rxdataF[0][blockoffsetF + element_id];
            *i = alaw2lin_if4p5[(data_block[element_id] & 0xff) ];
            *(i + 1) = alaw2lin_if4p5[(data_block[element_id] >> 8) ];

            i = (uint16_t *) &rxdataF[0][slotoffsetF + element_id];
            *i = alaw2lin_if4p5[(data_block[element_id + db_halflength] & 0xff) ];
            *(i + 1) = alaw2lin_if4p5[(data_block[element_id + db_halflength] >> 8) ];

            //if (element_id==0) LOG_I(PHY,"recv_if4p5: symbol %d rxdata0 = (%u,%u)\n",*symbol_number,*i,*(i+1));
        }
        LOG_D(PHY, "PULFFT_IF4p5: CC_id %d : frame %d, subframe %d (symbol %d)=> %d dB\n", ru->idx, *frame, *subframe, *symbol_number,
              dB_fixed(signal_energy((int *)&rxdataF[0][slotoffsetF], db_halflength) +
                       signal_energy((int *)&rxdataF[0][blockoffsetF], db_halflength)));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 0);
    }
    else if(*packet_type >= IF4p5_PRACH &&
            *packet_type <= IF4p5_PRACH + 4)
    {

        int16_t *rxF;

#if (LTE_RRC_VERSION >= MAKE_VERSION(14, 0, 0))
        if(*packet_type > IF4p5_PRACH)
        {
            rxF = &prach_rxsigF_br[*packet_type - IF4p5_PRACH - 1][0][0];
        }
        else
#endif
            rxF = &prach_rxsigF[0][0];

        // FIX: hard coded prach samples length
        db_fulllength = PRACH_NUM_SAMPLES;

        AssertFatal(rxF != NULL, "rxF is null\n");

        if(eth->flags == ETH_RAW_IF4p5_MODE)
        {
            memcpy(rxF,
                   (int16_t *)(rx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF4p5_header_t),
                   PRACH_BLOCK_SIZE_BYTES);
        }
        else
        {
            memcpy(rxF,
                   (int16_t *)(rx_buffer + sizeof_IF4p5_header_t),
                   PRACH_BLOCK_SIZE_BYTES);
        }

        LOG_D(PHY, "PRACH_IF4p5: CC_id %d : frame %d, subframe %d => %d dB\n", ru->idx, *frame, *subframe,
              dB_fixed(signal_energy((int *)&prach_rxsigF[0][0], 839)));
        for(idx = 0; idx < ru->num_eNB; idx++)
        {
            ru->wakeup_prach_eNB(ru->eNB_list[idx], ru, *frame, *subframe);
        }

    }
    else if(*packet_type == IF4p5_PULTICK)
    {

    }
    else
    {
        AssertFatal(1 == 0, "recv_IF4p5 - Unknown packet_type %x", *packet_type);
    }

    if(ru->idx == 0)
    {
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_RECV_IF4, 0);
    }
    return;
}


void gen_IF4p5_dl_header(IF4p5_header_t *dl_packet, int frame, int subframe)
{
    dl_packet->type = IF4p5_PACKET_TYPE;
    dl_packet->sub_type = IF4p5_PDLFFT;

    dl_packet->rsvd = 0;

    // Set frame status
    dl_packet->frame_status = 0;
    dl_packet->frame_status |= (frame & 0xffff) << 6;
    dl_packet->frame_status |= (subframe & 0x000f) << 22;
}


void gen_IF4p5_ul_header(IF4p5_header_t *ul_packet, uint16_t packet_subtype, int frame, int subframe)
{

    ul_packet->type = IF4p5_PACKET_TYPE;
    ul_packet->sub_type = packet_subtype;

    ul_packet->rsvd = 0;

    // Set frame status
    ul_packet->frame_status = 0;
    ul_packet->frame_status |= (frame & 0xffff) << 6;
    ul_packet->frame_status |= (subframe & 0x000f) << 22;
}


void gen_IF4p5_prach_header(IF4p5_header_t *prach_packet, int frame, int subframe)
{
    prach_packet->type = IF4p5_PACKET_TYPE;
    prach_packet->sub_type = IF4p5_PRACH;

    prach_packet->rsvd = 0;

    // Set LTE Prach configuration
    prach_packet->frame_status = 0;
    prach_packet->frame_status |= (frame & 0xffff) << 6;
    prach_packet->frame_status |= (subframe & 0x000f) << 22;
}


void malloc_IF4p5_buffer(RU_t *ru)
{
    // Keep the size large enough
    eth_state_t *eth = (eth_state_t *)(ru->ifdevice.priv);
    int i;

    if(eth->flags == ETH_RAW_IF4p5_MODE)
    {
        for(i = 0; i < 10; i++)
        {
            ru->ifbuffer.tx[i]       = malloc(RAW_IF4p5_PRACH_SIZE_BYTES);
            memset((void *)ru->ifbuffer.tx[i], 0, RAW_IF4p5_PRACH_SIZE_BYTES);
        }
        ru->ifbuffer.tx_prach = malloc(RAW_IF4p5_PRACH_SIZE_BYTES);
        memset((void *)ru->ifbuffer.tx_prach, 0, RAW_IF4p5_PRACH_SIZE_BYTES);
        ru->ifbuffer.rx       = malloc(RAW_IF4p5_PRACH_SIZE_BYTES);
        memset((void *)ru->ifbuffer.rx, 0, RAW_IF4p5_PRACH_SIZE_BYTES);
    }
    else
    {
        for(i = 0; i < 10; i++)
        {
            ru->ifbuffer.tx[i]       = malloc(UDP_IF4p5_PRACH_SIZE_BYTES);
            memset((void *)ru->ifbuffer.tx[i], 0, UDP_IF4p5_PRACH_SIZE_BYTES);
        }
        ru->ifbuffer.tx_prach = malloc(UDP_IF4p5_PRACH_SIZE_BYTES);
        memset((void *)ru->ifbuffer.tx_prach, 0, UDP_IF4p5_PRACH_SIZE_BYTES);
        ru->ifbuffer.rx       = malloc(UDP_IF4p5_PRACH_SIZE_BYTES);
        memset((void *)ru->ifbuffer.rx, 0, UDP_IF4p5_PRACH_SIZE_BYTES);
    }
}
