/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file PHY/LTE_TRANSPORT/if5_tools.c
* \brief 
* \author S. Sandeep Kumar, Raymond Knopp, Tien-Thinh Nguyen
* \date 2016
* \version 0.1
* \company Eurecom
* \email: ee13b1025@iith.ac.in, knopp@eurecom.fr, tien-thinh.nguyen@eurecom.fr 
* \note
* \warning
*/

#include "PHY/defs_eNB.h"
#include "PHY/TOOLS/alaw_lut.h"


//#include "targets/ARCH/ETHERNET/USERSPACE/LIB/if_defs.h"
#include "targets/ARCH/ETHERNET/USERSPACE/LIB/ethernet_lib.h"
#include <intertask_interface.h>
#include "common/utils/LOG/vcd_signal_dumper.h"
//#define DEBUG_DL_MOBIPASS
//#define DEBUG_UL_MOBIPASS
#define SUBFRAME_SKIP_NUM_MOBIPASS 8

const uint8_t lin2alaw_if5[65536] = {213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 213, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 212, 215, 215, 215, 215, 215, 215, 215, 215, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85, 85};

const uint16_t alaw2lin_if5[256] = {60032, 60288, 59520, 59776, 61056, 61312, 60544, 60800, 57984, 58240, 57472, 57728, 59008, 59264, 58496, 58752, 62784, 62912, 62528, 62656, 63296, 63424, 63040, 63168, 61760, 61888, 61504, 61632, 62272, 62400, 62016, 62144, 43520, 44544, 41472, 42496, 47616, 48640, 45568, 46592, 35328, 36352, 33280, 34304, 39424, 40448, 37376, 38400, 54528, 55040, 53504, 54016, 56576, 57088, 55552, 56064, 50432, 50944, 49408, 49920, 52480, 52992, 51456, 51968, 65192, 65208, 65160, 65176, 65256, 65272, 65224, 65240, 65064, 65080, 65032, 65048, 65128, 65144, 65096, 65112, 65448, 65464, 65416, 65432, 65512, 65528, 65480, 65496, 65320, 65336, 65288, 65304, 65384, 65400, 65352, 65368, 64160, 64224, 64032, 64096, 64416, 64480, 64288, 64352, 63648, 63712, 63520, 63584, 63904, 63968, 63776, 63840, 64848, 64880, 64784, 64816, 64976, 65008, 64912, 64944, 64592, 64624, 64528, 64560, 64720, 64752, 64656, 64688, 5504, 5248, 6016, 5760, 4480, 4224, 4992, 4736, 7552, 7296, 8064, 7808, 6528, 6272, 7040, 6784, 2752, 2624, 3008, 2880, 2240, 2112, 2496, 2368, 3776, 3648, 4032, 3904, 3264, 3136, 3520, 3392, 22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944, 30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136, 11008, 10496, 12032, 11520, 8960, 8448, 9984, 9472, 15104, 14592, 16128, 15616, 13056, 12544, 14080, 13568, 344, 328, 376, 360, 280, 264, 312, 296, 472, 456, 504, 488, 408, 392, 440, 424, 88, 72, 120, 104, 24, 8, 56, 40, 216, 200, 248, 232, 152, 136, 184, 168, 1376, 1312, 1504, 1440, 1120, 1056, 1248, 1184, 1888, 1824, 2016, 1952, 1632, 1568, 1760, 1696, 688, 656, 752, 720, 560, 528, 624, 592, 944, 912, 1008, 976, 816, 784, 880, 848};

struct timespec start_comp, start_decomp, end_comp, end_decomp;
int dummy_cnt = 0;
int subframe_skip_extra = 0;
int start_flag = 1;
int offset_cnt = 1;
void send_IF5(RU_t *ru, openair0_timestamp proc_timestamp, int subframe, uint8_t *seqno, uint16_t packet_type) {      
  
  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;
  int32_t *txp[fp->nb_antennas_tx], *rxp[fp->nb_antennas_rx]; 
  int32_t *tx_buffer=NULL;
#ifdef DEBUG_DL_MOBIPASS
  int8_t dummy_buffer[fp->samples_per_tti*2];
#endif
  void    *alaw_buffer = ru->ifbuffer.tx[subframe&1];
  uint16_t *data_block = NULL;
  uint16_t *j = NULL;
  uint16_t packet_id=0, i=0, element_id=0;

  uint32_t spp_eth  = (uint32_t) ru->ifdevice.openair0_cfg->samples_per_packet;
  uint32_t spsf     = (uint32_t) ru->ifdevice.openair0_cfg->samples_per_frame/10;
  eth_state_t *eth = (eth_state_t*) (ru->ifdevice.priv);
 
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_SEND_IF5, 1 );
  if (packet_type == IF5_RRH_GW_DL) {    
    if (eth->compression == ALAW_COMPRESS) {
      if (eth->flags == ETH_RAW_MODE) {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES + MAC_HEADER_SIZE_BYTES);
      } else {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES);
      }    
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        for (i=0; i < fp->nb_antennas_tx; i++) {
          for (element_id=0; element_id< spp_eth; element_id++){        
            j = (uint16_t*) &ru->common.txdata[i][subframe*fp->samples_per_tti+packet_id*spp_eth+element_id];
            data_block[element_id] = ((uint16_t) lin2alaw_if5[*j]) | (lin2alaw_if5[*(j+1)]<<8);
          }  
        }
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] Compress_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 0 );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        ru->ifdevice.trx_write_func(&ru->ifdevice,
				    (proc_timestamp + packet_id*spp_eth),
				    (void**)&data_block,
				    spp_eth,
				    fp->nb_antennas_tx,
				    0);
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] IF_Write_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0 );
      }
    } else if (eth->compression == NO_COMPRESS) {

      for (i=0; i < fp->nb_antennas_tx; i++)
        txp[i] = (void*)&ru->common.txdata[i][subframe*fp->samples_per_tti];
    
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        ru->ifdevice.trx_write_func(&ru->ifdevice,
				    (proc_timestamp + packet_id*spp_eth),
				    (void**)txp,
				    spp_eth,
				    fp->nb_antennas_tx,
				    0);
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] IF_Write_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0 );  
        for (i=0; i < fp->nb_antennas_tx; i++)
          txp[i] += spp_eth;

      }
    }
  } else if (packet_type == IF5_RRH_GW_UL) {
    if (eth->compression == ALAW_COMPRESS) {
      if (eth->flags == ETH_RAW_MODE) {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES + MAC_HEADER_SIZE_BYTES);
      } else {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES);
      }
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        for (i=0; i < fp->nb_antennas_rx; i++) {
          for (element_id=0; element_id< spp_eth; element_id++){
            j = (uint16_t*) &ru->common.rxdata[i][subframe*fp->samples_per_tti+packet_id*spp_eth+element_id];
            data_block[element_id] = ((uint16_t) lin2alaw_if5[*j]) | (lin2alaw_if5[*(j+1)]<<8);
          }
        }
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] Compress_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_COMPR_IF, 0 );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        ru->ifdevice.trx_write_func(&ru->ifdevice,
                                     (proc_timestamp + packet_id*spp_eth),
                                     (void**)&data_block,
                                     spp_eth,
                                     fp->nb_antennas_rx,
                                     0);
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] IF_Write_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0 );
      }
    } else if (eth->compression == NO_COMPRESS) {
      for (i=0; i < fp->nb_antennas_rx; i++)
        rxp[i] = (void*)&ru->common.rxdata[i][subframe*fp->samples_per_tti];
    
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_comp);
        ru->ifdevice.trx_write_func(&ru->ifdevice,
				    (proc_timestamp + packet_id*spp_eth),
				    (void**)rxp,
				    spp_eth,
				    fp->nb_antennas_rx,
				    0);
        clock_gettime( CLOCK_MONOTONIC, &end_comp);
        LOG_D(HW,"[SF %d] IF_Write_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_comp, end_comp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_WRITE_IF, 0 );            
        for (i=0; i < fp->nb_antennas_rx; i++)
          rxp[i] += spp_eth;

      }    
    }
  } else if (packet_type == IF5_MOBIPASS) {    
    /* the only difference between mobipass standalone and the other one
     * is the timestamp in trx_write_func, but let's duplicate anyway
     * (plus we don't call malloc for the standalone case)
     */
    if (ru->if_timing == synch_to_mobipass_standalone) {
      uint16_t db_fulllength = PAYLOAD_MOBIPASS_NUM_SAMPLES;

      __m128i *data_block=NULL, *data_block_head=NULL;
      __m128i *txp128;
      __m128i t0, t1;

      unsigned char _tx_buffer[MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t)];
      tx_buffer=(int32_t *)_tx_buffer;

      IF5_mobipass_header_t *header = (IF5_mobipass_header_t *)((uint8_t *)tx_buffer + MAC_HEADER_SIZE_BYTES);
      data_block_head = (__m128i *)((uint8_t *)tx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t);

      header->flags = 0;
      header->fifo_status = 0;
      header->seqno = *seqno;
      header->ack = 0;
      header->word0 = 0;

      txp[0] = (void*)&ru->common.txdata[0][subframe*ru->frame_parms.samples_per_tti];
      txp128 = (__m128i *) txp[0];

      for (packet_id=0; packet_id<fp->samples_per_tti/db_fulllength; packet_id++) {
        header->time_stamp = htonl((uint32_t)(proc_timestamp + packet_id*db_fulllength));
        data_block = data_block_head;

        for (i=0; i<db_fulllength>>2; i+=2) {
          t0 = _mm_srai_epi16(*txp128++, 4);
          t1 = _mm_srai_epi16(*txp128++, 4);
         _mm_storeu_si128(data_block++, _mm_packs_epi16(t0, t1));
        }

        // Write the packet to the fronthaul
        if ((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                          proc_timestamp + packet_id*db_fulllength,
                                          (void**)&tx_buffer,
                                          db_fulllength,
                                          1,
                                          IF5_MOBIPASS)) < 0) {
          perror("ETHERNET write for IF5_MOBIPASS\n");
        }
        header->seqno += 1;
      }
      *seqno = header->seqno;
      tx_buffer = NULL;
    } else {
      uint16_t db_fulllength = PAYLOAD_MOBIPASS_NUM_SAMPLES;
      
      __m128i *data_block=NULL, *data_block_head=NULL;

      __m128i *txp128;
      __m128i t0, t1;

      // tx_buffer = memalign(16, MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t));
      tx_buffer = malloc(MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t));
      IF5_mobipass_header_t *header = (IF5_mobipass_header_t *)((uint8_t *)tx_buffer + MAC_HEADER_SIZE_BYTES);
      data_block_head = (__m128i *)((uint8_t *)tx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t);
    
      header->flags = 0;
      header->fifo_status = 0;  
      header->seqno = *seqno;
      header->ack = 0;
      header->word0 = 0;  
      
      txp[0] = (void*)&ru->common.txdata[0][subframe*ru->frame_parms.samples_per_tti];
      txp128 = (__m128i *) txp[0];
                
      for (packet_id=0; packet_id<fp->samples_per_tti/db_fulllength; packet_id++) {
        header->time_stamp = htonl((uint32_t)(proc_timestamp + packet_id*db_fulllength));
        data_block = data_block_head; 
      
        for (i=0; i<db_fulllength>>2; i+=2) {
          t0 = _mm_srai_epi16(*txp128++, 4);
          t1 = _mm_srai_epi16(*txp128++, 4);   
//        *data_block++ = _mm_packs_epi16(t0, t1);     
         _mm_storeu_si128(data_block++, _mm_packs_epi16(t0, t1));     
        }
        
        // Write the packet to the fronthaul
        if ((ru->ifdevice.trx_write_func(&ru->ifdevice,
                                          packet_id,
                                          (void**)&tx_buffer,
                                          db_fulllength,
                                          1,
                                          IF5_MOBIPASS)) < 0) {
          perror("ETHERNET write for IF5_MOBIPASS\n");
        }

#ifdef DEBUG_DL_MOBIPASS
       if ((subframe==0)&&(dummy_cnt == 100)) {
          memcpy((void*)&dummy_buffer[packet_id*db_fulllength*2],(void*)data_block_head,db_fulllength*2);
        }
#endif
        header->seqno += 1;    
      }  
      *seqno = header->seqno;

#ifdef DEBUG_DL_MOBIPASS
      uint8_t txe;
      txe = dB_fixed(signal_energy(txp[0],fp->samples_per_tti));
      if (txe > 0){
        LOG_D(PHY,"[Mobipass] frame:%d, subframe:%d, energy %d\n", (proc_timestamp/(10*fp->samples_per_tti))&1023,subframe, txe);
      }
#endif  
    }
  } else {    
    AssertFatal(1==0, "send_IF5 - Unknown packet_type %x", packet_type);     
  }  
  
  free(tx_buffer);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_SEND_IF5, 0 );  
#ifdef DEBUG_DL_MOBIPASS 
  if(subframe==0) {
    if (dummy_cnt==100) {
      LOG_M("txsigmb.m","txs",(void*)dummy_buffer, fp->samples_per_tti,1, 5); 
      exit(-1);
    } else {
    dummy_cnt++;
    }
  }
#endif
  return;  		    
}


void recv_IF5(RU_t *ru, openair0_timestamp *proc_timestamp, int subframe, uint16_t packet_type) {

  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;
  int32_t *txp[fp->nb_antennas_tx], *rxp[fp->nb_antennas_rx]; 

  uint16_t packet_id=0, i=0, element_id=0;
#ifdef DEBUG_UL_MOBIPASS
  //int8_t dummy_buffer_rx[fp->samples_per_tti*2];
  uint8_t rxe;
#endif


  int32_t spp_eth  = (int32_t) ru->ifdevice.openair0_cfg->samples_per_packet;
  int32_t spsf     = (int32_t) ru->ifdevice.openair0_cfg->samples_per_frame/10;
  void    *alaw_buffer = ru->ifbuffer.rx; 
  uint16_t *data_block = NULL;
  uint16_t *j      = NULL;

  openair0_timestamp timestamp[spsf / spp_eth];
  eth_state_t *eth = (eth_state_t*) (ru->ifdevice.priv);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_RECV_IF5, 1 );  
  
  if (packet_type == IF5_RRH_GW_DL) {
    if (eth->compression == ALAW_COMPRESS) {
      if (eth->flags == ETH_RAW_MODE) {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES + MAC_HEADER_SIZE_BYTES);
      } else {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES);
      }
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_RECV_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        ru->ifdevice.trx_read_func(&ru->ifdevice,
				   &timestamp[packet_id],
				   (void**)&data_block,
				   spp_eth,
				   fp->nb_antennas_tx);

        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] IF_Read_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 0 );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        for (i=0; i < fp->nb_antennas_tx; i++) {
          for (element_id=0; element_id< spp_eth; element_id++) {
            j      = (uint16_t*) &ru->common.txdata[i][subframe*fp->samples_per_tti+packet_id*spp_eth+element_id];
            *j     = alaw2lin_if5[ (data_block[element_id] & 0xff) ];
            *(j+1) = alaw2lin_if5[ (data_block[element_id]>>8) ];
          }
        }
        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] Decomperss_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 0 );
      }
    } else if (eth->compression == NO_COMPRESS) {
      for (i=0; i < fp->nb_antennas_tx; i++)
        txp[i] = (void*)&ru->common.txdata[i][subframe*fp->samples_per_tti];
    
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_RECV_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 1 );  
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        ru->ifdevice.trx_read_func(&ru->ifdevice,
				   &timestamp[packet_id],
				   (void**)txp,
				   spp_eth,
				   fp->nb_antennas_tx);
        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] IF_Read_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 0 );  
        for (i=0; i < fp->nb_antennas_tx; i++)
          txp[i] += spp_eth;

      }
    }
    *proc_timestamp = timestamp[0];
    
  } else if (packet_type == IF5_RRH_GW_UL) { 
    if (eth->compression == ALAW_COMPRESS) {
      if (eth->flags == ETH_RAW_MODE) {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES + MAC_HEADER_SIZE_BYTES);
      } else {
        data_block = (uint16_t*)(alaw_buffer + APP_HEADER_SIZE_BYTES);
      }
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        ru->ifdevice.trx_read_func(&ru->ifdevice,
				   &timestamp[packet_id],
				   (void**)&data_block,
				   spp_eth,
				   fp->nb_antennas_rx);
        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] IF_Read_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 0 );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        for (i=0; i < fp->nb_antennas_rx; i++) {
          for (element_id=0; element_id< spp_eth; element_id++) {
            j      = (uint16_t*) &ru->common.rxdata[i][subframe*fp->samples_per_tti+packet_id*spp_eth+element_id];
            *j     = alaw2lin_if5[ (data_block[element_id] & 0xff) ];
            *(j+1) = alaw2lin_if5[ (data_block[element_id]>>8) ];
          }
        }
        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] Decomperss_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_DECOMPR_IF, 0 );
      }
    } else if (eth->compression == NO_COMPRESS) {
      for (i=0; i < fp->nb_antennas_rx; i++)
        rxp[i] = (void*)&ru->common.rxdata[i][subframe*fp->samples_per_tti];
    
      for (packet_id=0; packet_id < spsf / spp_eth; packet_id++) {
        VCD_SIGNAL_DUMPER_DUMP_VARIABLE_BY_NAME( VCD_SIGNAL_DUMPER_VARIABLES_SEND_IF5_PKT_ID, packet_id );
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 1 );
        clock_gettime( CLOCK_MONOTONIC, &start_decomp);
        ru->ifdevice.trx_read_func(&ru->ifdevice,
				   &timestamp[packet_id],
				   (void**)rxp,
				   spp_eth,
				   fp->nb_antennas_rx);
        clock_gettime( CLOCK_MONOTONIC, &end_decomp);
        LOG_D(HW,"[SF %d] IF_Read_Time: %"PRId64"\n",subframe,clock_difftime_ns(start_decomp, end_decomp));
        VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_TRX_READ_IF, 0 );            
        for (i=0; i < fp->nb_antennas_rx; i++)
          rxp[i] += spp_eth;

      }
    }
    *proc_timestamp = timestamp[0];
      
  } else if (packet_type == IF5_MOBIPASS) {
    if (ru->if_timing == synch_to_mobipass_standalone) {
      uint16_t db_fulllength = PAYLOAD_MOBIPASS_NUM_SAMPLES;
      openair0_timestamp timestamp_mobipass[fp->samples_per_tti/db_fulllength];
      int32_t *rx_buffer=NULL;
      __m128i *data_block=NULL, *data_block_head=NULL;
      __m128i *rxp128;
      __m128i r0;

      unsigned char _rx_buffer[MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t)];
      rx_buffer = (int32_t *)_rx_buffer;
      data_block_head = (__m128i *)((uint8_t *)rx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t);

      rxp[0] = (void*)&ru->common.rxdata[0][subframe*ru->frame_parms.samples_per_tti];
      rxp128 = (__m128i *) (rxp[0]);

      packet_id=0;
      while(packet_id<fp->samples_per_tti/db_fulllength) {
        data_block = data_block_head;

        ru->ifdevice.trx_read_func(&ru->ifdevice,
                                         &timestamp_mobipass[packet_id],
                                         (void**)&rx_buffer,
                                         db_fulllength,
                                          1
                                          );

          //store rxdata and increase packet_id
          rxp[0] = (void*)&ru->common.rxdata[0][(subframe*ru->frame_parms.samples_per_tti)+packet_id*db_fulllength];
          rxp128 = (__m128i *) (rxp[0]);
          for (i=0; i<db_fulllength>>2; i+=2) {
            r0 = _mm_loadu_si128(data_block++);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpacklo_epi8(r0,r0),8),4);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpackhi_epi8(r0,r0),8),4);
          }
          packet_id++;
      }//end while

      *proc_timestamp = ntohl(timestamp_mobipass[0]);
    } else {
      
      uint16_t db_fulllength = PAYLOAD_MOBIPASS_NUM_SAMPLES;
      openair0_timestamp timestamp_mobipass[fp->samples_per_tti/db_fulllength];
#ifdef DEBUG_UL_MOBIPASS
      int lower_offset = 0;
      int  upper_offset = 70000;
#endif
      int subframe_skip = 0;
      int reset_flag = 0;
      int32_t *rx_buffer=NULL;
      __m128i *data_block=NULL, *data_block_head=NULL;
      __m128i *rxp128;
      __m128i r0;

      //rx_buffer = memalign(16, MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t));
      rx_buffer = malloc(MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t + db_fulllength*sizeof(int16_t));
      IF5_mobipass_header_t *header = (IF5_mobipass_header_t *)((uint8_t *)rx_buffer + MAC_HEADER_SIZE_BYTES);
      data_block_head = (__m128i *)((uint8_t *)rx_buffer + MAC_HEADER_SIZE_BYTES + sizeof_IF5_mobipass_header_t);
   
      rxp[0] = (void*)&ru->common.rxdata[0][subframe*ru->frame_parms.samples_per_tti];
      rxp128 = (__m128i *) (rxp[0]);
   
      RU_proc_t *proc = &ru->proc;
/*
   //   while(packet_id<fp->samples_per_tti/db_fulllength) {
        data_block = data_block_head;

        eNB->ifdevice.trx_read_func(&eNB->ifdevice,
                                         &ts0,
                                         (void**)&rx_buffer,
                                         db_fulllength,
                                          1
                                          );

        if ((header->seqno == 1)&&(first_packet==1))  { 
           first_packet = 0;  //ignore the packets before synchnorization
           packet_id = 0;
          ts_offset = ntohl(ts0);
        } 
        if (first_packet==0) { 
          packet_cnt++;
          ts = ntohl(ts0);
          packet_id = (ts-ts_offset)/db_fulllength;
          packet_id = packet_id % (fp->samples_per_tti/db_fulllength);

          printf("[IF5_tools]packet_id:%d\n", packet_id);
          // if (ts_stored == 0) {
          //   ts_stored = 1;
          *proc_timestamp = ntohl(ts - (packet_id*db_fulllength));
          // }
          rxp[0] = (void*)&eNB->common_vars.rxdata[0][0][(subframe*eNB->frame_parms.samples_per_tti)+packet_id*db_fulllength];
          rxp128 = (__m128i *) (rxp[0]);

          for (i=0; i<db_fulllength>>2; i+=2) {
            r0 = _mm_loadu_si128(data_block++);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpacklo_epi8(r0,r0),8),4);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpackhi_epi8(r0,r0),8),4);
          }
        }
    //  }//end while
*/
   

      packet_id=0; 
      while(packet_id<fp->samples_per_tti/db_fulllength) {
        data_block = data_block_head;

	
	ru->ifdevice.trx_read_func(&ru->ifdevice,
				 &timestamp_mobipass[packet_id],
				 (void**)&rx_buffer,
				 db_fulllength,
				 1
				 );
#ifdef DEBUG_UL_MOBIPASS
        if (((proc->timestamp_tx + lower_offset) > ntohl(timestamp_mobipass[packet_id])) || ((proc->timestamp_tx + upper_offset) < ntohl(timestamp_mobipass[packet_id]))) {
          //ignore the packet
          subframe_skip_extra = (subframe_skip_extra + 1)%67;         
         LOG_D("[Mobipass] ignored packet, id:[%d,%d], proc->timestamp_tx:%llu, proc->timestamp_rx:%llu, seqno:%d\n", packet_id,subframe_skip_extra, proc->timestamp_tx, ntohl(timestamp_mobipass[packet_id]), header->seqno);
        }             
#endif
        //skip SUBFRAME_SKIP_NUM_MOBIPASS additional UL packets
        if ((start_flag == 1) && (subframe_skip < SUBFRAME_SKIP_NUM_MOBIPASS)){
          subframe_skip++;
          offset_cnt = header->seqno;
        } else {
          if ((offset_cnt != header->seqno) && (start_flag == 0) && (proc->first_rx > 3)){
#ifdef DEBUG_UL_MOBIPASS
             LOG_D(PHY,"[Mobipass] Reset sequence number, offset_cnt:%d, header->seqno:%d, packet_id:%d\n", offset_cnt, header->seqno, packet_id);
#endif
             reset_flag=1;
          }
          if ((reset_flag == 1) && (proc->first_rx > 3 ) && (start_flag == 0) && (packet_id == 0)) {
             packet_id = 1;  
             reset_flag = 0;
          }
          start_flag = 0;

          //store rxdata and increase packet_id
          rxp[0] = (void*)&ru->common.rxdata[0][(subframe*ru->frame_parms.samples_per_tti)+packet_id*db_fulllength];
          rxp128 = (__m128i *) (rxp[0]);
          for (i=0; i<db_fulllength>>2; i+=2) {
            r0 = _mm_loadu_si128(data_block++);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpacklo_epi8(r0,r0),8),4);
            *rxp128++ =_mm_slli_epi16(_mm_srai_epi16(_mm_unpackhi_epi8(r0,r0),8),4);
          }   
          packet_id++; 
          offset_cnt = (header->seqno+1)&255;
        }
      }//end while
    
        *proc_timestamp = ntohl(timestamp_mobipass[0]); 
#ifdef DEBUG_UL_MOBIPASS
	LOG_I(PHY,"[Mobipass][Recv_MOBIPASS] timestamp: %llu\n ",  *proc_timestamp);
	if (eNB->CC_id>0) {
	  rxe = dB_fixed(signal_energy(rxp[0],fp->samples_per_tti)); 
	  if (rxe > 0){
	    LOG_I(PHY,"[Mobipass] frame:%d, subframe:%d, energy %d\n", (*proc_timestamp/(10*fp->samples_per_tti))&1023,subframe, rxe);
	    
	    //    LOG_M("rxsigmb.m","rxs",(void*)dummy_buffer_rx, fp->samples_per_tti,1, 5); 
	    //    exit(-1);
	  }
	}
#endif


     
    }
  } else {
    AssertFatal(1==0, "recv_IF5 - Unknown packet_type %x", packet_type);     
  }  

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_RECV_IF5, 0 );  
  
  return;  
}

void malloc_IF5_buffer(RU_t *ru) {
  // Keep the size large enough, 3840 is the number of samples in a packet for 20MHz BW
  int i;
  eth_state_t *eth = (eth_state_t*) (ru->ifdevice.priv);
  if (eth->compression == ALAW_COMPRESS) {
    if (eth->flags == ETH_RAW_MODE) {
       for (i=0;i<10;i++)
         ru->ifbuffer.tx[i] = malloc(RAW_PACKET_SIZE_BYTES_ALAW(3840));
       ru->ifbuffer.rx = malloc(RAW_PACKET_SIZE_BYTES_ALAW(3840));
    } else {     
      for (i=0;i<10;i++)
        ru->ifbuffer.tx[i] = malloc(UDP_PACKET_SIZE_BYTES_ALAW(3840));
      ru->ifbuffer.rx = malloc(UDP_PACKET_SIZE_BYTES_ALAW(3840));
    }
  } else if (eth->compression == NO_COMPRESS) {
    if (eth->flags == ETH_RAW_MODE) {
      for (i=0;i<10;i++)
        ru->ifbuffer.tx[i] = malloc(RAW_PACKET_SIZE_BYTES(3840));
      ru->ifbuffer.rx = malloc(RAW_PACKET_SIZE_BYTES(3840));
    } else {
      for (i=0;i<10;i++)   
        ru->ifbuffer.tx[i] = malloc(UDP_PACKET_SIZE_BYTES(3840));
      ru->ifbuffer.rx = malloc(UDP_PACKET_SIZE_BYTES(3840));
    }
  }
}
