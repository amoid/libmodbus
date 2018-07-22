/*
 * Copyright © 2008-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <pthread.h>
#include <unistd.h>
#include "unit-test.h"

enum {
    TCP,
    TCP_PI,
    RTU
};

//#define VV_DEBUG    

#ifdef VV_DEBUG
#define VV_PRINTF(x)                         printf x
#define VV_LOG_TRACE()                    printf("%s %d\r\n", __func__, __LINE__) 
#define VV_VERBOSE_LOG_TRACE()    printf("%s %d\r\n", __func__, __LINE__)   
#else
#define VV_PRINTF(x)                        
#define VV_LOG_TRACE()                    
#define VV_VERBOSE_LOG_TRACE()    
#endif

#define VV_E_NONE                        0
#define VV_OS_NORMALL               -1
#define VV_E_NULL_PTR                 -10000

#define MAX_MODBUS_ELEMENT_CNT     64

#define TCP_MODBUS_DEFAULT_PORT_NO    502
#define TCP_MODBUS_DEFAULT_DEV_NO      0XFF

unsigned int modbusTermCnt = 1;

typedef enum {
   TcpModbusDevStaInactive = 0,
   TcpModbusDevStaInit = 1,
   TcpModbusDevStaActive = 2,
   TcpModbusDevStaConnected = 3,
   TcpModbusDevStaTimeout = 4,
   TcpModbusDevStaEnd
} TcpModbusDevSta_e; 

typedef struct tcp_modbus_dev_cfg_s {
   TcpModbusDevSta_e state;
   char ipAddr[24];
   unsigned portNum;
   unsigned devNum;
   unsigned char pData[16];
   modbus_t *pCtx;

   int (*state_reset)(struct tcp_modbus_dev_cfg_s *cfg, TcpModbusDevSta_e sta);

   int (*check_connection_and_keepalive)(struct tcp_modbus_dev_cfg_s *cfg);
   int (*walk_state_machine) (struct tcp_modbus_dev_cfg_s *cfg);
   int (*connect_to_remote) (struct tcp_modbus_dev_cfg_s *cfg);
   int (*get_remote_state) (struct tcp_modbus_dev_cfg_s *cfg);
   int (*set_remote_state) (struct tcp_modbus_dev_cfg_s *cfg);    
} tcp_modbus_dev_cfg_t;

typedef enum modbus_dev_reg_grp_s {
   ModbusDevRegGrg_DiInputSignal, 
   ModbusDevRegGrg_DoOutputValue, 
   ModbusDevRegGrg_PowerOnDigitalOutPutValue,
   ModbusDevRegGrg_CommunicationFailSafeValue,
   ModbusDevRegGrg_AuxiliaryMemoryMFlag,

   ModbusDevRegGrg_CurrentInputValue_0,
   ModbusDevRegGrg_CurrentInputValue_1,
   ModbusDevRegGrg_CurrentInputValue_2,
   ModbusDevRegGrg_CurrentInputValue_3,
   ModbusDevRegGrg_CommunicationFailSafeTimeSettingValue,
   ModbusDevRegGrg_AllDIValue,
   ModbusDevRegGrg_ModuleName_1,
   ModbusDevRegGrg_ModuleName_2,
   ModbusDevRegGrg_Version_1,
   ModbusDevRegGrg_Version_2,
   ModbusDevRegGrg_MacSerialNumber,
   ModbusDevRegGrg_ModuleIDInNormalMode,
   ModbusDevRegGrg_ProtocolInNormalMode,
   ModbusDevRegGrg_BaudRateInNormalMode,
   ModbusDevRegGrg_ParityOptionInNormalMode,
   ModbusDevRegGrg_StopBitsInNormalMode,
   ModbusDevRegGrg_TimeOutSettingInNormalMode,
   ModbusDevRegGrg_CurrentInputValue_1_32f,
   ModbusDevRegGrg_CurrentInputValue_2_32f,
   ModbusDevRegGrg_CurrentInputValue_3_32f,
   ModbusDevRegGrg_CurrentInputValue_4_32f,
   ModbusDevRegGrg_AnalogAuxiliaryMemoryAMFlag,
   ModbusDevRegGrg_WIFI_Mode,
   ModbusDevRegGrg_WIFI_Encryption_WPA2,
   ModbusDevRegGrg_WIFI_SSID,
   ModbusDevRegGrg_WIFI_Password,
   ModbusDevRegGrg_WIFI_Channel,
   ModbusDevRegGrg_WIFI_IP,
   ModbusDevRegGrg_WIFI_MASK,
   ModbusDevRegGrg_WIFI_MODBUS_ID,
   ModbusDevRegGrg_WIFI_LOCAL_PORT,
   ModbusDevRegGrg_WIFI_REMOTE_PORT,
   ModbusDevRegGrg_WIFI_DHCP_Enable,
   ModbusDevRegGrg_WIFI_PROTOCAL,
   ModbusDevRegGrg_WIFI_TX_POWER,
   ModbusDevRegGrg_MAC_ADDRESS,
   ModbusDevRegGrg_count,
} modbus_dev_reg_grp_e;

typedef enum modbus_reg_attr_s {
   ModbusRegNormRO,
   ModbusRegNormWO,
   ModbusRegNormRW,
   ModbusRegInitRO,
   ModbusRegInitWO,
   ModbusRegInitRW,
   ModbusRegAttrCnt,
} modbus_reg_attr_e;

typedef struct modbus_dev_reg_attr_s {
   modbus_dev_reg_grp_e regGrp;
   unsigned int regBase;
   unsigned int regCnt;
   unsigned char opcode[4];
   modbus_reg_attr_e nRw;
   modbus_reg_attr_e iRw;
} modbus_dev_reg_attr_t;

modbus_dev_reg_attr_t gModbusDevRegAttrTbl[] = {
   {ModbusDevRegGrg_DiInputSignal,                                 1,  16, {1,2,5,15}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_DoOutputValue,                               17,16, {1,2,5,15}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_PowerOnDigitalOutPutValue,              33,16,{1,2,5,15}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CommunicationFailSafeValue,            49,16,{1,2,5,15}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_AuxiliaryMemoryMFlag,                    129,1024,{1,2,5,15}, ModbusRegNormRO, ModbusRegInitRO},
   
   {ModbusDevRegGrg_CurrentInputValue_0,                      1,16,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_1,                     97,16,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_2,             129,16,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_3,                    161,16,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CommunicationFailSafeTimeSettingValue,  177,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_AllDIValue,                                   178,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_ModuleName_1,                            211,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_ModuleName_2,                            212,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_Version_1,                                   213,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_Version_2,                                   214,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_MacSerialNumber,                         215,6,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_ModuleIDInNormalMode,                300,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_ProtocolInNormalMode,                  301,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_BaudRateInNormalMode,                302,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_ParityOptionInNormalMode,            303,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_StopBitsInNormalMode,                  304,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_TimeOutSettingInNormalMode,        305,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_1_32f,              609,32,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_2_32f,              705,32,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_3_32f,              801,32,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_CurrentInputValue_4_32f,              897,32,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_AnalogAuxiliaryMemoryAMFlag,  1281,128,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_Mode,                                 401,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_Encryption_WPA2,                402,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_SSID,                                 403,32,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_Password,                           435,498,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_Channel,                             499,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_IP,                                     500,4,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_MASK,                                504,4,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_MODBUS_ID,                       512,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_LOCAL_PORT,                      513,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_REMOTE_PORT,                   514,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_DHCP_Enable,                     515,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_PROTOCAL,                         516,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_WIFI_TX_POWER,                        517,1,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_MAC_ADDRESS,                           518,6,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO},
   {ModbusDevRegGrg_count,                                         0xfff,6,{3,4,6,16}, ModbusRegNormRO, ModbusRegInitRO}
};

tcp_modbus_dev_cfg_t gModbusDevCfg[MAX_MODBUS_ELEMENT_CNT];

int tcp_modbus_state_reset(struct tcp_modbus_dev_cfg_s *cfg, TcpModbusDevSta_e sta) 
{
    if (NULL != cfg && NULL != cfg->state_reset)
        cfg->state = sta;

    return VV_E_NONE;
}

int tcp_modbus_check_connection_and_keepalive(struct tcp_modbus_dev_cfg_s *cfg) 
{
   int ret = VV_E_NONE;

   VV_VERBOSE_LOG_TRACE();
   if (NULL == cfg)
       return VV_E_NULL_PTR;

   VV_VERBOSE_LOG_TRACE();
   
   if (cfg->state == TcpModbusDevStaActive 
       || cfg->state == TcpModbusDevStaTimeout)  {
       VV_VERBOSE_LOG_TRACE();
       if (cfg->connect_to_remote) 
            ret = cfg->connect_to_remote(cfg);
   } else {
       VV_PRINTF(("This device is actvie yet, Plese deactive it and then active\r\n"));
   }
   VV_VERBOSE_LOG_TRACE();
   return ret;
}

int tcp_modbus_connect_to_remote(struct tcp_modbus_dev_cfg_s *cfg) 
{
   int ret = VV_E_NONE;

   VV_VERBOSE_LOG_TRACE();
   if (NULL == cfg)
       return VV_E_NULL_PTR;

   VV_VERBOSE_LOG_TRACE();
   if (cfg->state == TcpModbusDevStaActive
       || cfg->state == TcpModbusDevStaTimeout)  {
       cfg->pCtx = modbus_new_tcp(cfg->ipAddr, cfg->portNum);   
       VV_VERBOSE_LOG_TRACE();
       if (NULL != cfg->pCtx) {
           VV_VERBOSE_LOG_TRACE();
           modbus_set_slave(cfg->pCtx, cfg->devNum);  
           ret = modbus_connect(cfg->pCtx);
           if (VV_E_NONE != ret) {
                return ret;
           }
           cfg->state = TcpModbusDevStaConnected;
       } 
   } else {
       VV_PRINTF(("This device is actvie yet, Plese deactive it and then active\r\n"));
   }

   return ret;
}


int tcp_modbus_walk_state_machine(struct tcp_modbus_dev_cfg_s *cfg) 
{
   int ret = VV_E_NONE;

   VV_LOG_TRACE();
   if (NULL == cfg) 
       return VV_E_NULL_PTR;

   VV_LOG_TRACE();
   if (cfg->state != TcpModbusDevStaConnected)
       return ret;

   VV_LOG_TRACE();

/*
   if (cfg->get_remote_state != NULL)
       ret += cfg->get_remote_state(cfg);
*/
   if (cfg->set_remote_state != NULL)
       ret += cfg->set_remote_state(cfg);

   return ret;
}


int tcp_modbus_get_remote_state(struct tcp_modbus_dev_cfg_s *cfg) 

{
   int retry_cnt = 10;
   
   uint8_t tab_rp_bits[20] = {0};
   int ret = 0;
#ifdef VV_DEBUG
   VV_LOG_TRACE();
   VV_PRINTF(("%s %d %d\r\n", cfg->ipAddr, cfg->portNum, cfg->state));
#endif

   while (retry_cnt--) {
       ret = modbus_read_bits(cfg->pCtx, 17, 1, tab_rp_bits);
       if (ret != VV_OS_NORMALL) {
            return ret; 
       }
   }

   if (ret == VV_OS_NORMALL) {
        cfg->state_reset(cfg, TcpModbusDevStaTimeout);
   }
   return ret;
}

int tcp_modbus_set_remote_state (struct tcp_modbus_dev_cfg_s *cfg) 
{
   int retry_cnt = 10;
   int ret = VV_E_NONE;
#ifdef VV_DEBUG
   VV_LOG_TRACE();
#endif
   VV_LOG_TRACE();

   while (retry_cnt--) {
       ret = modbus_write_bit(cfg->pCtx, 17, ON);
       if (ret != VV_OS_NORMALL) {
            return ret; 
       }
   }

   if (ret == VV_OS_NORMALL) {
        cfg->state_reset(cfg, TcpModbusDevStaTimeout);
    }

   return ret;
}

void *tcp_modbus_cfg_thread(void *pData) 
{
   int i;
   tcp_modbus_dev_cfg_t *pInst;
   unsigned int devCnt = modbusTermCnt;
   i = 0; 

   VV_LOG_TRACE(); 
   while (1) {
       //VV_LOG_TRACE(); 
       i = i%devCnt;
        VV_LOG_TRACE(); 
       pInst = &gModbusDevCfg[i]; 
       if (pInst->walk_state_machine != NULL) 
           pInst->walk_state_machine(pInst);   
       usleep(1000);
       i++;
   }
}

void *tcp_modbus_kick_off_keepalive_thread(void *arg) 
{
   int i;
   tcp_modbus_dev_cfg_t *pInst;
   unsigned int devCnt = modbusTermCnt;
   i = 0; 

   VV_LOG_TRACE(); 
   // Connect to all remote device at first time

   for (i = 0; i < devCnt; i++) {
       VV_LOG_TRACE(); 
       pInst = &gModbusDevCfg[i];
       if (pInst->check_connection_and_keepalive != NULL)  {
           VV_LOG_TRACE(); 
           pInst->check_connection_and_keepalive(pInst); 
       }
       VV_PRINTF(("modbusTermCnt = %d %d \r\n", modbusTermCnt, pInst->check_connection_and_keepalive == NULL));
       VV_VERBOSE_LOG_TRACE();
   }
   VV_LOG_TRACE(); 
   i = 0;
   
   while (1) {
       //VV_LOG_TRACE(); 
       i = i%devCnt;
       pInst = &gModbusDevCfg[i]; 
       
       if (pInst->check_connection_and_keepalive != NULL) 
           pInst->check_connection_and_keepalive(pInst);   
       
       usleep(1000000);
       i++;
   }
}

int tcp_modbus_cfg_reset_to_default (tcp_modbus_dev_cfg_t *cfg) 
{
   if (NULL == cfg)
       return VV_E_NULL_PTR;
   
   memset(cfg, 0x0, sizeof(tcp_modbus_dev_cfg_t)); 
   cfg->pCtx = NULL;
   cfg->state = TcpModbusDevStaInactive;
   cfg->portNum = TCP_MODBUS_DEFAULT_PORT_NO;
   cfg->devNum = TCP_MODBUS_DEFAULT_DEV_NO;

   cfg->state_reset = tcp_modbus_state_reset;
   cfg->check_connection_and_keepalive = tcp_modbus_check_connection_and_keepalive;
   cfg->connect_to_remote = tcp_modbus_connect_to_remote;
   cfg->walk_state_machine = tcp_modbus_walk_state_machine;
   cfg->get_remote_state = tcp_modbus_get_remote_state;
   cfg->set_remote_state = tcp_modbus_set_remote_state;
   cfg->state = TcpModbusDevStaActive;
   return VV_E_NONE;
}


int tcp_modbus_cfg_test_init(void) 

{
#if 0
   // we set to two by test
   modbusTermCnt = 1;
   gModbusDevCfg[0].devNum = 1;
   gModbusDevCfg[0].portNum = 502;
   strcpy(gModbusDevCfg[0].ipAddr, "192.168.1.201");
   gModbusDevCfg[1].devNum = 2;
   gModbusDevCfg[1].portNum = 202;
   strcpy(gModbusDevCfg[1].ipAddr, "192.168.1.202");
   modbusTermCnt = 2;
#else
   modbusTermCnt = 1;
   gModbusDevCfg[0].devNum = 2;
   gModbusDevCfg[0].portNum = 202;
   strcpy(gModbusDevCfg[0].ipAddr, "192.168.1.202");
#endif   
   return VV_E_NONE;
}

int tcp_modbus_cfg_init(void) 
{
   int i;
   
   for (i = 0; i < MAX_MODBUS_ELEMENT_CNT; i++) {
       tcp_modbus_cfg_reset_to_default(&gModbusDevCfg[i]);   
   }

   /*

    * Load cfg from config files

    */
   tcp_modbus_cfg_test_init();
   return VV_E_NONE;
}

int tcp_modbus_di_bits_read(modbus_t *ctx, uint8_t *data)
{
    return VV_E_NONE;
}   

int tcp_modbus_do_bits_read(modbus_t *ctx, uint8_t *data) 
{
    return VV_E_NONE;
}

int tcp_modbus_do_bits_write(modbus_t *ctx, uint8_t *data) 
{
    return VV_E_NONE;
}

int tcp_modbus_mac_serial_num_read(modbus_t *ctx, uint8_t *mac) 
{
    return VV_E_NONE;
}


int main (int argc, char *argv[]) 
{
    int ret = VV_E_NONE;
    pthread_t poll_cfg_thread_id, kick_off_keepalive_id;

    tcp_modbus_cfg_init();

    ret = pthread_create(&kick_off_keepalive_id, NULL, tcp_modbus_kick_off_keepalive_thread, NULL);
    if (ret != VV_E_NONE) {
        VV_PRINTF(("create tcp_modbus_kick_off_keepalive_thread failed\r\n")); 
        return ret;
    }

    ret = pthread_create(&poll_cfg_thread_id, NULL, tcp_modbus_cfg_thread, NULL);
    if (ret != VV_E_NONE) {
        VV_PRINTF(("create tcp_modbus_cfg_thread failed\r\n")); 
        return ret;
    }

    VV_PRINTF(("Finished init\r\n"));
    while (1) {
        //VV_PRINTF(("Finished init\r\n")); 
        usleep(100000);
    }
    
    return VV_E_NONE;
}

int main_new(int argc, char *argv[])
{
    uint8_t *tab_rp_bits;
    uint16_t *tab_rp_registers;
    uint16_t *tab_rp_registers_bad;
    modbus_t *ctx;
    int i;
    uint8_t value;
    int nb_points;
    int rc;
    float real;
    uint32_t ireal;
    struct timeval old_response_timeout;
    struct timeval response_timeout;
    int use_backend;

    if (argc > 1) {
        if (strcmp(argv[1], "tcp") == 0) {
            use_backend = TCP;
	} else if (strcmp(argv[1], "tcppi") == 0) {
            use_backend = TCP_PI;
        } else if (strcmp(argv[1], "rtu") == 0) {
            use_backend = RTU;
        } else {
            printf("Usage:\n  %s [tcp|tcppi|rtu] - Modbus client for unit testing\n\n", argv[0]);
            exit(1);
        }
    } else {
        /* By default */
        use_backend = TCP;
    }

    if (use_backend == TCP) {
       ctx = modbus_new_tcp("192.168.1.202", 202);
       modbus_set_slave(ctx, 2);
    } else if (use_backend == TCP_PI) {
        ctx = modbus_new_tcp_pi("::1", "1502");
    } else {
        ctx = modbus_new_rtu("/dev/ttyUSB1", 115200, 'N', 8, 1);
    }
    if (ctx == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return -1;
    }
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx,
                              MODBUS_ERROR_RECOVERY_LINK |
                              MODBUS_ERROR_RECOVERY_PROTOCOL);

    if (use_backend == RTU) {
          modbus_set_slave(ctx, SERVER_ID);
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    /* Allocate and initialize the memory to store the bits */
    nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
    tab_rp_bits = (uint8_t *) malloc(nb_points * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));

    /* Allocate and initialize the memory to store the registers */
    nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    printf("** UNIT TESTING **\n");

    printf("\nTEST WRITE/READ:\n");

    /** COIL BITS **/

    /* Single */
   while (1) {
    rc = modbus_write_bit(ctx, 17, ON);
    printf("1/2 modbus_write_bit: ");
    if (rc == 1) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }
    }
    rc = modbus_read_bits(ctx, 17, 1, tab_rp_bits);
    printf("2/2 modbus_read_bits: ");
    if (rc != 1) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }
    
    if (tab_rp_bits[0] != ON) {
        printf("FAILED (%0X = != %0X)\n", tab_rp_bits[0], ON);
        goto close;
    }
    printf("OK\n");
    /* End single */

    /* Multiple bits */
    {
        uint8_t tab_value[UT_BITS_NB];

        modbus_set_bits_from_bytes(tab_value, 0, UT_BITS_NB, UT_BITS_TAB);
        rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                               UT_BITS_NB, tab_value);
        printf("1/2 modbus_write_bits: ");
        if (rc == UT_BITS_NB) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
    printf("2/2 modbus_read_bits: ");
    if (rc != UT_BITS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_BITS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n", value, UT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printf("OK\n");
    /* End of multiple bits */

    /** DISCRETE INPUTS **/
    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB, tab_rp_bits);
    printf("1/1 modbus_read_input_bits: ");

    if (rc != UT_INPUT_BITS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_INPUT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_INPUT_BITS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n", value, UT_INPUT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printf("OK\n");

    /** HOLDING REGISTERS **/

    /* Single register */
    rc = modbus_write_register(ctx, UT_REGISTERS_ADDRESS, 0x1234);
    printf("1/2 modbus_write_register: ");
    if (rc == 1) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               1, tab_rp_registers);
    printf("2/2 modbus_read_registers: ");
    if (rc != 1) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    if (tab_rp_registers[0] != 0x1234) {
        printf("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], 0x1234);
        goto close;
    }
    printf("OK\n");
    /* End of single register */

    /* Many registers */
    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                UT_REGISTERS_NB, UT_REGISTERS_TAB);
    printf("1/5 modbus_write_registers: ");
    if (rc == UT_REGISTERS_NB) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("2/5 modbus_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_REGISTERS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i],
                   UT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printf("OK\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               0, tab_rp_registers);
    printf("3/5 modbus_read_registers (0): ");
    if (rc != -1 && errno == EMBMDATA) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }
    printf("OK\n");

    nb_points = (UT_REGISTERS_NB >
                 UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    /* Write registers to zero from tab_rp_registers and store read registers
       into tab_rp_registers. So the read registers must set to 0, except the
       first one because there is an offset of 1 register on write. */
    rc = modbus_write_and_read_registers(ctx,
                                         UT_REGISTERS_ADDRESS + 1, UT_REGISTERS_NB - 1,
                                         tab_rp_registers,
                                         UT_REGISTERS_ADDRESS,
                                         UT_REGISTERS_NB,
                                         tab_rp_registers);
    printf("4/5 modbus_write_and_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printf("FAILED (nb points %d != %d)\n", rc, UT_REGISTERS_NB);
        goto close;
    }

    if (tab_rp_registers[0] != UT_REGISTERS_TAB[0]) {
        printf("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], UT_REGISTERS_TAB[0]);
    }

    for (i=1; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != 0) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], 0);
            goto close;
        }
    }
    printf("OK\n");

    /* End of many registers */


    /** INPUT REGISTERS **/
    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB,
                                     tab_rp_registers);
    printf("1/1 modbus_read_input_registers: ");
    if (rc != UT_INPUT_REGISTERS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_INPUT_REGISTERS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], UT_INPUT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printf("OK\n");

    printf("\nTEST FLOATS\n");
    /** FLOAT **/
    printf("1/2 Set float: ");
    modbus_set_float(UT_REAL, tab_rp_registers);
    if (tab_rp_registers[1] == (UT_IREAL >> 16) &&
        tab_rp_registers[0] == (UT_IREAL & 0xFFFF)) {
        printf("OK\n");
    } else {
        /* Avoid *((uint32_t *)tab_rp_registers)
         * https://github.com/stephane/libmodbus/pull/104 */
        ireal = (uint32_t) tab_rp_registers[0] & 0xFFFF;
        ireal |= (uint32_t) tab_rp_registers[1] << 16;
        printf("FAILED (%x != %x)\n", ireal, UT_IREAL);
        goto close;
    }

    printf("2/2 Get float: ");
    real = modbus_get_float(tab_rp_registers);
    if (real == UT_REAL) {
        printf("OK\n");
    } else {
        printf("FAILED (%f != %f)\n", real, UT_REAL);
        goto close;
    }

    printf("\nAt this point, error messages doesn't mean the test has failed\n");

    /** ILLEGAL DATA ADDRESS **/
    printf("\nTEST ILLEGAL DATA ADDRESS:\n");

    /* The mapping begins at 0 and ends at address + nb_points so
     * the addresses are not valid. */

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          UT_BITS_NB + 1, tab_rp_bits);
    printf("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB + 1, tab_rp_bits);
    printf("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB + 1, tab_rp_registers);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB + 1,
                                     tab_rp_registers);
    printf("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS + UT_BITS_NB, ON);
    printf("* modbus_write_bit: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS + UT_BITS_NB,
                           UT_BITS_NB, tab_rp_bits);
    printf("* modbus_write_coils: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS + UT_REGISTERS_NB,
                                UT_REGISTERS_NB, tab_rp_registers);
    printf("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /** TOO MANY DATA **/
    printf("\nTEST TOO MANY DATA ERROR:\n");

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printf("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printf("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               MODBUS_MAX_READ_REGISTERS + 1,
                               tab_rp_registers);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     MODBUS_MAX_READ_REGISTERS + 1,
                                     tab_rp_registers);
    printf("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                           MODBUS_MAX_WRITE_BITS + 1, tab_rp_bits);
    printf("* modbus_write_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        goto close;
        printf("FAILED\n");
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                MODBUS_MAX_WRITE_REGISTERS + 1,
                                tab_rp_registers);
    printf("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /** SLAVE REPLY **/
    printf("\nTEST SLAVE REPLY:\n");
    modbus_set_slave(ctx, INVALID_SERVER_ID);
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    if (use_backend == RTU) {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { INVALID_SERVER_ID, 0x03, 0x00, 0x01, 0xFF, 0xFF };
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        /* No response in RTU mode */
        printf("1/4-A No response from slave %d: ", INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }

        /* Send an invalid query with a wrong slave ID */
        modbus_send_raw_request(ctx, raw_req,
                                RAW_REQ_LENGTH * sizeof(uint8_t));
        rc = modbus_receive_confirmation(ctx, rsp);

        printf("1/4-B No response from slave %d with invalid request: ",
               INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", rc);
            goto close;
        }

    } else {
        /* Response in TCP mode */
        printf("1/4 Response from slave %d: ", 18);

        if (rc == UT_REGISTERS_NB) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }
    }

    rc = modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
    if (rc == -1) {
        printf("Invalid broacast address\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("2/4 Reply after a broadcast query: ");
    if (rc == UT_REGISTERS_NB) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Restore slave */
    if (use_backend == RTU) {
        modbus_set_slave(ctx, SERVER_ID);
    } else {
        modbus_set_slave(ctx, MODBUS_TCP_SLAVE);
    }

    printf("3/4 Report slave ID: \n");
    /* tab_rp_bits is used to store bytes */
    rc = modbus_report_slave_id(ctx, tab_rp_bits);
    if (rc == -1) {
        printf("FAILED\n");
        goto close;
    }

    /* Slave ID is an arbitraty number for libmodbus */
    if (rc > 0) {
        printf("OK Slave ID is %d\n", tab_rp_bits[0]);
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Run status indicator */
    if (rc > 1 && tab_rp_bits[1] == 0xFF) {
        printf("OK Run Status Indicator is %s\n", tab_rp_bits[1] ? "ON" : "OFF");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Print additional data as string */
    if (rc > 2) {
        printf("Additional data: ");
        for (i=2; i < rc; i++) {
            printf("%c", tab_rp_bits[i]);
        }
        printf("\n");
    }

    /* Save original timeout */
    modbus_get_response_timeout(ctx, &old_response_timeout);

    /* Define a new and too short timeout */
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 0;
    modbus_set_response_timeout(ctx, &response_timeout);

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("4/4 Too short timeout: ");
    if (rc == -1 && errno == ETIMEDOUT) {
        printf("OK\n");
    } else {
        printf("FAILED (can fail on slow systems or Windows)\n");
    }

    /* Restore original timeout */
    modbus_set_response_timeout(ctx, &old_response_timeout);

    /* A wait and flush operation is done by the error recovery code of
     * libmodbus */

    /** BAD RESPONSE **/
    printf("\nTEST BAD RESPONSE ERROR:\n");

    /* Allocate only the required space */
    tab_rp_registers_bad = (uint16_t *) malloc(
        UT_REGISTERS_NB_SPECIAL * sizeof(uint16_t));
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB_SPECIAL, tab_rp_registers_bad);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBBADDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    free(tab_rp_registers_bad);

    /** MANUAL EXCEPTION **/
    printf("\nTEST MANUAL EXCEPTION:\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS_SPECIAL,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("* modbus_read_registers at special address: ");
    if (rc == -1 && errno == EMBXSBUSY) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /** RAW REQUEST */
    printf("\nTEST RAW REQUEST:\n");
    {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { (use_backend == RTU) ? SERVER_ID : 0xFF,
                              0x03, 0x00, 0x01, 0x0, 0x05 };
        int req_length;
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        req_length = modbus_send_raw_request(ctx, raw_req,
                                             RAW_REQ_LENGTH * sizeof(uint8_t));

        printf("* modbus_send_raw_request: ");
        if ((use_backend == RTU && req_length == (RAW_REQ_LENGTH + 2)) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             req_length == (RAW_REQ_LENGTH + 6))) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", req_length);
            goto close;
        }

        printf("* modbus_receive_confirmation: ");
        rc  = modbus_receive_confirmation(ctx, rsp);
        if ((use_backend == RTU && rc == 15) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             rc == 19)) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", rc);
            goto close;
        }
    }

    printf("\nALL TESTS PASS WITH SUCCESS.\n");

close:
    /* Free the memory */
    free(tab_rp_bits);
    free(tab_rp_registers);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}



int main_origin(int argc, char *argv[])
{
    uint8_t *tab_rp_bits;
    uint16_t *tab_rp_registers;
    uint16_t *tab_rp_registers_bad;
    modbus_t *ctx;
    int i;
    uint8_t value;
    int nb_points;
    int rc;
    float real;
    uint32_t ireal;
    struct timeval old_response_timeout;
    struct timeval response_timeout;
    int use_backend;

    if (argc > 1) {
        if (strcmp(argv[1], "tcp") == 0) {
            use_backend = TCP;
	} else if (strcmp(argv[1], "tcppi") == 0) {
            use_backend = TCP_PI;
        } else if (strcmp(argv[1], "rtu") == 0) {
            use_backend = RTU;
        } else {
            printf("Usage:\n  %s [tcp|tcppi|rtu] - Modbus client for unit testing\n\n", argv[0]);
            exit(1);
        }
    } else {
        /* By default */
        use_backend = TCP;
    }

    if (use_backend == TCP) {
       ctx = modbus_new_tcp("192.168.1.202", 202);
       modbus_set_slave(ctx, 2);
    } else if (use_backend == TCP_PI) {
        ctx = modbus_new_tcp_pi("::1", "1502");
    } else {
        ctx = modbus_new_rtu("/dev/ttyUSB1", 115200, 'N', 8, 1);
    }
    if (ctx == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return -1;
    }
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx,
                              MODBUS_ERROR_RECOVERY_LINK |
                              MODBUS_ERROR_RECOVERY_PROTOCOL);

    if (use_backend == RTU) {
          modbus_set_slave(ctx, SERVER_ID);
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    /* Allocate and initialize the memory to store the bits */
    nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
    tab_rp_bits = (uint8_t *) malloc(nb_points * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));

    /* Allocate and initialize the memory to store the registers */
    nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    printf("** UNIT TESTING **\n");

    printf("\nTEST WRITE/READ:\n");

    /** COIL BITS **/

    /* Single */
   while (1) {
    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, ON);
    printf("1/2 modbus_write_bit: ");
    if (rc == 1) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
    printf("2/2 modbus_read_bits: ");
    if (rc != 1) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }
   
    if (tab_rp_bits[0] != ON) {
        printf("FAILED (%0X = != %0X)\n", tab_rp_bits[0], ON);
        goto close;
    }
    printf("OK\n");
    /* End single */

    /* Multiple bits */
    {
        uint8_t tab_value[UT_BITS_NB];

        modbus_set_bits_from_bytes(tab_value, 0, UT_BITS_NB, UT_BITS_TAB);
        rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                               UT_BITS_NB, tab_value);
        printf("1/2 modbus_write_bits: ");
        if (rc == UT_BITS_NB) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
    printf("2/2 modbus_read_bits: ");
    if (rc != UT_BITS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_BITS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n", value, UT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printf("OK\n");
    /* End of multiple bits */

    /** DISCRETE INPUTS **/
    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB, tab_rp_bits);
    printf("1/1 modbus_read_input_bits: ");

    if (rc != UT_INPUT_BITS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    i = 0;
    nb_points = UT_INPUT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        if (value != UT_INPUT_BITS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n", value, UT_INPUT_BITS_TAB[i]);
            goto close;
        }

        nb_points -= nb_bits;
        i++;
    }
    printf("OK\n");

    /** HOLDING REGISTERS **/

    /* Single register */
    rc = modbus_write_register(ctx, UT_REGISTERS_ADDRESS, 0x1234);
    printf("1/2 modbus_write_register: ");
    if (rc == 1) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               1, tab_rp_registers);
    printf("2/2 modbus_read_registers: ");
    if (rc != 1) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    if (tab_rp_registers[0] != 0x1234) {
        printf("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], 0x1234);
        goto close;
    }
    printf("OK\n");
    /* End of single register */

    /* Many registers */
    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                UT_REGISTERS_NB, UT_REGISTERS_TAB);
    printf("1/5 modbus_write_registers: ");
    if (rc == UT_REGISTERS_NB) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("2/5 modbus_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_REGISTERS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i],
                   UT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printf("OK\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               0, tab_rp_registers);
    printf("3/5 modbus_read_registers (0): ");
    if (rc != -1 && errno == EMBMDATA) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }
    printf("OK\n");

    nb_points = (UT_REGISTERS_NB >
                 UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    /* Write registers to zero from tab_rp_registers and store read registers
       into tab_rp_registers. So the read registers must set to 0, except the
       first one because there is an offset of 1 register on write. */
    rc = modbus_write_and_read_registers(ctx,
                                         UT_REGISTERS_ADDRESS + 1, UT_REGISTERS_NB - 1,
                                         tab_rp_registers,
                                         UT_REGISTERS_ADDRESS,
                                         UT_REGISTERS_NB,
                                         tab_rp_registers);
    printf("4/5 modbus_write_and_read_registers: ");
    if (rc != UT_REGISTERS_NB) {
        printf("FAILED (nb points %d != %d)\n", rc, UT_REGISTERS_NB);
        goto close;
    }

    if (tab_rp_registers[0] != UT_REGISTERS_TAB[0]) {
        printf("FAILED (%0X != %0X)\n",
               tab_rp_registers[0], UT_REGISTERS_TAB[0]);
    }

    for (i=1; i < UT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != 0) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], 0);
            goto close;
        }
    }
    printf("OK\n");

    /* End of many registers */


    /** INPUT REGISTERS **/
    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB,
                                     tab_rp_registers);
    printf("1/1 modbus_read_input_registers: ");
    if (rc != UT_INPUT_REGISTERS_NB) {
        printf("FAILED (nb points %d)\n", rc);
        goto close;
    }

    for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        if (tab_rp_registers[i] != UT_INPUT_REGISTERS_TAB[i]) {
            printf("FAILED (%0X != %0X)\n",
                   tab_rp_registers[i], UT_INPUT_REGISTERS_TAB[i]);
            goto close;
        }
    }
    printf("OK\n");

    printf("\nTEST FLOATS\n");
    /** FLOAT **/
    printf("1/2 Set float: ");
    modbus_set_float(UT_REAL, tab_rp_registers);
    if (tab_rp_registers[1] == (UT_IREAL >> 16) &&
        tab_rp_registers[0] == (UT_IREAL & 0xFFFF)) {
        printf("OK\n");
    } else {
        /* Avoid *((uint32_t *)tab_rp_registers)
         * https://github.com/stephane/libmodbus/pull/104 */
        ireal = (uint32_t) tab_rp_registers[0] & 0xFFFF;
        ireal |= (uint32_t) tab_rp_registers[1] << 16;
        printf("FAILED (%x != %x)\n", ireal, UT_IREAL);
        goto close;
    }

    printf("2/2 Get float: ");
    real = modbus_get_float(tab_rp_registers);
    if (real == UT_REAL) {
        printf("OK\n");
    } else {
        printf("FAILED (%f != %f)\n", real, UT_REAL);
        goto close;
    }

    printf("\nAt this point, error messages doesn't mean the test has failed\n");

    /** ILLEGAL DATA ADDRESS **/
    printf("\nTEST ILLEGAL DATA ADDRESS:\n");

    /* The mapping begins at 0 and ends at address + nb_points so
     * the addresses are not valid. */

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          UT_BITS_NB + 1, tab_rp_bits);
    printf("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB + 1, tab_rp_bits);
    printf("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB + 1, tab_rp_registers);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     UT_INPUT_REGISTERS_NB + 1,
                                     tab_rp_registers);
    printf("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBXILADD)
        printf("OK\n");
    else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS + UT_BITS_NB, ON);
    printf("* modbus_write_bit: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS + UT_BITS_NB,
                           UT_BITS_NB, tab_rp_bits);
    printf("* modbus_write_coils: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS + UT_REGISTERS_NB,
                                UT_REGISTERS_NB, tab_rp_registers);
    printf("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBXILADD) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    }
    /** TOO MANY DATA **/
    printf("\nTEST TOO MANY DATA ERROR:\n");

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS,
                          MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printf("* modbus_read_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                MODBUS_MAX_READ_BITS + 1, tab_rp_bits);
    printf("* modbus_read_input_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               MODBUS_MAX_READ_REGISTERS + 1,
                               tab_rp_registers);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_read_input_registers(ctx, UT_INPUT_REGISTERS_ADDRESS,
                                     MODBUS_MAX_READ_REGISTERS + 1,
                                     tab_rp_registers);
    printf("* modbus_read_input_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    rc = modbus_write_bits(ctx, UT_BITS_ADDRESS,
                           MODBUS_MAX_WRITE_BITS + 1, tab_rp_bits);
    printf("* modbus_write_bits: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        goto close;
        printf("FAILED\n");
    }

    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                MODBUS_MAX_WRITE_REGISTERS + 1,
                                tab_rp_registers);
    printf("* modbus_write_registers: ");
    if (rc == -1 && errno == EMBMDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /** SLAVE REPLY **/
    printf("\nTEST SLAVE REPLY:\n");
    modbus_set_slave(ctx, INVALID_SERVER_ID);
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    if (use_backend == RTU) {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { INVALID_SERVER_ID, 0x03, 0x00, 0x01, 0xFF, 0xFF };
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        /* No response in RTU mode */
        printf("1/4-A No response from slave %d: ", INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }

        /* Send an invalid query with a wrong slave ID */
        modbus_send_raw_request(ctx, raw_req,
                                RAW_REQ_LENGTH * sizeof(uint8_t));
        rc = modbus_receive_confirmation(ctx, rsp);

        printf("1/4-B No response from slave %d with invalid request: ",
               INVALID_SERVER_ID);

        if (rc == -1 && errno == ETIMEDOUT) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", rc);
            goto close;
        }

    } else {
        /* Response in TCP mode */
        printf("1/4 Response from slave %d: ", 18);

        if (rc == UT_REGISTERS_NB) {
            printf("OK\n");
        } else {
            printf("FAILED\n");
            goto close;
        }
    }

    rc = modbus_set_slave(ctx, MODBUS_BROADCAST_ADDRESS);
    if (rc == -1) {
        printf("Invalid broacast address\n");
        goto close;
    }

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("2/4 Reply after a broadcast query: ");
    if (rc == UT_REGISTERS_NB) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Restore slave */
    if (use_backend == RTU) {
        modbus_set_slave(ctx, SERVER_ID);
    } else {
        modbus_set_slave(ctx, MODBUS_TCP_SLAVE);
    }

    printf("3/4 Report slave ID: \n");
    /* tab_rp_bits is used to store bytes */
    rc = modbus_report_slave_id(ctx, tab_rp_bits);
    if (rc == -1) {
        printf("FAILED\n");
        goto close;
    }

    /* Slave ID is an arbitraty number for libmodbus */
    if (rc > 0) {
        printf("OK Slave ID is %d\n", tab_rp_bits[0]);
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Run status indicator */
    if (rc > 1 && tab_rp_bits[1] == 0xFF) {
        printf("OK Run Status Indicator is %s\n", tab_rp_bits[1] ? "ON" : "OFF");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /* Print additional data as string */
    if (rc > 2) {
        printf("Additional data: ");
        for (i=2; i < rc; i++) {
            printf("%c", tab_rp_bits[i]);
        }
        printf("\n");
    }

    /* Save original timeout */
    modbus_get_response_timeout(ctx, &old_response_timeout);

    /* Define a new and too short timeout */
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 0;
    modbus_set_response_timeout(ctx, &response_timeout);

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("4/4 Too short timeout: ");
    if (rc == -1 && errno == ETIMEDOUT) {
        printf("OK\n");
    } else {
        printf("FAILED (can fail on slow systems or Windows)\n");
    }

    /* Restore original timeout */
    modbus_set_response_timeout(ctx, &old_response_timeout);

    /* A wait and flush operation is done by the error recovery code of
     * libmodbus */

    /** BAD RESPONSE **/
    printf("\nTEST BAD RESPONSE ERROR:\n");

    /* Allocate only the required space */
    tab_rp_registers_bad = (uint16_t *) malloc(
        UT_REGISTERS_NB_SPECIAL * sizeof(uint16_t));
    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB_SPECIAL, tab_rp_registers_bad);
    printf("* modbus_read_registers: ");
    if (rc == -1 && errno == EMBBADDATA) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    free(tab_rp_registers_bad);

    /** MANUAL EXCEPTION **/
    printf("\nTEST MANUAL EXCEPTION:\n");

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS_SPECIAL,
                               UT_REGISTERS_NB, tab_rp_registers);
    printf("* modbus_read_registers at special address: ");
    if (rc == -1 && errno == EMBXSBUSY) {
        printf("OK\n");
    } else {
        printf("FAILED\n");
        goto close;
    }

    /** RAW REQUEST */
    printf("\nTEST RAW REQUEST:\n");
    {
        const int RAW_REQ_LENGTH = 6;
        uint8_t raw_req[] = { (use_backend == RTU) ? SERVER_ID : 0xFF,
                              0x03, 0x00, 0x01, 0x0, 0x05 };
        int req_length;
        uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

        req_length = modbus_send_raw_request(ctx, raw_req,
                                             RAW_REQ_LENGTH * sizeof(uint8_t));

        printf("* modbus_send_raw_request: ");
        if ((use_backend == RTU && req_length == (RAW_REQ_LENGTH + 2)) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             req_length == (RAW_REQ_LENGTH + 6))) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", req_length);
            goto close;
        }

        printf("* modbus_receive_confirmation: ");
        rc  = modbus_receive_confirmation(ctx, rsp);
        if ((use_backend == RTU && rc == 15) ||
            ((use_backend == TCP || use_backend == TCP_PI) &&
             rc == 19)) {
            printf("OK\n");
        } else {
            printf("FAILED (%d)\n", rc);
            goto close;
        }
    }

    printf("\nALL TESTS PASS WITH SUCCESS.\n");

close:
    /* Free the memory */
    free(tab_rp_bits);
    free(tab_rp_registers);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);

    return 0;
}
