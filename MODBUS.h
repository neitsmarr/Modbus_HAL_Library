#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

/*DEFINE NUMBER OF REGISTERS*/
#define I_REG_COUNT				10	//number of the input registers
#define H_REG_COUNT				60	//number of the holding registers

/*MODBUS LIBRARY SETTINGS*/

/*FUNCTIONS THAT CAN BE USED IN OTHER MODULES*/
uint32_t FEE_Get_Version(void);
void MBR_Init_Modbus(UART_HandleTypeDef *huart, void *read_handler, void *write_handler);	//call this function in main.c after initialisation of all hardware
void MBR_Check_For_Request(UART_HandleTypeDef *huart);
void MBR_Switch_DE_Callback(uint8_t state);	//weak ref, can be defined in other modules. state variants: 0=reset_DERE, 1=set_DERE
uint8_t MBR_Check_Restrictions_Callback(uint16_t register_address, uint16_t register_data);	//weak ref, can be defined in other modules. return 0 when OK, return 1 when NOK
void MBR_Register_Update_Callback(uint16_t register_address, uint16_t register_data);
void MBR_Register_Read_Callback(uint16_t register_address, uint16_t *register_data);
void MBR_Register_Init_Callback(uint16_t register_address, uint16_t *register_data);
void MBR_Start_Sending_Callback(UART_HandleTypeDef *huart);
void MBR_End_Sending_Callback(UART_HandleTypeDef *huart);
void MBR_Update_Communication_Parameters(UART_HandleTypeDef *huart);


/*BUFFERS AND FLAGS THAT CAN BE USED IN OTHER MODULES [READ-ONLY]*/
/*buffers*/

/*flags*/
extern uint8_t flg_modbus_no_comm;	//raises after uint_hold_reg[7] seconds
extern uint8_t flg_modbus_packet_received;



#endif
