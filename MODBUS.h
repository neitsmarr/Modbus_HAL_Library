#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

typedef enum
{
	input_registers		= 0,
	holding_registers	= 1
} register_type_t;


typedef struct __address_space_t address_space_t;
typedef struct __modbus_hanle_t modbus_handle_t;


/*FUNCTIONS THAT CAN BE USED IN OTHER MODULES*/
uint32_t FEE_Get_Version(void);

modbus_handle_t *MBR_Init_Modbus(UART_HandleTypeDef *huart);	//call this function in main.c after initialization of all hardware
void MBR_Destroy_Modbus(modbus_handle_t *hmodbus);

address_space_t *MBR_Init_Address_Space(register_type_t type, uint16_t start_offset, uint16_t size, uint16_t *address);
void MBR_Add_Address_Space(modbus_handle_t *hmodbus, address_space_t *address_space);
void MBR_Remove_Address_Space(modbus_handle_t *hmodbus, uint16_t *address);

void MBR_Set_Communication_Parameters(modbus_handle_t *hmodbus, uint8_t slave_id, uint8_t baudrate, uint8_t parity);

void MBR_Check_For_Request(modbus_handle_t *hmodbus);

uint8_t MBR_Check_Restrictions_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t register_data);	//weak ref, can be defined in other modules. return 0 when OK, return 1 when NOK

void MBR_Register_Update_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t register_data);
void MBR_Register_Read_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t *register_data);
void MBR_Register_Init_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t *register_data);

void MBR_Start_Sending_Callback(UART_HandleTypeDef *huart);
void MBR_End_Sending_Callback(UART_HandleTypeDef *huart);

void MBR_Update_Communication_Parameters(modbus_handle_t *hmodbus);

void MBR_Communication_Lost_Callback(modbus_handle_t *hmodbus);
void MBR_Communication_Restored_Callback(modbus_handle_t *hmodbus);

uint32_t Custom_Command_Callback(modbus_handle_t *hmodbus);

#endif
