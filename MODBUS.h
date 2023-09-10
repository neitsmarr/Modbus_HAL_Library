#ifndef __MODBUS_H
#define __MODBUS_H

#include "main.h"

typedef enum
{
	input_registers		= 0,
	holding_registers	= 1
} register_type_t;

typedef struct __modbus_init_t
{
	uint8_t				slave_id;	//communication parameters
	uint8_t				baudrate;	//pointer to holding register buffer
	uint8_t				parity;		//number of input registers
} modbus_init_t;

typedef struct __address_space_t
{
	register_type_t		type;
	uint16_t			start_offset;
	uint16_t			size;
	uint16_t			*address;
} address_space_t;

typedef struct __modbus_hanle_t
{
	modbus_init_t		init;					//communication parameters
//	const uint16_t		*input_registers;		//pointer to input register buffer
//	uint8_t				ir_quantity;			//number of input registers
//	const uint16_t		*holding_registers;		//pointer to holding register buffer
//	uint8_t				hr_quantity;			//number of holding registers
	uint32_t			mode;					//slave / master / slave+master
	UART_HandleTypeDef	*huart;					//pointer to UART handle
	HAL_LockTypeDef		Lock;					//locking object (useful for RTOS)
	uint32_t			ErrorCode;				//error code
	address_space_t		address_spaces[0x10];
	uint8_t				num_address_spaces;
} modbus_handle_t;


/*MODBUS LIBRARY SETTINGS*/

/*FUNCTIONS THAT CAN BE USED IN OTHER MODULES*/
uint32_t FEE_Get_Version(void);
void MBR_Init_Modbus(modbus_handle_t *hmodbus);	//call this function in main.c after initialization of all hardware
void MBR_Add_Address_Space(modbus_handle_t *hmodbus, address_space_t *address_space);
void MBR_Remove_Address_Space(modbus_handle_t *hmodbus, uint16_t *address);

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

/*BUFFERS AND FLAGS THAT CAN BE USED IN OTHER MODULES [READ-ONLY]*/
/*buffers*/

/*flags*/
extern uint8_t flg_modbus_no_comm;	//raises after uint_hold_reg[7] seconds
extern uint8_t flg_modbus_packet_received;




#endif
