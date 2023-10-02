/*MODBUS.c*/
#include "MODBUS.h"
#include <stdlib.h>
#include <string.h>

enum
{
	MBR_VERSION_MAJOR = 0x00,
	MBR_VERSION_MINOR = 0x08,
	MBR_VERSION_PATCH = 0x00
};


#define MODBUS_BUFFER_SIZE			0x100

/*Modbus function codes*/
enum function_code_e
{
	read_holding_registers = 0x03,
	read_input_registers = 0x04,
	write_single_register = 0x06,
	write_multiple_registers = 0x10,
	exception = 0x80
};


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
	uint32_t			mode;					//slave / master / slave+master
	UART_HandleTypeDef	*huart;					//pointer to UART handle
	HAL_LockTypeDef		Lock;					//locking object (useful for RTOS)
	uint32_t			ErrorCode;				//error code
	address_space_t		*address_spaces[0x10];
	uint8_t				num_address_spaces;
} modbus_handle_t;


/*VARIABLES*/
/*for internal and external usage*/
uint8_t flg_modbus_no_comm;
/*for internal usage only*/
uint32_t last_communication_time;
uint8_t len_modbus_frame;
uint8_t buf_modbus[MODBUS_BUFFER_SIZE];
uint8_t flg_modbus_packet_received;

uint8_t communication_parity;
uint8_t communication_baudrate;
uint8_t communication_slave_id;

uint32_t test[2];

/*FUNCTION PROTOTYPES*/
/*for internal use only*/
static void Send_Response(modbus_handle_t *hmodbus, uint8_t payload_size);
static void Check_Frame(modbus_handle_t *hmodbus);
static void Process_Request(modbus_handle_t *hmodbus);
static void Send_Exeption(modbus_handle_t *hmodbus, uint8_t exeption_code);

/*PUBLIC FUNCTIONS*/
/**
 *
 */

/**
 * @brief Getting the version of Modbus library.
 * @param none
 * @retval version in format (MSB to LSB): 8bits: zeroed, 8bits: Major, 8bits: Minor, 8bits: Patch
 */
uint32_t MBR_Get_Version(void)
{
	return (MBR_VERSION_MAJOR<<16) | (MBR_VERSION_MINOR<<8) | (MBR_VERSION_PATCH);
}

/**
 * @brief Allocating the memory for Modbus handle and making initial setup.
 * @param huart UART handle.
 * @retval pointer to modbus handle
 */
modbus_handle_t *MBR_Init_Modbus(UART_HandleTypeDef *huart)
{
	modbus_handle_t *hmodbus;

	hmodbus = (modbus_handle_t*) malloc(sizeof(modbus_handle_t));
	memset(hmodbus, 0, sizeof(modbus_handle_t));

	hmodbus->huart = huart;

	//init usart and dma
	HAL_UART_ReceiverTimeout_Config(hmodbus->huart, 34);
	HAL_UART_EnableReceiverTimeout(hmodbus->huart);
	HAL_UART_Receive_DMA(hmodbus->huart, buf_modbus, 0x100);

	return hmodbus;
}

/**
 * @brief Allocating the memory for Address Space handle and making setup of the address space.
 * @param type Type of address space
 * @param start_offset Address of the first element in address space
 * @param size Number of elements in address space
 * @param address Pointer to array with actual values
 * @retval pointer to modbus handle
 */
address_space_t *MBR_Init_Address_Space(register_type_t type, uint16_t start_offset, uint16_t size, uint16_t *address)
{
	address_space_t *address_space;

	address_space = (address_space_t*) malloc(sizeof(address_space_t));

	address_space->type = type;
	address_space->start_offset = start_offset;
	address_space->size = size;
	address_space->address = address;

	return address_space;
}

void MBR_Destroy_Modbus(modbus_handle_t *hmodbus)
{
	//TODO free all address spaces?
	free(hmodbus);
}

void MBR_Add_Address_Space(modbus_handle_t *hmodbus, address_space_t *address_space)
{
	//if(hmodbus->num_address_spaces < 8)
	hmodbus->address_spaces[hmodbus->num_address_spaces] = address_space;
	hmodbus->num_address_spaces++;
}

void MBR_Remove_Address_Space(modbus_handle_t *hmodbus, uint16_t *address)
{
	uint8_t address_deleted;

	for(uint32_t i=0; i<hmodbus->num_address_spaces; i++)
	{
		if(hmodbus->address_spaces[i]->address == address)	//TODO address has not been found
		{
			free(hmodbus->address_spaces[i]);
			hmodbus->num_address_spaces--;
			address_deleted = i;
			break;
		}
	}

	for(uint32_t i=address_deleted; i<hmodbus->num_address_spaces; i++)
	{
		hmodbus->address_spaces[i] = hmodbus->address_spaces[i+1];
	}
}

/**
 * @brief Check for the new received Modbus request.
 * @param none
 * @retval none
 */
void MBR_Check_For_Request(modbus_handle_t *hmodbus)
{
	uint32_t modbus_no_comm, current_tick;

	if(flg_modbus_packet_received)
	{
		flg_modbus_packet_received = 0;

		Check_Frame(hmodbus);
	}
	else
	{
		if(flg_modbus_no_comm == 0)
		{
			current_tick = HAL_GetTick();

			if(current_tick > last_communication_time)	//TODO handle false condition
			{
				modbus_no_comm = current_tick - last_communication_time;
			}

			if(modbus_no_comm > 10*1000)	//TODO make it configurable
			{
				flg_modbus_no_comm = 1;
			}
		}
	}
}

/*CALLBACKS*/
/**
 * @brief This function is called every time when Modbus master tries to update holding register value.
 * @param none
 * @retval 0 = ok (new value is allowed), 1 = not ok (new value is not allowed)
 */
__weak uint8_t MBR_Check_Restrictions_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t register_data)
{
	UNUSED(register_address);
	UNUSED(register_data);
	return 0;
}

/**
 * @brief This function is called when holding register value has been updated.
 * @param none
 * @retval none
 */
__weak void MBR_Register_Update_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t register_data)
{
	UNUSED(register_address);
	UNUSED(register_data);
}

__weak void MBR_Register_Read_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t *register_data)
{
	UNUSED(hmodbus);
	UNUSED(register_address);
	UNUSED(register_data);
}

//__weak void MBR_Register_Init_Callback(modbus_handle_t *hmodbus, uint16_t register_address, uint16_t *register_data)
//{
//	UNUSED(register_address);
//	UNUSED(register_data);
//}

__weak void MBR_Start_Sending_Callback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
}

__weak void MBR_End_Sending_Callback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
}

//TODO replace with MBR_Add_Custom_Command()
__weak void MBR_Custom_Command_Callback(uint8_t *buf_modbus, response_t *response)
{
	UNUSED(buf_modbus);

	response->flg_response = 1;
	response->exception = illegal_function;
}

/*HAL CALLBACKS*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	test[0]++;
	if(huart->ErrorCode == HAL_UART_ERROR_RTO)
	{
		len_modbus_frame = MODBUS_BUFFER_SIZE - huart->hdmarx->Instance->CNDTR;
		if(len_modbus_frame > 7)	//minimum Modbus frame length (for requests)
		{
			flg_modbus_packet_received = 1;
		}
	}

	huart->ErrorCode = HAL_UART_ERROR_NONE;

	HAL_UART_Receive_DMA(huart, buf_modbus, MODBUS_BUFFER_SIZE);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	test[1]++;
	MBR_End_Sending_Callback(huart);
	HAL_UART_Receive_DMA(huart, buf_modbus, MODBUS_BUFFER_SIZE);
}


/*PRIVATE FUNCTIONS*/
uint16_t Calculate_CRC16(uint8_t *buf, uint16_t len)	//TODO compute table at Modbus_Init
{
	static const uint16_t crc_table[] = {
			0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
			0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
			0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
			0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
			0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
			0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
			0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
			0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
			0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
			0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
			0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
			0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
			0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
			0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
			0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
			0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
			0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
			0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
			0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
			0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
			0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
			0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
			0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
			0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
			0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
			0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
			0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
			0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
			0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
			0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
			0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
			0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

	uint8_t xor;
	uint16_t crc = 0xFFFF;

	while(len--)
	{
		xor = *buf++ ^ crc;
		crc >>= 8;
		crc ^= crc_table[xor];
	}

	return crc;
}


static void Check_Frame(modbus_handle_t *hmodbus)
{
	uint16_t crc_int;

	crc_int = (buf_modbus[len_modbus_frame-1]<<8) + buf_modbus[len_modbus_frame-2];	//get CRC16 bytes from the received packet

	if(crc_int == Calculate_CRC16(buf_modbus,(len_modbus_frame-2)))	// Check does the CRC match
	{
		if ((buf_modbus[0] == hmodbus->init.slave_id) || buf_modbus[0] == 0x00)	//Check if the device address is correct
		{
			Process_Request(hmodbus);	// Return flag OK;
			flg_modbus_no_comm = 0;
			last_communication_time = HAL_GetTick();
		}
	}
}

static void Read_Input_Registers(modbus_handle_t *hmodbus, struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t register_count, data;
	address_space_t *address_space;

	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];
	buf_modbus[2] = register_count*2;	// byte count

	response_s->exception = 0x02;
	for(uint32_t i=0; i<hmodbus->num_address_spaces; i++)
	{
		if(hmodbus->address_spaces[i]->type == input_registers)
		{
			if(start_address >= hmodbus->address_spaces[i]->start_offset && start_address + register_count <= hmodbus->address_spaces[i]->start_offset + hmodbus->address_spaces[i]->size)
			{
				response_s->exception = 0x00;
				address_space = hmodbus->address_spaces[i];
				break;
			}
		}
	}

	if(response_s->exception == 0)
	{
		for(uint32_t i = 0; i < register_count; i++)
		{
			MBR_Register_Read_Callback(hmodbus, start_address+i, &data);
			data = address_space->address[start_address-(address_space->start_offset)+i];
			buf_modbus[3+(i)*2] = data>>8;
			buf_modbus[4+(i)*2] = data;
		}
	}

	response_s->payload_size = 3 + buf_modbus[2];
}

static void Read_Holding_Registers(modbus_handle_t *hmodbus, struct response_s *response_s)
{
	uint16_t start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t register_count, data;
	address_space_t *address_space;

	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];
	buf_modbus[2] = register_count*2;	// byte count

	response_s->exception = 0x02;
	for(uint32_t i=0; i<hmodbus->num_address_spaces; i++)
	{
		if(hmodbus->address_spaces[i]->type == holding_registers)
		{
			if(start_address >= hmodbus->address_spaces[i]->start_offset && start_address + register_count <= hmodbus->address_spaces[i]->start_offset + hmodbus->address_spaces[i]->size)
			{
				response_s->exception = 0x00;
				address_space = hmodbus->address_spaces[i];
				break;
			}
		}
	}

	if(response_s->exception == 0)
	{

		for(uint32_t i = 0; i < register_count; i++)
		{
			MBR_Register_Read_Callback(hmodbus, start_address+i, &data);
			data = address_space->address[start_address-(address_space->start_offset)+i];
			buf_modbus[3+(i)*2] = data>>8;
			buf_modbus[4+(i)*2] = data;
		}
	}

	response_s->payload_size = 3 + buf_modbus[2];

	if(start_address == 0 && register_count == 4)	//response to broadcast
	{
		response_s->flg_response = 1;
	}
}

static void Write_Multiple_Registers(modbus_handle_t *hmodbus, struct response_s *response_s)
{
	uint16_t start_address, register_count;
	uint16_t reg_data;
	uint16_t uint_hold_reg_temporary[125/*hold_reg_count*/] = {0};
	address_space_t *address_space;

	start_address  = (buf_modbus[2]<<8)+ buf_modbus[3];
	register_count =  (buf_modbus[4]<<8)+ buf_modbus[5];

	response_s->exception = 0x02;
	for(uint32_t i=0; i<hmodbus->num_address_spaces; i++)
	{
		if(hmodbus->address_spaces[i]->type == holding_registers)
		{
			if(start_address >= hmodbus->address_spaces[i]->start_offset && start_address + register_count <= hmodbus->address_spaces[i]->start_offset + hmodbus->address_spaces[i]->size)
			{
				response_s->exception = 0x00;
				address_space = hmodbus->address_spaces[i];
				break;
			}
		}
	}

	if(response_s->exception == 0)
	{
		for(uint32_t i = 0; i < register_count; i++)
		{
			reg_data = (buf_modbus[7+(i)*2]<<8) + buf_modbus[8+(i)*2];

			if(MBR_Check_Restrictions_Callback(hmodbus, start_address+i, reg_data))
			{
				response_s->exception = 0x03;
				break;
			}
			else
			{
				uint_hold_reg_temporary[i] = reg_data;
			}

		}
	}

	if(response_s->exception == 0)
	{
		for(uint32_t i = 0; i < register_count; i++)//write the new data;	//TODO move it under flag no_exception
		{
			address_space->address[start_address-(address_space->start_offset)+i] = uint_hold_reg_temporary[i];
			MBR_Register_Update_Callback(hmodbus, start_address+i, uint_hold_reg_temporary[i]);
		}

		response_s->payload_size = 6;
	}
}

static void Write_Single_Register(modbus_handle_t *hmodbus, struct response_s *response_s)
{
	uint16_t start_address = (buf_modbus[2]<<8)+ buf_modbus[3];
	uint16_t reg_data;
	address_space_t *address_space;

	response_s->exception = 0x02;
	for(uint32_t i=0; i<hmodbus->num_address_spaces; i++)
	{
		if(hmodbus->address_spaces[i]->type == holding_registers)
		{
			if(start_address >= hmodbus->address_spaces[i]->start_offset && start_address < hmodbus->address_spaces[i]->start_offset + hmodbus->address_spaces[i]->size)
			{
				response_s->exception = 0x00;
				address_space = hmodbus->address_spaces[i];
				break;
			}
		}
	}

	if(response_s->exception == 0)
	{
		reg_data = (buf_modbus[4]<<8)+ buf_modbus[5];

		if(MBR_Check_Restrictions_Callback(hmodbus, start_address, reg_data))
		{
			response_s->exception = 0x03;
		}
		else
		{
			address_space->address[start_address-(address_space->start_offset)] = reg_data;
			MBR_Register_Update_Callback(hmodbus, start_address, reg_data);
		}

		buf_modbus[4] = reg_data>>8;	//Register value 1st byte
		buf_modbus[5] = reg_data;	//Register value 2nd byte

		response_s->payload_size = 6;
	}
}

static void Process_Request(modbus_handle_t *hmodbus)
{
	struct response_s response_s = {0, 0, 0};

	if(buf_modbus[0])
	{
		response_s.flg_response = 1;
	}

	switch(buf_modbus[1])
	{
	case read_input_registers:
		Read_Input_Registers(hmodbus, &response_s);
		break;

	case read_holding_registers:
		Read_Holding_Registers(hmodbus, &response_s);
		break;

	case write_single_register:
		Write_Single_Register(hmodbus, &response_s);
		break;

	case write_multiple_registers:
		Write_Multiple_Registers(hmodbus, &response_s);
		break;

	default:	//if the command is not supported by default
		MBR_Custom_Command_Callback(buf_modbus, &response_s);
	}

	if(response_s.flg_response)
	{
		if(response_s.exception)
		{
			Send_Exeption(hmodbus, response_s.exception);
		}
		else
		{
			Send_Response(hmodbus, response_s.payload_size);	// Send packet response
		}
	}
}

static void Send_Response(modbus_handle_t *hmodbus, uint8_t payload_size)
{
	uint16_t crc16;

	crc16 = Calculate_CRC16(buf_modbus, payload_size);
	buf_modbus[payload_size] = crc16;									// CRC Lo byte
	buf_modbus[payload_size+1] = crc16>>8;								// CRC Hi byte

	MBR_Start_Sending_Callback(hmodbus->huart);
	HAL_UART_Transmit_DMA(hmodbus->huart, buf_modbus, payload_size+2);
}

static void Send_Exeption(modbus_handle_t *hmodbus, uint8_t exeption_code)
{
	buf_modbus[0] = communication_slave_id;		// Device address
	buf_modbus[1] += exception;					// Modbus error code (0x80+command)
	buf_modbus[2] = exeption_code;				// exception code

	Send_Response(hmodbus, 3);					// Send frame
}

void MBR_Set_Communication_Parameters(modbus_handle_t *hmodbus, uint8_t slave_id, uint8_t baudrate, uint8_t parity)
{
	UART_HandleTypeDef *huart;

	huart = hmodbus->huart;

	hmodbus->init.slave_id = slave_id;
	hmodbus->init.baudrate = baudrate;
	hmodbus->init.parity = parity;

	switch(hmodbus->init.baudrate)
	{
	case 0:
		huart->Init.BaudRate = 4800;
		HAL_UART_ReceiverTimeout_Config(huart, 39);
		break;
	case 1:
		huart->Init.BaudRate = 9600;
		HAL_UART_ReceiverTimeout_Config(huart, 39);
		break;
	case 2:
		huart->Init.BaudRate = 19200;
		HAL_UART_ReceiverTimeout_Config(huart, 39);
		break;
	case 3:
		huart->Init.BaudRate = 38400;
		HAL_UART_ReceiverTimeout_Config(huart, 67);
		break;
	case 4:
		huart->Init.BaudRate = 57600;
		HAL_UART_ReceiverTimeout_Config(huart, 101);
		break;
	case 5:
		huart->Init.BaudRate = 115200;
		HAL_UART_ReceiverTimeout_Config(huart, 202);
		break;
	case 6:
		huart->Init.BaudRate = 230400;
		HAL_UART_ReceiverTimeout_Config(huart, 403);
		break;
	} //default is 19200

	/*parity*/
	switch(hmodbus->init.parity)
	{
	case 0:	//none
		huart->Init.WordLength = UART_WORDLENGTH_8B;
		huart->Init.Parity = UART_PARITY_NONE;
		break;
	case 1:	// even
		huart->Init.WordLength = UART_WORDLENGTH_9B;
		huart->Init.Parity = UART_PARITY_EVEN;
		break;
	case 2:	//odd
		huart->Init.WordLength = UART_WORDLENGTH_9B;
		huart->Init.Parity = UART_PARITY_ODD;
		break;
	} //default is even parity

	HAL_UART_Abort_IT(huart);	//TODO do we need _IT function?
	HAL_UART_Init(huart);
	HAL_UART_Receive_DMA(hmodbus->huart, buf_modbus, 0x100);
}

