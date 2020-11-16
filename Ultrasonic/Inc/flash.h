#include "stm32f0xx_hal.h"
#include "stdint.h"
#include "string.h"

void deleteBuffer(uint8_t* data);
void 	Flash_Lock(void);
void 	Flash_Unlock(void);
void 	Flash_Erase(uint32_t addr);
void 	Flash_Write_Int(uint32_t addr, uint16_t data);
uint16_t Flash_Read_Int(uint32_t addr);
void 	Flash_Write_Char(uint32_t addr, uint8_t* data);
void 	Flash_ReadChar(uint8_t* dataOut, uint32_t addr1, uint32_t addr2);
void 	Flash_ProgramPage(uint8_t* dataIn, uint32_t addr1, uint32_t addr2);
