#ifndef __w25qxx_H
#define __w25qxx_H

#ifdef __cplusplus
extern "C" {
#endif

#include "octospi.h"

/* =============== W25Qxx CMD ================ */
#define W25X_WriteEnable         0x06
#define W25X_WriteDisable        0x04
#define W25X_ReadStatusReg1      0x05
#define W25X_ReadStatusReg2      0x35
#define W25X_ReadStatusReg3      0x15
#define W25X_WriteStatusReg1     0x01
#define W25X_WriteStatusReg2     0x31
#define W25X_WriteStatusReg3     0x11
#define W25X_ReadData            0x03
#define W25X_FastReadData        0x0B
#define W25X_FastReadDual        0x3B
#define W25X_PageProgram         0x02
#define W25X_BlockErase          0xD8
#define W25X_SectorErase         0x20
#define W25X_ChipErase           0xC7
#define W25X_PowerDown           0xB9
#define W25X_ReleasePowerDown    0xAB
#define W25X_DeviceID            0xAB
#define W25X_ManufactDeviceID    0x90
#define W25X_JedecDeviceID       0x9F
#define W25X_Enable4ByteAddr     0xB7
#define W25X_Exit4ByteAddr       0xE9
#define W25X_SetReadParam        0xC0
#define W25X_EnterQSPIMode       0x38
#define W25X_ExitQSPIMode        0xFF
#define W25X_EnableReset         0x66
#define W25X_ResetDevice         0x99

#define W25X_QUAD_FAST_READ_DTR_CMD               0x0D
#define W25X_QUAD_INOUT_FAST_READ_CMD             0xEB
#define W25X_QUAD_INOUT_FAST_READ_DTR_CMD         0xED
#define W25X_QUAD_INOUT_FAST_READ_4_BYTE_ADDR_CMD 0xEC
#define W25X_QUAD_INOUT_FAST_READ_4_BYTE_DTR_CMD  0xEE
#define W25X_QUAD_ManufactDeviceID                0x94
#define W25X_QUAD_INPUT_PAGE_PROG_CMD             0x32

#define W25X_ENTER_4_BYTE_ADDR_MODE_CMD           0xB7
#define W25X_EXIT_4_BYTE_ADDR_MODE_CMD            0xE9

#define W25X_SR_WIP              (0x01)
#define W25X_SR_WREN             (0x02)

#define W25X_DUMMY_CYCLES_READ_QUAD_DTR           4U
#define W25X_DUMMY_CYCLES_READ_QUAD               6U
#define W25X_DUMMY_CYCLES_QUAD_INOUT_FAST_READ    4U
#define W25X_DUMMY_CYCLES_FAST_READ_QPI_MDOE      8U

/* ============================================================
 *  w25qxx_qspi-compatible API (используется main.c, sd_logger.c)
 * ============================================================ */

typedef enum {
    w25qxx_OK      = 0,
    w25qxx_ERROR   = 1,
    w25qxx_BUSY    = 2,
    w25qxx_TIMEOUT = 3
} w25qxx_StatusTypeDef;

extern uint16_t w25qxx_ID;
extern uint8_t  w25qxx_StatusReg[3];

void    w25qxx_Init(void);
uint8_t W25qxx_WriteEnable(void);
uint8_t W25qxx_IsBusy(void);
uint8_t W25qxx_SectorEraseStart(uint32_t SectorAddress);
uint8_t W25qxx_PageProgramStart(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
uint8_t W25qxx_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
uint8_t W25qxx_EraseSector(uint32_t SectorAddress);
uint8_t W25qxx_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size);



#ifdef __cplusplus
}
#endif

#endif /* __w25qxx_H */
