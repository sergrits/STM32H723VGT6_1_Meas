/* w25qxx_ospi.c — wrapper: maps w25qxx_qspi API to OCTOSPI1 (HAL_OSPI) */
#include "w25qxx_ospi.h"  /* уже включает octospi.h → hospi1 */

/* Forward declarations */
static int32_t OSPI_SendCmd_NoData(uint32_t instruction);
static int32_t OSPI_ReadReg(uint32_t instruction, uint8_t *data, uint32_t size);

uint16_t w25qxx_ID = 0;
uint8_t  w25qxx_StatusReg[3] = {0};

/* ---------- внутренние функции ---------- */

static int32_t OSPI_SendCmd_NoData(uint32_t instruction)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction     = instruction;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode        = HAL_OSPI_DATA_NONE;
    cmd.DummyCycles     = 0;
    cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
    return HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

static int32_t OSPI_ReadReg(uint32_t instruction, uint8_t *data, uint32_t size)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction     = instruction;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_NONE;
    cmd.DataMode        = HAL_OSPI_DATA_1_LINE;
    cmd.NbData          = size;
    cmd.DummyCycles     = 0;
    cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return w25qxx_ERROR;
    return HAL_OSPI_Receive(&hospi1, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
}

/* ---------- публичный API (w25qxx_qspi совместимый) ---------- */

void w25qxx_Init(void)
{
    HAL_Delay(5);

    /* --- Сброс устройства: Enable Reset + Reset Device --- */
    OSPI_SendCmd_NoData(W25X_EnableReset);
    HAL_Delay(1);
    OSPI_SendCmd_NoData(W25X_ResetDevice);
    HAL_Delay(1);

    /* --- Установить QE бит в SR2 если не установлен --- */
    uint8_t sr2 = 0;
    OSPI_ReadReg(W25X_ReadStatusReg2, &sr2, 1);
    if ((sr2 & 0x02U) == 0U) {
        W25qxx_WriteEnable();
        sr2 |= 0x02U;
        OSPI_RegularCmdTypeDef cmd = {0};
        cmd.OperationType    = HAL_OSPI_OPTYPE_COMMON_CFG;
        cmd.FlashId          = HAL_OSPI_FLASH_ID_1;
        cmd.Instruction      = W25X_WriteStatusReg2;
        cmd.InstructionMode  = HAL_OSPI_INSTRUCTION_1_LINE;
        cmd.InstructionSize  = HAL_OSPI_INSTRUCTION_8_BITS;
        cmd.AddressMode      = HAL_OSPI_ADDRESS_NONE;
        cmd.DataMode         = HAL_OSPI_DATA_1_LINE;
        cmd.NbData           = 1;
        cmd.DummyCycles      = 0;
        cmd.SIOOMode         = HAL_OSPI_SIOO_INST_EVERY_CMD;
        HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
        HAL_OSPI_Transmit(&hospi1, &sr2, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
        HAL_Delay(15);  /* NV SR write занимает до 15 мс */
    }

    /* --- Читаем JEDEC ID (3 байта: manufacturer, type, capacity) --- */
    {
        uint8_t id[3] = {0};
        OSPI_RegularCmdTypeDef cmd = {0};
        cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
        cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
        cmd.Instruction     = W25X_JedecDeviceID;   /* 0x9F */
        cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
        cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
        cmd.AddressMode     = HAL_OSPI_ADDRESS_NONE;
        cmd.DataMode        = HAL_OSPI_DATA_1_LINE;
        cmd.NbData          = 3;
        cmd.DummyCycles     = 0;
        cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
        if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK) {
            HAL_OSPI_Receive(&hospi1, id, HAL_OSPI_TIMEOUT_DEFAULT_VALUE);
        }
        /* id[0]=Manufacturer(EF), id[1]=MemType(40), id[2]=Capacity(17=64Mbit) */
        w25qxx_ID = ((uint16_t)id[1] << 8) | id[2];
    }

    /* --- Читаем все статусные регистры --- */
    OSPI_ReadReg(W25X_ReadStatusReg1, &w25qxx_StatusReg[0], 1);
    OSPI_ReadReg(W25X_ReadStatusReg2, &w25qxx_StatusReg[1], 1);
    OSPI_ReadReg(W25X_ReadStatusReg3, &w25qxx_StatusReg[2], 1);
}


uint8_t W25qxx_WriteEnable(void)
{
    return (OSPI_SendCmd_NoData(W25X_WriteEnable) == HAL_OK) ? w25qxx_OK : w25qxx_ERROR;
}

uint8_t W25qxx_IsBusy(void)
{
    uint8_t sr1 = 0;
    if (OSPI_ReadReg(W25X_ReadStatusReg1, &sr1, 1) != HAL_OK) return w25qxx_BUSY;
    return (sr1 & W25X_SR_WIP) ? w25qxx_BUSY : w25qxx_OK;
}

uint8_t W25qxx_SectorEraseStart(uint32_t SectorAddress)
{
    if (W25qxx_WriteEnable() != w25qxx_OK) return w25qxx_ERROR;

    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction     = W25X_SectorErase;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Address         = SectorAddress;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = HAL_OSPI_ADDRESS_24_BITS;
    cmd.DataMode        = HAL_OSPI_DATA_NONE;
    cmd.DummyCycles     = 0;
    cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
    return (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
           ? w25qxx_OK : w25qxx_ERROR;
}

uint8_t W25qxx_PageProgramStart(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
    if (W25qxx_WriteEnable() != w25qxx_OK) return w25qxx_ERROR;

    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction     = W25X_PageProgram;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Address         = WriteAddr;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = HAL_OSPI_ADDRESS_24_BITS;
    cmd.DataMode        = HAL_OSPI_DATA_1_LINE;
    cmd.NbData          = Size;
    cmd.DummyCycles     = 0;
    cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return w25qxx_ERROR;
    return (HAL_OSPI_Transmit(&hospi1, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
           ? w25qxx_OK : w25qxx_ERROR;
}

uint8_t W25qxx_PageProgram(uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
    uint8_t st = W25qxx_PageProgramStart(pData, WriteAddr, Size);
    if (st != w25qxx_OK) return st;
    uint32_t t0 = HAL_GetTick();
    while (W25qxx_IsBusy() == w25qxx_BUSY) {
        if ((HAL_GetTick() - t0) > 100U) return w25qxx_TIMEOUT;
    }
    return w25qxx_OK;
}

uint8_t W25qxx_EraseSector(uint32_t SectorAddress)
{
    uint8_t st = W25qxx_SectorEraseStart(SectorAddress);
    if (st != w25qxx_OK) return st;
    uint32_t t0 = HAL_GetTick();
    while (W25qxx_IsBusy() == w25qxx_BUSY) {
        if ((HAL_GetTick() - t0) > 2000U) return w25qxx_TIMEOUT;
    }
    return w25qxx_OK;
}

uint8_t W25qxx_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType   = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId         = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction     = W25X_FastReadData;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.Address         = ReadAddr;
    cmd.AddressMode     = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize     = HAL_OSPI_ADDRESS_24_BITS;
    cmd.DataMode        = HAL_OSPI_DATA_1_LINE;
    cmd.NbData          = Size;
    cmd.DummyCycles     = 8;
    cmd.SIOOMode        = HAL_OSPI_SIOO_INST_EVERY_CMD;
    if (HAL_OSPI_Command(&hospi1, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) return w25qxx_ERROR;
    return (HAL_OSPI_Receive(&hospi1, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK)
           ? w25qxx_OK : w25qxx_ERROR;
}
