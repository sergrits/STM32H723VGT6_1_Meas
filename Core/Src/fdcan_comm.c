/* fdcan_comm.c - simple FDCAN receiver for frequency packets (ID 0x123) */
#include "fdcan_comm.h"
#include "main.h"
#include "stm32h7xx_hal_fdcan.h"

/* Console_Log определена в main.c */
extern void Console_Log(const char *fmt, ...);

/* Minimal ITM helpers for quick logging (matches pattern used in H562 project) */
#if 0
static void itm_write_char(char ch)
{
	if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U) return;
	if (((ITM->TCR & ITM_TCR_ITMENA_Msk) == 0U) || ((ITM->TER & 1U) == 0U)) return;
	(void)ITM_SendChar((uint32_t)ch);
}

static void itm_write_str(const char *s)
{
	while (*s != '\0') itm_write_char(*s++);
}

static void itm_write_line(const char *s)
{
	itm_write_str(s);
	itm_write_str("\r\n");
}

/* Small integer -> string helper for ITM logging */
static void u32_to_str(char *buf, uint32_t v)
{
	char tmp[12];
	int p = 0;
	if (v == 0) tmp[p++] = '0';
	while (v != 0 && p < (int)sizeof(tmp)) { tmp[p++] = (char)('0' + (v % 10)); v /= 10; }
	int idx = 0;
	for (int i = p - 1; i >= 0; --i) buf[idx++] = tmp[i];
	buf[idx] = '\0';
}
#endif

#define ITM_LOG(s)
#define ITM_LOG_STR(s)
#define ITM_LOG_LINE(s)

extern FDCAN_HandleTypeDef hfdcan1;

#define FDCAN_FREQ_STD_ID 0x123U
#define FDCAN_VERIFY_REQ_ID 0x124U
#define FDCAN_VERIFY_STATUS_ID 0x125U
#define FDCAN_VERIFY_RES_ID 0x126U
#define FDCAN_MODE_STATUS_ID 0x127U

static volatile uint32_t s_last_freq_hz = 0U;
static volatile uint8_t s_last_mode = 0U;
static volatile uint32_t s_rx_count = 0U;
static volatile uint32_t s_last_rx_tick = 0U;
static volatile uint32_t s_tx_ping_ok = 0U;
static volatile uint32_t s_tx_ping_fail = 0U;
static volatile verify_status_t s_verify_status = VERIFY_IDLE;

#if HARD_HW_TEST
static void fdcan_send_ping(void)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[2] = {0xA5U, (uint8_t)(s_tx_ping_ok & 0xFFU)};

	txHeader.Identifier = 0x321U;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_2;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0U;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		s_tx_ping_ok++;
		ITM_LOG_LINE("TXPING+");
	} else {
		s_tx_ping_fail++;
		ITM_LOG_LINE("TXPING-");
	}
}  /* закрываем fdcan_send_ping() */
#endif

int FDCAN_SendVerifyRequest(uint32_t freq_hz)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[4];
	txData[0] = (uint8_t)(freq_hz & 0xFFU);
	txData[1] = (uint8_t)((freq_hz >> 8) & 0xFFU);
	txData[2] = (uint8_t)((freq_hz >> 16) & 0xFFU);
	txData[3] = (uint8_t)((freq_hz >> 24) & 0xFFU);

	txHeader.Identifier = FDCAN_VERIFY_REQ_ID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_4;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		return 0;
	}
	return -1;
}

int FDCAN_SendVerifyResult(uint8_t confirmed)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[1] = {confirmed};

	txHeader.Identifier = FDCAN_VERIFY_RES_ID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_1;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		return 0;
	}
	return -1;
}

/**
 * @brief Отправить команду удаленного перезапуска H5 через CAN
 * @details Используется при старте H7 для синхронизации или при детектировании зависания H5
 * @return 0 если успешно отправлено в TX FIFO, -1 если FIFO переполнена
 */
int FDCAN_SendRemoteReboot(void)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[1] = {0xAAU};  /* Magic byte для подтверждения команды */

	txHeader.Identifier = FDCAN_REMOTE_REBOOT_ID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_1;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		return 0;
	}
	return -1;
}

/**
 * @brief Отправить команду паузы (остановить PWM на H5)
 * @details Используется перед критичной SD операцией для предотвращения помех
 * @return 0 если успешно, -1 если ошибка
 */
int FDCAN_SendPauseCommand(void)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[1] = {0x50U};  /* Magic byte 'P' для PAUSE */

	txHeader.Identifier = FDCAN_PAUSE_CMD_ID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_1;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		return 0;
	}
	return -1;
}

/**
 * @brief Отправить команду возобновления (запустить PWM на H5)
 * @details Используется после завершения критичной SD операции
 * @return 0 если успешно, -1 если ошибка
 */
int FDCAN_SendResumeCommand(void)
{
	FDCAN_TxHeaderTypeDef txHeader = {0};
	uint8_t txData[1] = {0x52U};  /* Magic byte 'R' для RESUME */

	txHeader.Identifier = FDCAN_RESUME_CMD_ID;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_1;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txData) == HAL_OK) {
		return 0;
	}
	return -1;
}

verify_status_t FDCAN_GetVerifyStatus(void)
{
	return s_verify_status;
}

static uint32_t fdcan_dlc_to_len(uint32_t dlc)
{
	switch (dlc) {
	case FDCAN_DLC_BYTES_0:  return 0U;
	case FDCAN_DLC_BYTES_1:  return 1U;
	case FDCAN_DLC_BYTES_2:  return 2U;
	case FDCAN_DLC_BYTES_3:  return 3U;
	case FDCAN_DLC_BYTES_4:  return 4U;
	case FDCAN_DLC_BYTES_5:  return 5U;
	case FDCAN_DLC_BYTES_6:  return 6U;
	case FDCAN_DLC_BYTES_7:  return 7U;
	case FDCAN_DLC_BYTES_8:  return 8U;
	case FDCAN_DLC_BYTES_12: return 12U;
	case FDCAN_DLC_BYTES_16: return 16U;
	case FDCAN_DLC_BYTES_20: return 20U;
	case FDCAN_DLC_BYTES_24: return 24U;
	case FDCAN_DLC_BYTES_32: return 32U;
	case FDCAN_DLC_BYTES_48: return 48U;
	case FDCAN_DLC_BYTES_64: return 64U;
	default:                 return 0U;
	}
}

static void fdcan_process_fifo0(void)
{
	while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
		FDCAN_RxHeaderTypeDef rxHeader = { 0 };
		uint8_t rxData[8] = { 0 };

		if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
			break;
		}

		s_rx_count++;
		s_last_rx_tick = HAL_GetTick();

		if (rxHeader.IdType == FDCAN_STANDARD_ID) {
			if (rxHeader.Identifier == FDCAN_FREQ_STD_ID) {
				if (fdcan_dlc_to_len(rxHeader.DataLength) >= 4U) {
					s_last_freq_hz = ((uint32_t)rxData[0])
							| ((uint32_t)rxData[1] << 8)
							| ((uint32_t)rxData[2] << 16)
							| ((uint32_t)rxData[3] << 24);
				}
			} else if (rxHeader.Identifier == FDCAN_VERIFY_STATUS_ID) {
				if (fdcan_dlc_to_len(rxHeader.DataLength) >= 1U) {
					s_verify_status = (verify_status_t)rxData[0];
				}
			} else if (rxHeader.Identifier == FDCAN_MODE_STATUS_ID) {
				if (fdcan_dlc_to_len(rxHeader.DataLength) >= 1U) {
					s_last_mode = rxData[0];
				}
			}
		}
	}
}

void FDCAN_Comm_Init(void)
{
	if (hfdcan1.Instance == NULL) {
		return;
	}

	FDCAN_FilterTypeDef filter = { 0 };
	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0U;
	filter.FilterType = FDCAN_FILTER_MASK;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	/* Highly permissive filter: Accept all standard IDs to ensure connectivity */
	filter.FilterID1 = 0x000U;
	filter.FilterID2 = 0x000U;
	(void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

	(void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
	                                 FDCAN_ACCEPT_IN_RX_FIFO0,
	                                 FDCAN_REJECT,
	                                 FDCAN_FILTER_REMOTE,
	                                 FDCAN_FILTER_REMOTE);

	HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

	(void)HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
									 FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
									 FDCAN_IT_BUS_OFF |
									 FDCAN_IT_ARB_PROTOCOL_ERROR,
									 FDCAN_INTERRUPT_LINE0);

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Console_Log("[H7] FDCAN START FAIL!\n");
		return;
	}
	Console_Log("[H7] FDCAN START OK. Waiting for data...\n");

	(void)HAL_FDCAN_ActivateNotification(&hfdcan1,
									FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
									FDCAN_IT_BUS_OFF |
									FDCAN_IT_ARB_PROTOCOL_ERROR,
									0U);

	// ... (ваш код ініціалізації)
	fdcan_process_fifo0();

	/* Одразу перевіримо стан після старту */
#if 0
	    {
	        uint32_t fill = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
	        char tb[64];
	        itm_write_str("Initial RXF0=");
	        u32_to_str(tb, fill);
	        itm_write_line(tb);
	    }
#endif

}

void FDCAN_PollRx(void)
{
    if (hfdcan1.Instance == NULL) {
        return;
    }

/* Резервний варіант опитування: очищення всіх очікуваних RX-кадрів. Мигання при кожному отриманні пакета CAN*/
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
		fdcan_process_fifo0();
		// HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // <-- закомментировано
	}
}

uint32_t FDCAN_GetLastFreq(void)
{
	return s_last_freq_hz;
}

uint8_t FDCAN_GetLastMode(void)
{
	return s_last_mode;
}

uint32_t FDCAN_GetRxCount(void)
{
	return s_rx_count;
}

uint32_t FDCAN_GetLastRxTick(void)
{
	return s_last_rx_tick;
}

/**
 * @brief Получить диагностику CAN ошибок (Error Counters)
 * @details Читает счетчики ошибок из CAN контроллера и возвращает в структуре
 * @return Структура fdcan_diag_t с текущими счетчиками ошибок
 */
fdcan_diag_t FDCAN_GetDiagnostics(void)
{
	fdcan_diag_t diag = {0};
	FDCAN_ErrorCountersTypeDef ec = {0};
	
	/* Получить счетчики ошибок из CAN контроллера */
	if (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec) == HAL_OK) {
		diag.tx_error_count = ec.TxErrorCnt;   /**< Ошибки передачи (0-255) */
		diag.rx_error_count = ec.RxErrorCnt;   /**< Ошибки приема (0-255) */
	}
	
	diag.last_check_ms = HAL_GetTick();
	
	return diag;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	(void)RxFifo0ITs;
	if (hfdcan == &hfdcan1) {
		fdcan_process_fifo0();
	}
}

/**
 * @brief Bus-Off и protocol error recovery для H7
 * @details При Bus-Off автоматически перезапускает CAN peripheral
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan != &hfdcan1) return;

	if (ErrorStatusITs & FDCAN_IT_BUS_OFF) {
		(void)HAL_FDCAN_Stop(&hfdcan1);
		(void)HAL_FDCAN_Start(&hfdcan1);
		(void)HAL_FDCAN_ActivateNotification(&hfdcan1,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
			FDCAN_IT_BUS_OFF |
			FDCAN_IT_ARB_PROTOCOL_ERROR, 0U);
	}
}
