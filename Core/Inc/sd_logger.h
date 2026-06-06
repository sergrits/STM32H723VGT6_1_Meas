#ifndef INC_SD_LOGGER_H_
#define INC_SD_LOGGER_H_

#include <stdint.h>

typedef enum {
    SD_LOGGER_STATE_IDLE = 0,
    SD_LOGGER_STATE_WAIT_RETRY,
    SD_LOGGER_STATE_READY
} sd_logger_state_t;

/**
 * @brief Инициализация SD логгера (создаёт новую сессию с уникальным файлом)
 * @return 1 если файл уже открыт, 0 если инициализация требуется
 */
uint8_t SD_Logger_Init(void);

/**
 * @brief Периодическая обработка SD логгера (retry logic, f_sync)
 * @param now Текущий системный таймстемп (HAL_GetTick)
 */
void SD_Logger_Process(uint32_t now);

/**
 * @brief Запись верифицированной строки на SD и в QSPI Flash
 * @param freq Частота (Гц)
 * @param weight Вес (ADC отсчеты)
 * @details Создаёт CSV строку и дублирует в QSPI при необходимости
 */
void SD_Logger_LogLine(uint32_t freq, uint32_t weight);

/**
 * @brief Получить текущее состояние логгера
 * @return Состояние (IDLE, WAIT_RETRY, READY)
 */
sd_logger_state_t SD_Logger_GetState(void);

/**
 * @brief Остановить PWM перед записью на SD (отправить CAN PAUSE команду)
 * @return 0 если успешно, -1 если ошибка
 */
int SD_Logger_PauseGenerator(void);

/**
 * @brief Возобновить PWM после записи на SD (отправить CAN RESUME команду)
 * @return 0 если успешно, -1 если ошибка
 */
int SD_Logger_ResumeGenerator(void);

/* ============================================================
 *  SD CARD TEST — ручная проверка работоспособности карты
 * ============================================================
 */

//#define SD_TEST_ENABLED

#ifdef SD_TEST_ENABLED
uint8_t SD_Test_Run(void);
#endif /* SD_TEST_ENABLED */

#endif /* INC_SD_LOGGER_H_ */
