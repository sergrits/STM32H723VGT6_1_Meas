#include "sd_logger.h"
#include "fatfs.h"
#include "fdcan_comm.h"
#include "bsp_driver_sd.h"
#include "w25qxx_ospi.h"
#include <stdio.h>
#include <string.h>

/* ITM вывод для тестовой функции */
static inline void sd_test_itm(const char *s) {
    while (*s) {
        while (ITM->PORT[0U].u32 == 0U) { __NOP(); }
        ITM->PORT[0U].u8 = (uint8_t)(*s++);
    }
}

#define SD_WRITE_CHUNK_BYTES    512U  /* Размер SD-сектора — оптимальная атомарная запись */
#define SD_SYNC_AFTER_LINE_MS   100U  /* f_sync после каждой верифицированной записи */
#define SD_RETRY_MIN_MS        2000U
#define SD_RETRY_MAX_MS       60000U

/* QSPI Flash logging: адреса для логирования верифицированных данных */
#define QSPI_VERIFIED_LOG_START  0x100000UL  /* 1MB offset (страница 0x400) */
#define QSPI_VERIFIED_LOG_SIZE   0x3F0000UL  /* ~4MB для логирования */
static uint32_t s_qspi_write_addr = QSPI_VERIFIED_LOG_START;

static FATFS s_fs;
static FIL s_file;
static char s_filename[32];  /* "data_XXXXXXXX.csv" (по HAL_GetTick) */
static uint8_t s_is_mounted = 0;
static uint8_t s_file_open = 0;
static uint8_t s_start_requested = 0;

static uint32_t s_next_try_ms = 0;
static uint32_t s_retry_ms = SD_RETRY_MIN_MS;
static uint32_t s_last_sync_ms = 0;

static void SD_Logger_ScheduleRetry(uint32_t now) {
    s_next_try_ms = now + s_retry_ms;
    if (s_retry_ms < SD_RETRY_MAX_MS) {
        s_retry_ms <<= 1;
        if (s_retry_ms > SD_RETRY_MAX_MS) {
            s_retry_ms = SD_RETRY_MAX_MS;
        }
    }
}

static uint8_t SD_Logger_TryOpen(uint32_t now) {
    if (s_file_open) {
        return 1U;
    }

    if (BSP_SD_GetCardState() != MSD_OK) {
        SD_Logger_ScheduleRetry(now);
        return 0U;
    }

    FRESULT res = f_mount(&s_fs, SDPath, 1);
    if (res != FR_OK) {
        s_is_mounted = 0;
        SD_Logger_ScheduleRetry(now);
        return 0U;
    }

    s_is_mounted = 1;
    
    /* Создаём уникальное имя файла на основе HAL_GetTick (без RTC) */
    uint32_t session_tick = HAL_GetTick();
    snprintf(s_filename, sizeof(s_filename), "data_%08lx.csv", (unsigned long)session_tick);
    
    res = f_open(&s_file, s_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        (void)f_mount(NULL, SDPath, 0);
        s_is_mounted = 0;
        SD_Logger_ScheduleRetry(now);
        return 0U;
    }

    s_file_open = 1;
    s_retry_ms = SD_RETRY_MIN_MS;
    s_next_try_ms = now + SD_RETRY_MIN_MS;
    s_last_sync_ms = now;

    /* Записываем заголовок CSV */
    (void)f_printf(&s_file, "Tick_ms;Freq_Hz;Weight_ADC\n");
    (void)f_sync(&s_file);
    
    /* Логируем начало сессии в QSPI Flash */
    char qspi_line[64];
    int qspi_len = snprintf(qspi_line, sizeof(qspi_line), "[SESSION_START] tick=%lu file=%s\n", 
                            (unsigned long)session_tick, s_filename);
    if (qspi_len > 0 && s_qspi_write_addr + qspi_len < QSPI_VERIFIED_LOG_START + QSPI_VERIFIED_LOG_SIZE) {
        W25qxx_PageProgram((uint8_t*)qspi_line, s_qspi_write_addr, qspi_len);
        s_qspi_write_addr += qspi_len;
    }

    return 1U;
}

uint8_t SD_Logger_Init(void) {
    s_start_requested = 1U;
    return s_file_open;
}

int SD_Logger_PauseGenerator(void) {
    /* Отправить CAN PAUSE команду на H5 */
    return FDCAN_SendPauseCommand();
}

int SD_Logger_ResumeGenerator(void) {
    /* Отправить CAN RESUME команду на H5 */
    return FDCAN_SendResumeCommand();
}

void SD_Logger_LogLine(uint32_t freq, uint32_t weight) {
    uint32_t now = HAL_GetTick();
    char line[64];
    int len = snprintf(line, sizeof(line), "%lu;%lu;%lu\n", 
                      (unsigned long)now, (unsigned long)freq, (unsigned long)weight);
    
    if (len <= 0 || len >= (int)sizeof(line)) {
        return;
    }

    /* Если SD готов — пишем на SD */
    if (s_file_open) {
        UINT bw = 0;
        if (f_write(&s_file, line, (UINT)len, &bw) == FR_OK && bw == (UINT)len) {
            /* f_sync сразу после верифицированной записи */
            (void)f_sync(&s_file);
        } else {
            s_file_open = 0;
            if (s_is_mounted) {
                (void)f_mount(NULL, SDPath, 0);
                s_is_mounted = 0;
            }
            SD_Logger_ScheduleRetry(now);
        }
    }
    
    /* Дублируем в QSPI Flash */
    if (s_qspi_write_addr + len < QSPI_VERIFIED_LOG_START + QSPI_VERIFIED_LOG_SIZE) {
        W25qxx_PageProgram((uint8_t*)line, s_qspi_write_addr, len);
        s_qspi_write_addr += len;
    }
}

void SD_Logger_Process(uint32_t now) {
    if (!s_file_open) {
        if (!s_start_requested) {
            return;
        }
        if ((int32_t)(now - s_next_try_ms) >= 0) {
            (void)SD_Logger_TryOpen(now);
        }
        return;
    }
}

sd_logger_state_t SD_Logger_GetState(void) {
    if (s_file_open) {
        return SD_LOGGER_STATE_READY;
    }
    if (s_start_requested) {
        return SD_LOGGER_STATE_WAIT_RETRY;
    }
    return SD_LOGGER_STATE_IDLE;
}

/* ============================================================
 *  SD_Test_Run — тестовая функция записи/чтения SD карты
 * ============================================================ */
#ifdef SD_TEST_ENABLED

#define SD_TEST_FILE       "sd_test.csv"
#define SD_TEST_ROWS       20U
#define SD_TEST_MAGIC      0xABCDU   /* Контрольное слово */

uint8_t SD_Test_Run(void)
{
    FATFS  test_fs;
    FIL    test_file;
    FRESULT res;
    char   buf[80];
    char   itm_msg[128];

    sd_test_itm("\r\n====== SD CARD TEST START ======\r\n");

    /* --- Шаг 1: Проверка физического состояния карты --- */
    snprintf(itm_msg, sizeof(itm_msg), "[SD TEST] BSP state: %s\r\n",
             (BSP_SD_GetCardState() == MSD_OK) ? "OK" : "NOT READY");
    sd_test_itm(itm_msg);

    if (BSP_SD_GetCardState() != MSD_OK) {
        sd_test_itm("[SD TEST] FAIL: Card not present or not responding\r\n");
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
        return 0U;
    }

    /* --- Шаг 2: Монтирование файловой системы --- */
    res = f_mount(&test_fs, SDPath, 1);
    snprintf(itm_msg, sizeof(itm_msg), "[SD TEST] f_mount: %s (code=%d)\r\n",
             (res == FR_OK) ? "OK" : "FAIL", (int)res);
    sd_test_itm(itm_msg);

    if (res != FR_OK) {
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
        return 0U;
    }

    /* --- Шаг 3: Получение информации о карте --- */
    FATFS *pfs = &test_fs;
    DWORD  fre_clust = 0;
    if (f_getfree(SDPath, &fre_clust, &pfs) == FR_OK) {
        uint32_t total_kb = (uint32_t)(((pfs->n_fatent - 2U) * pfs->csize) / 2U);
        uint32_t free_kb  = (uint32_t)((fre_clust * pfs->csize) / 2U);
        snprintf(itm_msg, sizeof(itm_msg),
                 "[SD TEST] Card: Total=%lu KB, Free=%lu KB\r\n",
                 (unsigned long)total_kb, (unsigned long)free_kb);
        sd_test_itm(itm_msg);
    }

    /* --- Шаг 4: Запись тестового CSV файла --- */
    res = f_open(&test_file, SD_TEST_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    snprintf(itm_msg, sizeof(itm_msg), "[SD TEST] f_open (write): %s\r\n",
             (res == FR_OK) ? "OK" : "FAIL");
    sd_test_itm(itm_msg);

    if (res != FR_OK) {
        (void)f_mount(NULL, SDPath, 0);
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
        return 0U;
    }

    /* Заголовок CSV */
    (void)f_printf(&test_file, "Row;Tick_ms;Freq_Hz;Weight_ADC;Checksum\n");

    /* Тестовые данные + checksum каждой строки */
    uint32_t write_checksum = 0U;
    uint8_t  write_ok = 1U;
    for (uint32_t i = 0U; i < SD_TEST_ROWS; i++) {
        uint32_t freq   = 100000UL + i * 5000UL;   /* 100000..195000 Гц */
        uint32_t weight = 1000UL  + i * 50UL;       /* 1000..1950 */
        uint32_t tick   = HAL_GetTick() + i * 10UL;
        uint32_t crc    = (i ^ freq ^ weight ^ SD_TEST_MAGIC) & 0xFFFFU;

        write_checksum += crc;

        int len = snprintf(buf, sizeof(buf), "%lu;%lu;%lu;%lu;%04lX\n",
                           (unsigned long)i,
                           (unsigned long)tick,
                           (unsigned long)freq,
                           (unsigned long)weight,
                           (unsigned long)crc);
        UINT bw = 0;
        if (f_write(&test_file, buf, (UINT)len, &bw) != FR_OK || bw != (UINT)len) {
            write_ok = 0U;
            break;
        }
    }

    (void)f_sync(&test_file);
    (void)f_close(&test_file);

    snprintf(itm_msg, sizeof(itm_msg),
             "[SD TEST] Write %lu rows: %s (checksum=0x%04lX)\r\n",
             (unsigned long)SD_TEST_ROWS,
             write_ok ? "OK" : "FAIL",
             (unsigned long)write_checksum);
    sd_test_itm(itm_msg);

    if (!write_ok) {
        (void)f_mount(NULL, SDPath, 0);
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
        return 0U;
    }

    /* --- Шаг 5: Чтение обратно и проверка checksum --- */
    res = f_open(&test_file, SD_TEST_FILE, FA_READ);
    snprintf(itm_msg, sizeof(itm_msg), "[SD TEST] f_open (read):  %s\r\n",
             (res == FR_OK) ? "OK" : "FAIL");
    sd_test_itm(itm_msg);

    if (res != FR_OK) {
        (void)f_mount(NULL, SDPath, 0);
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
        return 0U;
    }

    uint32_t read_checksum  = 0U;
    uint32_t rows_read      = 0U;
    uint8_t  first_line     = 1U;   /* Пропускаем заголовок */

    while (f_gets(buf, (int)sizeof(buf), &test_file) != NULL) {
        if (first_line) { first_line = 0U; continue; }  /* Пропустить заголовок */

        unsigned long row, tick, freq, weight;
        unsigned long crc_read;
        if (sscanf(buf, "%lu;%lu;%lu;%lu;%lX",
                   &row, &tick, &freq, &weight, &crc_read) == 5) {
            uint32_t crc_calc = (row ^ freq ^ weight ^ SD_TEST_MAGIC) & 0xFFFFU;
            if ((uint32_t)crc_read != crc_calc) {
                snprintf(itm_msg, sizeof(itm_msg),
                         "[SD TEST] CRC ERROR row %lu: got %04lX, expected %04lX\r\n",
                         row, crc_read, (unsigned long)crc_calc);
                sd_test_itm(itm_msg);
            }
            read_checksum += (uint32_t)crc_read;
            rows_read++;
        }
    }
    (void)f_close(&test_file);

    snprintf(itm_msg, sizeof(itm_msg),
             "[SD TEST] Read  %lu rows: checksum=0x%04lX\r\n",
             (unsigned long)rows_read,
             (unsigned long)read_checksum);
    sd_test_itm(itm_msg);

    /* --- Шаг 6: Итог --- */
    uint8_t pass = (rows_read == SD_TEST_ROWS) &&
                   (read_checksum == write_checksum);

    snprintf(itm_msg, sizeof(itm_msg),
             "[SD TEST] Rows match: %s | Checksum match: %s\r\n",
             (rows_read == SD_TEST_ROWS) ? "YES" : "NO",
             (read_checksum == write_checksum) ? "YES" : "NO");
    sd_test_itm(itm_msg);

    if (pass) {
        sd_test_itm("====== SD CARD TEST PASSED ======\r\n\r\n");
    } else {
        sd_test_itm("====== SD CARD TEST FAILED ======\r\n\r\n");
    }

    (void)f_mount(NULL, SDPath, 0);
    return pass ? 1U : 0U;
}

#endif /* SD_TEST_ENABLED */