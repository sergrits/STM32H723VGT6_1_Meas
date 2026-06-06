/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "w25qxx_ospi.h"
#include "fdcan_comm.h"
#include "sd_logger.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Адаптивный IIR-фильтр (H723 ADC 16-bit: 0-65535) */
#define ADAPT_FAST_TH  8000U   /* ~500 * 16 для 16-bit ADC */
#define ADAPT_MED_TH   1920U   /* ~120 * 16 */
#define ADC_BUF_LEN    256

/* Flash layout (8MB W25Q64) */
#define LOG_PAGE_SIZE          256UL
#define LOG_SECTOR_SIZE       4096UL
#define LOG_BASE_ADDR       0x000000UL
#define LOG_TOTAL_BYTES   (8UL * 1024UL * 1024UL)
#define LOG_END_ADDR      (LOG_BASE_ADDR + LOG_TOTAL_BYTES)
#define VERIFIED_BASE_ADDR    (6UL * 1024UL * 1024UL)
#define VERIFIED_END_ADDR     (8UL * 1024UL * 1024UL)

/* UI координаты */
#define UI_Y0  0
#define UI_Y1  20
#define UI_Y2  38
#define UI_Y3  60

/* Misc */
#define LOG_IO_POLL_MS        2UL
#define ADC_PROCESS_BUDGET   32U
#define UI_STARTUP_SETTLE_MS 300UL

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan1;

OSPI_HandleTypeDef hospi1;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
static volatile uint32_t W_THRESH       = 160;  /* 16-bit ADC масштаб */
static volatile uint32_t W_DEADBAND_MS  = 10;

/* ADC DMA буфер */
static uint16_t adc_buf[ADC_BUF_LEN];

/* Состояние измерений */
static volatile uint32_t w_avg  = 0;
static volatile uint32_t w_filt = 0;
static volatile uint32_t w0     = 0;
static volatile uint32_t events = 0;
static volatile uint8_t  ui_force_redraw = 0;

/* Кольцевая очередь усреднённых значений */
#define AVG_Q_SIZE 1024
static volatile uint32_t avg_q[AVG_Q_SIZE];
static volatile uint32_t avg_q_ts[AVG_Q_SIZE];
static volatile uint32_t avg_q_cnt[AVG_Q_SIZE];
static volatile uint32_t avg_q_head = 0;
static volatile uint32_t avg_q_tail = 0;
static volatile uint32_t avg_q_overflow = 0;
static volatile uint32_t adc_half_cb = 0;
static volatile uint32_t adc_full_cb = 0;
static volatile uint32_t adc_last_cb_ms = 0;

/* Статистика */
static volatile uint16_t w_min = 0xFFFF;
static volatile uint16_t w_max = 0;

/* Структура записи лога */
typedef struct __attribute__((packed)) {
    uint32_t n;
    uint32_t freq_hz;
    int16_t  dW;
    uint8_t  dir;
    uint8_t  rsv0;
    uint32_t t_ms;
} log_rec_t;

/* Верификационный менеджер */
typedef enum {
    V_MGR_IDLE = 0, V_MGR_TRIGGERED, V_MGR_SEND_REQ,
    V_MGR_WAIT_START, V_MGR_VERIFYING, V_MGR_WAIT_DONE, V_MGR_NEXT
} v_mgr_state_t;

static struct {
    v_mgr_state_t state;
    uint32_t read_idx;
    uint32_t total_to_verify;
    uint32_t current_freq;
    uint32_t start_ms;
    int      confirmed_count;
    int      total_sweeps;
    uint32_t last_v_avg;
} v_mgr;

/* Flash logging */
static uint32_t log_used_bytes        = 0;
static uint32_t log_rec_no            = 0;
static uint32_t verified_write_addr   = VERIFIED_BASE_ADDR;
static uint32_t verified_count        = 0;
static volatile uint8_t  log_full     = 0;
static volatile uint8_t  log_error    = 0;
static uint32_t log_last_flush_ms     = 0;
static volatile uint32_t log_prog_to  = 0;
static volatile uint32_t log_erase_to = 0;

typedef enum { LOG_IO_IDLE = 0, LOG_IO_ERASE_WAIT, LOG_IO_PROG_WAIT } log_io_state_t;
static log_io_state_t log_io_state      = LOG_IO_IDLE;
static uint8_t        log_io_page       = 0;
static uint32_t       log_io_addr       = 0;
static uint32_t       log_io_sector_base= 0;
static uint32_t       log_io_started_ms = 0;
static uint32_t       log_io_next_poll_ms = 0;

static uint8_t          log_pages[2][LOG_PAGE_SIZE];
static uint32_t         log_pos[2]      = {0, 0};
static volatile uint8_t log_wr_idx      = 0;
static volatile uint8_t log_ready_mask  = 0;
static uint32_t         log_write_addr  = LOG_BASE_ADDR;
static uint32_t         log_next_erase_sector = LOG_BASE_ADDR;

static uint8_t sd_logger_start_requested = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI4_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_OCTOSPI1_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void Console_Log(const char *fmt, ...);
static void Log_Append(int16_t dW, uint8_t dir, uint32_t freq_hz, uint32_t t_ms);
static void process_avg(uint32_t a, uint32_t t_ms, uint32_t cnt);
static void V_ManagerTask(uint32_t now);
static void UI_Task(uint32_t now, uint32_t period_ms);
static void Log_FlushTask(uint32_t now);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Console_Log(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    for (int i = 0; i < len; i++) ITM_SendChar(buf[i]);
}

static inline uint32_t avg_u16(const uint16_t *p, uint32_t n) {
    uint32_t s = 0;
    const uint16_t *end4 = p + (n & ~3UL);
    while (p < end4) { s += p[0]+p[1]+p[2]+p[3]; p += 4; }
    const uint16_t *end = p + (n & 3UL);
    while (p < end) s += *p++;
    return s / n;
}

static inline void avg_queue_push(uint32_t avg, uint32_t t, uint32_t cnt) {
    uint32_t h = avg_q_head;
    uint32_t next = (h + 1U) & (AVG_Q_SIZE - 1U);
    if (next == avg_q_tail) { avg_q_overflow++; return; }
    avg_q[h] = avg; avg_q_ts[h] = t; avg_q_cnt[h] = cnt;
    __DMB(); avg_q_head = next;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adc_half_cb++;
        adc_last_cb_ms = HAL_GetTick();
        uint32_t a = avg_u16(&adc_buf[0], ADC_BUF_LEN / 2);
        avg_queue_push(a, HAL_GetTick(), htim2.Instance->CNT);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) {
        adc_full_cb++;
        adc_last_cb_ms = HAL_GetTick();
        uint32_t a = avg_u16(&adc_buf[ADC_BUF_LEN / 2], ADC_BUF_LEN / 2);
        avg_queue_push(a, HAL_GetTick(), htim2.Instance->CNT);
    }
}

static inline int u32_to_str(char *buf, uint32_t v) {
    char tmp[12]; int p = 0;
    if (v == 0U) tmp[p++] = '0';
    while (v != 0U && p < (int)sizeof(tmp)) { tmp[p++] = (char)('0'+(v%10U)); v/=10U; }
    int idx = 0;
    for (int i = p-1; i >= 0; --i) buf[idx++] = tmp[i];
    buf[idx] = '\0'; return idx;
}

static inline void u32_to_str_pad(char *out, size_t out_sz, uint32_t v, int width) {
    char tmp[16]; int len = u32_to_str(tmp, v);
    int pad = width - len; if (pad < 0) pad = 0;
    size_t i = 0;
    while (pad-- > 0 && i+1U < out_sz) out[i++] = ' ';
    for (int j = 0; j < len && i+1U < out_sz; ++j) out[i++] = tmp[j];
    out[i] = '\0';
}

static void process_avg(uint32_t a, uint32_t t_ms, uint32_t cnt) {
    (void)cnt;
    w_avg = a;
    if (a < w_min) w_min = (uint16_t)a;
    if (a > w_max) w_max = (uint16_t)a;

    uint32_t diff = (a > w_filt) ? (a - w_filt) : (w_filt - a);
    uint32_t alpha_num, alpha_den;
    if      (diff >= ADAPT_FAST_TH) { alpha_num=1U; alpha_den=2U; }
    else if (diff >= ADAPT_MED_TH)  { alpha_num=1U; alpha_den=3U; }
    else                             { alpha_num=1U; alpha_den=4U; }
    w_filt = ((alpha_den - alpha_num) * w_filt + alpha_num * a) / alpha_den;

    int32_t dW = (int32_t)w_filt - (int32_t)w0;
    uint32_t adW = (dW >= 0) ? (uint32_t)dW : (uint32_t)(-dW);
    static uint32_t t_last_evt = 0;

    if (adW >= W_THRESH && (t_ms - t_last_evt) >= W_DEADBAND_MS) {
        t_last_evt = t_ms;
        if (v_mgr.state == V_MGR_VERIFYING) {
            v_mgr.confirmed_count++;
            v_mgr.last_v_avg = a;
        }
        events++;
        w0 = (uint32_t)w_filt;
        uint8_t dir = (dW >= 0) ? 1U : 0U;
        uint32_t freq = FDCAN_GetLastFreq();
        Log_Append((int16_t)dW, dir, freq, t_ms);
    }
}

static void UI_Task(uint32_t now, uint32_t period_ms) {
    static uint32_t t_prev = 0;
    static uint32_t last_w_avg = 0xFFFFFFFFU;
    static uint32_t last_events = 0xFFFFFFFFU;
    static uint8_t  last_mode = 0xFFU;
    static uint32_t last_v_cnt = 0xFFFFFFFFU;
    static uint32_t last_used_kb = 0xFFFFFFFFU;
    static uint8_t  last_conn = 0xFFU;
    static uint32_t t_last_force = 0;

    if ((now - t_prev) < period_ms && !ui_force_redraw) return;
    t_prev = now;

    if ((now - t_last_force) >= 1000U) { t_last_force = now; last_w_avg = 0xFFFFFFFFU; }

    if (ui_force_redraw) {
        last_w_avg=0xFFFFFFFFU; last_events=0xFFFFFFFFU; last_mode=0xFFU;
        last_v_cnt=0xFFFFFFFFU; last_used_kb=0xFFFFFFFFU; last_conn=0xFFU;
        ui_force_redraw = 0;
    }
    char s[32];

    /* Строка 1: A:xxxx  E:xxxx */
    if (w_avg != last_w_avg) {
        last_w_avg = w_avg;
        s[0]='A'; s[1]=':'; s[2]='\0';
        u32_to_str_pad(&s[2], sizeof(s)-2U, w_avg, 5);
        ST7735_DrawString5x7_Scaled(0, UI_Y0, s, ST7735_WHITE, ST7735_BLACK, 2);
    }
    if (events != last_events) {
        last_events = events;
        s[0]='E'; s[1]=':'; s[2]='\0';
        u32_to_str_pad(&s[2], sizeof(s)-2U, events, 4);
        ST7735_DrawString5x7_Scaled(84, UI_Y0, s, ST7735_WHITE, ST7735_BLACK, 2);
    }

    /* Строка 2: M:x V:xxx [CON/DIS] */
    uint8_t cur_mode = FDCAN_GetLastMode();
    uint32_t last_rx = FDCAN_GetLastRxTick();
    uint32_t conn_tick = HAL_GetTick();
    uint8_t is_conn = (last_rx != 0 && (conn_tick - last_rx) < 2000U);
    if (is_conn != last_conn) last_mode = 0xFFU;
    if (cur_mode != last_mode || verified_count != last_v_cnt || is_conn != last_conn) {
        last_mode = cur_mode; last_v_cnt = verified_count; last_conn = is_conn;
        snprintf(s, sizeof(s), "M:%u V:%lu [%s]  ",
                 cur_mode, (unsigned long)verified_count, is_conn ? "CON" : "DIS");
        ST7735_DrawString5x7_Scaled(0, UI_Y1, s,
            is_conn ? ST7735_CYAN : ST7735_RED, ST7735_BLACK, 2);
    }

    /* Строка 3: LOG:xxxK xx% */
    uint32_t used_kb = log_used_bytes / 1024UL;
    if (used_kb != last_used_kb) {
        last_used_kb = used_kb;
        uint32_t pct = (LOG_TOTAL_BYTES == 0) ? 0 :
                       (log_used_bytes * 100UL) / LOG_TOTAL_BYTES;
        snprintf(s, sizeof(s), "LOG:%luK %lu%%   ",
                 (unsigned long)used_kb, (unsigned long)pct);
        ST7735_DrawString5x7_Scaled(0, UI_Y2, s, ST7735_YELLOW, ST7735_BLACK, 2);
    }

    /* Строка 4: F:xxxxxxx Hz */
    {
        static uint32_t last_freq = 0xFFFFFFFFU;
        uint32_t freq = FDCAN_GetLastFreq();
        if (freq != last_freq || (now - t_last_force) < period_ms) {
            last_freq = freq;
            snprintf(s, sizeof(s), "F:%7lu Hz  ", (unsigned long)freq);
            ST7735_DrawString5x7_Scaled(0, UI_Y3, s, ST7735_MAGENTA, ST7735_BLACK, 2);
        }
    }
}

static void Log_Append(int16_t dW, uint8_t dir, uint32_t freq_hz, uint32_t t_ms) {
    if (log_full || log_error) return;
    log_rec_t r;
    r.n = log_rec_no++; r.freq_hz = freq_hz;
    r.dW = dW; r.dir = dir; r.rsv0 = 0; r.t_ms = t_ms;

    uint32_t prim = __get_PRIMASK(); __disable_irq();
    uint8_t i = (uint8_t)log_wr_idx;
    if (log_ready_mask & (1U<<i)) {
        uint8_t j = (uint8_t)(i^1U);
        if (log_ready_mask & (1U<<j)) { __set_PRIMASK(prim); return; }
        log_wr_idx = j; i = j;
    }
    if (log_pos[i] + sizeof(r) > LOG_PAGE_SIZE) {
        log_ready_mask |= (1U<<i);
        uint8_t j = (uint8_t)(i^1U);
        if (log_ready_mask & (1U<<j)) { __set_PRIMASK(prim); return; }
        log_wr_idx = j; i = j;
        if (log_pos[i] + sizeof(r) > LOG_PAGE_SIZE) { __set_PRIMASK(prim); return; }
    }
    memcpy(&log_pages[i][log_pos[i]], &r, sizeof(r));
    __DMB(); log_pos[i] += sizeof(r);
    if (log_pos[i] == LOG_PAGE_SIZE) { log_ready_mask |= (1U<<i); log_wr_idx ^= 1U; }
    __set_PRIMASK(prim);
}

static void Log_FlushTask(uint32_t now) {
    if (log_full) return;
    if (log_error) {
        static uint32_t s_err_since = 0U; static uint8_t s_err_count = 0U;
        if (s_err_since == 0U) s_err_since = now;
        if (s_err_count < 3U && (now - s_err_since) >= 5000U) {
            log_error = 0; log_io_state = LOG_IO_IDLE; s_err_since = 0U; s_err_count++;
        } else return;
    }
    if (log_io_state == LOG_IO_ERASE_WAIT) {
        if ((int32_t)(now - log_io_started_ms) >= 2000) { log_erase_to++; log_error=1; return; }
        if ((int32_t)(now - log_io_next_poll_ms) < 0) return;
        log_io_next_poll_ms = now + LOG_IO_POLL_MS;
        if (W25qxx_IsBusy() == w25qxx_BUSY) return;
        log_next_erase_sector = log_io_sector_base + LOG_SECTOR_SIZE;
        uint8_t st = W25qxx_PageProgramStart(log_pages[log_io_page], log_io_addr, LOG_PAGE_SIZE);
        if (st == w25qxx_BUSY) return;
        if (st != w25qxx_OK) { log_error=1; return; }
        log_io_state = LOG_IO_PROG_WAIT; log_io_started_ms = now;
        log_io_next_poll_ms = now + LOG_IO_POLL_MS; return;
    }
    if (log_io_state == LOG_IO_PROG_WAIT) {
        if ((int32_t)(now - log_io_started_ms) >= 100) { log_prog_to++; log_error=1; return; }
        if ((int32_t)(now - log_io_next_poll_ms) < 0) return;
        log_io_next_poll_ms = now + LOG_IO_POLL_MS;
        if (W25qxx_IsBusy() == w25qxx_BUSY) return;
        __disable_irq(); log_pos[log_io_page]=0;
        log_ready_mask &= (uint8_t)~(1U<<log_io_page); __enable_irq();
        log_write_addr += LOG_PAGE_SIZE; log_used_bytes += LOG_PAGE_SIZE;
        if (log_write_addr + LOG_PAGE_SIZE > LOG_END_ADDR) log_full = 1;
        log_io_state = LOG_IO_IDLE; return;
    }
    if (log_write_addr + LOG_PAGE_SIZE > LOG_END_ADDR) { log_full=1; return; }
    uint8_t ready; __disable_irq(); ready = log_ready_mask; __enable_irq();
    if (!ready) return;
    uint8_t page = (ready & 0x01) ? 0 : 1;
    if (log_pos[page] == 0) return;
    log_io_page = page; log_io_addr = log_write_addr;
    log_io_sector_base = log_io_addr & ~(LOG_SECTOR_SIZE - 1U);
    log_io_started_ms = now; log_io_next_poll_ms = now + LOG_IO_POLL_MS;
    if (log_io_addr == log_next_erase_sector) {
        uint8_t st = W25qxx_SectorEraseStart(log_io_sector_base);
        if (st == w25qxx_BUSY) return;
        if (st != w25qxx_OK) { log_error=1; return; }
        log_io_state = LOG_IO_ERASE_WAIT; return;
    }
    uint8_t st = W25qxx_PageProgramStart(log_pages[page], log_io_addr, LOG_PAGE_SIZE);
    if (st == w25qxx_BUSY) return;
    if (st != w25qxx_OK) { log_error=1; return; }
    log_io_state = LOG_IO_PROG_WAIT;
}

static void V_ManagerTask(uint32_t now) {
    switch (v_mgr.state) {
    case V_MGR_IDLE:
        if (log_used_bytes >= (LOG_TOTAL_BYTES * 6 / 10)) {
            v_mgr.state = V_MGR_TRIGGERED;
            v_mgr.read_idx = 0; v_mgr.total_to_verify = log_rec_no;
        }
        break;
    case V_MGR_TRIGGERED:
        if (v_mgr.read_idx >= v_mgr.total_to_verify) { v_mgr.state=V_MGR_IDLE; break; }
        { log_rec_t r;
          if (W25qxx_Read((uint8_t*)&r, LOG_BASE_ADDR+v_mgr.read_idx*sizeof(log_rec_t), sizeof(r))==w25qxx_OK)
            { v_mgr.current_freq=r.freq_hz; v_mgr.state=V_MGR_SEND_REQ; }
          else v_mgr.read_idx++; }
        break;
    case V_MGR_SEND_REQ:
        if (FDCAN_SendVerifyRequest(v_mgr.current_freq)==0)
            { v_mgr.state=V_MGR_WAIT_START; v_mgr.start_ms=now; }
        break;
    case V_MGR_WAIT_START:
        if (FDCAN_GetVerifyStatus()==VERIFY_START || FDCAN_GetVerifyStatus()==VERIFY_BUSY)
            { v_mgr.state=V_MGR_VERIFYING; v_mgr.confirmed_count=0; v_mgr.start_ms=now; }
        else if (now-v_mgr.start_ms>2000) { v_mgr.state=V_MGR_TRIGGERED; v_mgr.read_idx++; }
        break;
    case V_MGR_VERIFYING:
        if (FDCAN_GetVerifyStatus()==VERIFY_DONE) {
            if (v_mgr.confirmed_count>=5) {
                log_rec_t vr={0}; vr.n=verified_count++;
                vr.freq_hz=v_mgr.current_freq; vr.t_ms=now;
                if (verified_write_addr+sizeof(vr)<VERIFIED_END_ADDR) {
                    W25qxx_PageProgram((uint8_t*)&vr, verified_write_addr, sizeof(vr));
                    verified_write_addr += sizeof(vr);
                }
                SD_Logger_LogLine(vr.freq_hz, v_mgr.last_v_avg);
            }
            v_mgr.state=V_MGR_NEXT;
        } else if (now-v_mgr.start_ms>30000) v_mgr.state=V_MGR_NEXT;
        break;
    case V_MGR_NEXT:
        v_mgr.read_idx++; v_mgr.state=V_MGR_TRIGGERED; break;
    default: break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI4_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_OCTOSPI1_Init();
  MX_FDCAN1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  static uint8_t system_ready = 0;
  static uint8_t h5_reboot_requested = 0;
  uint32_t start_time = HAL_GetTick();

  HAL_TIM_Base_Start(&htim2);

  if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
      Console_Log("[ADC] TIM6 start failed\n");

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) != HAL_OK)
      Console_Log("[ADC] DMA start failed\n");
  adc_last_cb_ms = HAL_GetTick();

  FDCAN_Comm_Init();

  ST7735_Init();
  ST7735_FillScreen(ST7735_BLACK);

  w25qxx_Init();
  Console_Log("[QSPI] ID=0x%04X SR=[0x%02X 0x%02X 0x%02X]\n",
      (unsigned int)w25qxx_ID,
      (unsigned int)w25qxx_StatusReg[0],
      (unsigned int)w25qxx_StatusReg[1],
      (unsigned int)w25qxx_StatusReg[2]);

  Console_Log("--- H723 MEAS START ---\n");
  Console_Log("Clock: %lu Hz\n", HAL_RCC_GetSysClockFreq());

  events = 0;
  ui_force_redraw = 1;
  log_used_bytes = 0; log_rec_no = 0;
  log_write_addr = LOG_BASE_ADDR; log_next_erase_sector = LOG_BASE_ADDR;
  log_pos[0] = 0; log_pos[1] = 0;
  log_wr_idx = 0; log_ready_mask = 0;
  log_full = 0; log_error = 0; log_io_state = LOG_IO_IDLE;
  log_io_page = 0; log_io_addr = 0; log_io_sector_base = 0;
  log_io_started_ms = 0; log_io_next_poll_ms = 0;
  log_last_flush_ms = HAL_GetTick();
  sd_logger_start_requested = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if (!h5_reboot_requested && system_ready) {
        h5_reboot_requested = 1;
        FDCAN_SendRemoteReboot();
    }

    if (!system_ready && (now - start_time > UI_STARTUP_SETTLE_MS)) {
        uint32_t current_raw = avg_u16(adc_buf, ADC_BUF_LEN);
        system_ready = 1;
        __disable_irq(); avg_q_tail = avg_q_head; __enable_irq();
        w_avg = current_raw; w_filt = current_raw; w0 = current_raw;
        events = 0;
        ST7735_FillScreen(ST7735_BLACK);
        ui_force_redraw = 1;
        Console_Log("System Ready: baseline=%lu\n", current_raw);
    }

    V_ManagerTask(now);
    FDCAN_PollRx();

    if (!log_full && !log_error && (now - log_last_flush_ms) >= 20) {
        log_last_flush_ms = now;
        __disable_irq();
        uint8_t i = (uint8_t)log_wr_idx;
        if (!(log_ready_mask & (1U<<i)) && log_pos[i] > 0)
            log_ready_mask |= (1U<<i);
        __enable_irq();
    }

    Log_FlushTask(now);

    uint32_t head = avg_q_head; uint32_t processed = 0;
    while (avg_q_tail != head && processed < ADC_PROCESS_BUDGET) {
        uint32_t t = avg_q_ts[avg_q_tail];
        uint32_t a = avg_q[avg_q_tail];
        uint32_t cnt = avg_q_cnt[avg_q_tail];
        avg_q_tail = (avg_q_tail + 1U) & (AVG_Q_SIZE - 1U);
        if (system_ready) process_avg(a, t, cnt);
        processed++; head = avg_q_head;
    }

    if ((now - adc_last_cb_ms) > 1000U) {
        HAL_ADC_Stop_DMA(&hadc1);
        (void)HAL_TIM_Base_Stop(&htim6);
        (void)HAL_TIM_Base_Start(&htim6);
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN) == HAL_OK) {
            adc_last_cb_ms = now;
            Console_Log("[ADC] DMA restarted\n");
        }
    }

    UI_Task(now, 500);

    if (system_ready && !sd_logger_start_requested) {
        (void)SD_Logger_Init();
        sd_logger_start_requested = 1U;
    }
    SD_Logger_Process(now);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_OSPI|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 1;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 5;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
      /* Non-fatal: SD_Logger handles retry logic */
  }

  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 5999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 8;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_HEARTBEAT_Pin|LCD_BL_Pin|LCD_CS_Pin|LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_HEARTBEAT_Pin LCD_BL_Pin LCD_CS_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LED_HEARTBEAT_Pin|LCD_BL_Pin|LCD_CS_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
