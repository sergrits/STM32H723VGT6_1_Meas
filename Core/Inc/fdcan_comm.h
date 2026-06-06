/**
 * @file fdcan_comm.h
 * @brief FDCAN communication protocol definitions and function declarations
 * @details
 *   - Frequency transmission from H5 to H7 (ID 0x123)
 *   - Verification requests/responses (0x124-0x126)
 *   - Mode status (0x127)
 *   - Remote reboot capability (0x12A-0x12B)
 *   - PAUSE/RESUME commands for SD operations (0x128-0x129)
 */

#ifndef FDCAN_COMM_H
#define FDCAN_COMM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CAN ID DEFINITIONS (FDCAN Standard IDs)
 * ============================================================================ */

/** @brief Frequency data from H5 → H7 (4 bytes: frequency in Hz) */
#define FDCAN_FREQ_STD_ID           0x123U

/** @brief Verification request from H7 → H5 (4 bytes: target frequency) */
#define FDCAN_VERIFY_REQ_ID         0x124U

/** @brief Verification status from H5 → H7 (1 byte: status enum) */
#define FDCAN_VERIFY_STATUS_ID      0x125U

/** @brief Verification result from H7 → H5 (1 byte: confirmed yes/no) */
#define FDCAN_VERIFY_RES_ID         0x126U

/** @brief PWM mode status from H5 → H7 (1 byte: mode index) */
#define FDCAN_MODE_STATUS_ID        0x127U

/** @brief PAUSE command from H7 → H5 (stop PWM generation) */
#define FDCAN_PAUSE_CMD_ID          0x128U

/** @brief RESUME command from H7 → H5 (start PWM generation) */
#define FDCAN_RESUME_CMD_ID         0x129U

/** @brief Remote reboot command from H7 → H5 (emergency reset) */
#define FDCAN_REMOTE_REBOOT_ID      0x12AU

/** @brief Reboot ACK from H5 → H7 (confirmation before reset) */
#define FDCAN_REBOOT_ACK_ID         0x12BU

/* ============================================================================
 * TYPE DEFINITIONS
 * ============================================================================ */

/**
 * @enum verify_status_t
 * @brief Verification state machine status
 */
typedef enum {
    VERIFY_IDLE = 0,        /**< No verification in progress */
    VERIFY_START,           /**< Verification started by H7 */
    VERIFY_BUSY,            /**< H5 performing sweep/verification */
    VERIFY_DONE,            /**< Verification completed successfully */
    VERIFY_FAIL             /**< Verification failed (mismatch detected) */
} verify_status_t;

/**
 * @struct fdcan_diag_t
 * @brief CAN bus diagnostic counters
 */
typedef struct {
    uint8_t tx_error_count;     /**< TX error counter (0-255) */
    uint8_t rx_error_count;     /**< RX error counter (0-255) */
    uint32_t last_check_ms;     /**< Timestamp of last check (ms) */
} fdcan_diag_t;

/* ============================================================================
 * FUNCTION DECLARATIONS
 * ============================================================================ */

/**
 * @brief Initialize FDCAN communication
 * @details Sets up filters, starts CAN, enables interrupts
 * @return void
 */
void FDCAN_Comm_Init(void);

/**
 * @brief Polling RX fallback (for systems with interrupt issues)
 * @details Should be called regularly from main loop
 * @return void
 */
void FDCAN_PollRx(void);

/**
 * @brief Send frequency verification request to H5
 * @param freq_hz Target frequency in Hz to verify
 * @return 0 if success, -1 if TX FIFO full
 */
int FDCAN_SendVerifyRequest(uint32_t freq_hz);

/**
 * @brief Send verification result to H5
 * @param confirmed 1 if frequency confirmed, 0 if rejected
 * @return 0 if success, -1 if TX FIFO full
 */
int FDCAN_SendVerifyResult(uint8_t confirmed);

/**
 * @brief Send remote reboot command to H5
 * @details Triggers NVIC_SystemReset() on H5
 * @return 0 if success, -1 if TX FIFO full
 */
int FDCAN_SendRemoteReboot(void);

/**
 * @brief Send PAUSE command to H5 (stop PWM)
 * @details Used before critical SD write operations
 * @return 0 if success, -1 if TX FIFO full
 */
int FDCAN_SendPauseCommand(void);

/**
 * @brief Send RESUME command to H5 (start PWM)
 * @details Used after critical SD write completes
 * @return 0 if success, -1 if TX FIFO full
 */
int FDCAN_SendResumeCommand(void);

/**
 * @brief Get last received frequency from H5
 * @return Frequency in Hz (0 if no message received yet)
 */
uint32_t FDCAN_GetLastFreq(void);

/**
 * @brief Get last received PWM mode from H5
 * @return Mode index (0-3 typically)
 */
uint8_t FDCAN_GetLastMode(void);

/**
 * @brief Get total count of RX messages
 * @return RX message counter
 */
uint32_t FDCAN_GetRxCount(void);

/**
 * @brief Get timestamp of last RX message
 * @return HAL_GetTick() value at last RX event
 */
uint32_t FDCAN_GetLastRxTick(void);

/**
 * @brief Get current verification status
 * @return Current state from verify_status_t enum
 */
verify_status_t FDCAN_GetVerifyStatus(void);

/**
 * @brief Get CAN bus diagnostic counters (error monitoring)
 * @details Reads TX/RX error counters from CAN controller
 * @return fdcan_diag_t structure with current error counts
 */
fdcan_diag_t FDCAN_GetDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* FDCAN_COMM_H */
