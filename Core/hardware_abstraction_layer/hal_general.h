/***********************************************************************************************************************
 * Main_Controller
 * hal_general.h
 *
 * wilson
 * 10/21/22
 * 9:18 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_GENERAL_H
#define MAIN_CONTROLLER_HAL_GENERAL_H

#include <cstdint>

#define HAL_GENERAL_SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define HAL_GENERAL_CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))
#define HAL_GENERAL_READ_REG(REG)           ((REG))
#define HAL_GENERAL_UNUSED(X)               (void)X
#define HAL_GENERAL_IS_BIT_SET(REG, BIT)    (((REG) & (BIT)) == (BIT))
#define HAL_GENERAL_IS_BIT_CLR(REG, BIT)    (((REG) & (BIT)) == 0U)

#if (USE_RTOS == 1U)
  #error "USE_RTOS should be 0 in the current HAL release"
#else
    #define HAL_LOCK_MODULE(__HANDLE__)                                                                             \
                                    do                                                                              \
                                    {                                                                               \
                                        if((__HANDLE__)->lock == HAL_MODULE_LOCKED) { return HAL_STATUS_BUSY; }     \
                                        else { (__HANDLE__)->lock = HAL_MODULE_LOCKED; }                            \
                                    } while (0U)                                                                    \

    #define HAL_UNLOCK_MODULE(__HANDLE__) do { (__HANDLE__)->lock = HAL_MODULE_UNLOCKED; } while (0U)
#endif
/*************************************** spi control register 1 definitions *******************************************/
#define HAL_SPI_CR1_CLOCK_PHASE_POSITION                            (0U)
#define HAL_SPI_CR1_CLOCK_PHASE_MASK                                (0x1UL << HAL_SPI_CR1_CLOCK_PHASE_POSITION)
#define HAL_SPI_CR1_CLOCK_PHASE                                     HAL_SPI_CR1_CLOCK_PHASE_MASK
#define HAL_SPI_CR1_CLOCK_POLARITY_POSITION                         (1U)
#define HAL_SPI_CR1_CLOCK_POLARITY_MASK                             (0x1UL << HAL_SPI_CR1_CLOCK_POLARITY_POSITION)
#define HAL_SPI_CR1_CLOCK_POLARITY                                  HAL_SPI_CR1_CLOCK_POLARITY_MASK
#define HAL_SPI_CR1_CONTROLLER_POSITION                             (2U)
#define HAL_SPI_CR1_CONTROLLER_MASK                                 (0x1UL << HAL_SPI_CR1_CONTROLLER_POSITION)
#define HAL_SPI_CR1_CONTROLLER                                      HAL_SPI_CR1_CONTROLLER_MASK
#define HAL_SPI_CR1_BAUD_RATE_POSITION                              (3U)
#define HAL_SPI_CR1_BAUD_RATE_MASK                                  (0x7UL << HAL_SPI_CR1_BAUD_RATE_POSITION)
#define HAL_SPI_CR1_BAUD_RATE                                       HAL_SPI_CR1_BAUD_RATE_MASK
#define HAL_SPI_CR1_BAUD_RATE_0                                     (0x1UL << HAL_SPI_CR1_BAUD_RATE_POSITION)
#define HAL_SPI_CR1_BAUD_RATE_1                                     (0x2UL << HAL_SPI_CR1_BAUD_RATE_POSITION)
#define HAL_SPI_CR1_BAUD_RATE_2                                     (0x4UL << HAL_SPI_CR1_BAUD_RATE_POSITION)
#define HAL_SPI_CR1_SPI_ENABLE_POSITION                             (6U)
#define HAL_SPI_CR1_SPI_ENABLE_MASK                                 (0x1UL << HAL_SPI_CR1_SPI_ENABLE_POSITION)
#define HAL_SPI_CR1_SPI_ENABLE                                      HAL_SPI_CR1_SPI_ENABLE_MASK
#define HAL_SPI_CR1_LSB_FIRST_POSITION                              (7U)
#define HAL_SPI_CR1_LSB_FIRST_MASK                                  (0x1UL << HAL_SPI_CR1_LSB_FIRST_POSITION)
#define HAL_SPI_CR1_LSB_FIRST                                       HAL_SPI_CR1_LSB_FIRST_MASK
#define HAL_SPI_CR1_INTERNAL_CHIP_SELECT_POSITION                   (8U)
#define HAL_SPI_CR1_INTERNAL_CHIP_SELECT_MASK                       (0x1UL << HAL_SPI_CR1_INTERNAL_CHIP_SELECT_POSITION)
#define HAL_SPI_CR1_INTERNAL_CHIP_SELECT                            HAL_SPI_CR1_INTERNAL_CHIP_SELECT_MASK
#define HAL_SPI_CR1_SOFTWARE_CHIP_SELECT_POSITION                   (9U)
#define HAL_SPI_CR1_SOFTWARE_CHIP_SELECT_MASK                       (0x1UL << HAL_SPI_CR1_SOFTWARE_CHIP_SELECT_POSITION)
#define HAL_SPI_CR1_SOFTWARE_CHIP_SELECT                            HAL_SPI_CR1_SOFTWARE_CHIP_SELECT_MASK
#define HAL_SPI_CR1_RECEIVE_ONLY_POSITION                           (10U)
#define HAL_SPI_CR1_RECEIVE_ONLY_MASK                               (0x1UL << HAL_SPI_CR1_RECEIVE_ONLY_POSITION)
#define HAL_SPI_CR1_RECEIVE_ONLY                                    HAL_SPI_CR1_RECEIVE_ONLY_MASK
#define HAL_SPI_CR1_DATA_FRAME_FORMAT_POSITION                      (11U)
#define HAL_SPI_CR1_DATA_FRAME_FORMAT_MASK                          (0x1UL << HAL_SPI_CR1_DATA_FRAME_FORMAT_POSITION)
#define HAL_SPI_CR1_DATA_FRAME_FORMAT                               HAL_SPI_CR1_DATA_FRAME_FORMAT_MASK
#define HAL_SPI_CR1_CRC_ENABLE_POSITION                             (13U)
#define HAL_SPI_CR1_CRC_ENABLE_MASK                                 (0x1UL << HAL_SPI_CR1_CRC_ENABLE_POSITION)
#define HAL_SPI_CR1_CRC_ENABLE                                      HAL_SPI_CR1_CRC_ENABLE_MASK
#define HAL_SPI_CR1_BIDIRECTIONAL_MODE_POSITION                     (15U)
#define HAL_SPI_CR1_BIDIRECTIONAL_MODE_MASK                         (0x1UL << HAL_SPI_CR1_BIDIRECTIONAL_MODE_POSITION)
#define HAL_SPI_CR1_BIDIRECTIONAL_MODE                              HAL_SPI_CR1_BIDIRECTIONAL_MODE_MASK
/*************************************** spi control register 2 definitions *******************************************/
#define HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION                   (0U)
#define HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK                       0x1UL << HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION
#define HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE                            HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK
#define HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION                   (1U)
#define HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK                       0x1UL << HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION
#define HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE                            HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK
#define HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_POSITION              (2U)
#define HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_MASK                  (0x1UL << HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_POSITION)
#define HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE                       HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_MASK
#define HAL_SPI_CR2_FRAME_FORMAT_POSITION                           (4U)
#define HAL_SPI_CR2_FRAME_FORMAT_MASK                               (0x1UL << HAL_SPI_CR2_FRAME_FORMAT_POSITION)
#define HAL_SPI_CR2_FRAME_FORMAT                                    HAL_SPI_CR2_FRAME_FORMAT_MASK
#define HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_POSITION                 (5U)
#define HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_MASK                     (0x1UL << HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_POSITION)
#define HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE                          HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_MASK
#define HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION   (6U)
#define HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK       (0x1UL << HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION)
#define HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE            HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK
#define HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION       (7U)
#define HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK           (0x1UL << HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION)
#define HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE                HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK
/***************************************** spi status register definitions ********************************************/
#define HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_POSITION                     (0U)
#define HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_MASK                         (0x1UL << HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_POSITION)
#define HAL_SPI_SR_RX_BUFFER_NOT_EMPTY                              HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_MASK
#define HAL_SPI_SR_TX_BUFFER_EMPTY_POSITION                         (1U)
#define HAL_SPI_SR_TX_BUFFER_EMPTY_MASK                             (0x1UL << HAL_SPI_SR_TX_BUFFER_EMPTY_POSITION)
#define HAL_SPI_SR_TX_BUFFER_EMPTY                                  HAL_SPI_SR_TX_BUFFER_EMPTY_MASK
#define HAL_SPI_SR_CRC_ERROR_POSITION                               (4U)
#define HAL_SPI_SR_CRC_ERROR_MASK                                   (0x1UL << HAL_SPI_SR_CRC_ERROR_POSITION)
#define HAL_SPI_SR_CRC_ERROR                                        HAL_SPI_SR_CRC_ERROR_MASK
#define HAL_SPI_SR_MODE_FAULT_POSITION                              (5U)
#define HAL_SPI_SR_MODE_FAULT_MASK                                  (0x1UL << HAL_SPI_SR_MODE_FAULT_POSITION)
#define HAL_SPI_SR_MODE_FAULT                                       HAL_SPI_SR_MODE_FAULT_MASK
#define HAL_SPI_SR_OVERRUN_POSITION                                 (6U)
#define HAL_SPI_SR_OVERRUN_MASK                                     (0x1UL << HAL_SPI_SR_OVERRUN_POSITION)
#define HAL_SPI_SR_OVERRUN                                          HAL_SPI_SR_OVERRUN_MASK
#define HAL_SPI_SR_BUSY_POSITION                                    (7U)
#define HAL_SPI_SR_BUSY_MASK                                        (0x1UL << HAL_SPI_SR_BUSY_POSITION)
#define HAL_SPI_SR_BUSY                                             HAL_SPI_SR_BUSY_MASK
#define HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_POSITION              (8U)
#define HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_MASK                  (0x1UL << HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_POSITION)
#define HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR                       HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_MASK
/**********************************************************************************************************************/
#define HAL_SPI_I2S_MODE_SELECT_POSITION      (11U)
#define HAL_SPI_I2S_MODE_SELECT_MASK      (0x1UL << HAL_SPI_I2S_MODE_SELECT_POSITION)           /*!< 0x00000800 */
#define HAL_SPI_I2S_MODE_SELECT          HAL_SPI_I2S_MODE_SELECT_MASK                     /*!<I2S mode selection */

/**************************************** dma control register definitions ********************************************/
#define HAL_DMA_SxCR_ENABLE_POSITION                                (0U)
#define HAL_DMA_SxCR_ENABLE_MASK                                    (0x1UL << HAL_DMA_SxCR_ENABLE_POSITION)
#define HAL_DMA_SxCR_ENABLE                                         HAL_DMA_SxCR_ENABLE_MASK
/**********************************************************************************************************************/

typedef enum
{
    HAL_MODULE_UNLOCKED = 0x00U,
    HAL_MODULE_LOCKED   = 0x01U
} hal_lock_t;

typedef enum
{
    FLAG_RESET = 0x00U,
    FLAG_SET   = 0x01U
} flag_status_t;

typedef enum
{
    HAL_STATUS_OK       = 0x00U,
    HAL_STATUS_ERROR    = 0x01U,
    HAL_STATUS_BUSY     = 0x02U,
    HAL_STATUS_TIMEOUT  = 0x03U
} hal_status_t;

#define PERIPHERAL_BASE_ADDRESS             0x40000000UL
#define APB1_PERIPHERAL_BASE_ADDRESS        PERIPHERAL_BASE_ADDRESS
#define APB2_PERIPHERAL_BASE_ADDRESS        (PERIPHERAL_BASE_ADDRESS + 0x00010000UL)

#define SPI_1_BASE_ADDRESS                  (APB2_PERIPHERAL_BASE_ADDRESS + 0x3000UL)
#define SPI_2_BASE_ADDRESS                  (APB1_PERIPHERAL_BASE_ADDRESS + 0x3800UL)
#define SPI_3_BASE_ADDRESS                  (APB1_PERIPHERAL_BASE_ADDRESS + 0x3C00UL)
#define SPI_4_BASE_ADDRESS                  (APB2_PERIPHERAL_BASE_ADDRESS + 0x3400UL)

#define SPI_1                               ((hal_spi_t *) SPI_1_BASE_ADDRESS)
#define SPI_2                               ((hal_spi_t *) SPI_2_BASE_ADDRESS)
#define SPI_3                               ((hal_spi_t *) SPI_3_BASE_ADDRESS)
#define SPI_4                               ((hal_spi_t *) SPI_4_BASE_ADDRESS)


typedef struct
{
    volatile uint32_t CONTROL_REG_1;
    volatile uint32_t CONTROL_REG_2;
    volatile uint32_t STATUS_REG;
    volatile uint32_t DATA_REG;
    volatile uint32_t CRC_POLYNOMIAL_REG;
    volatile uint32_t CRC_RX_REG;
    volatile uint32_t CRC_TX_REG;
    volatile uint32_t I2S_CONFIG_REG;
    volatile uint32_t I2S_PRESCALER_REG;
} hal_spi_t;
/*************************************************** dma ******************************************************/
typedef struct
{
    volatile uint32_t CONFIG_REG;
    volatile uint32_t DATA_REG_NUMBER;
    volatile uint32_t PERIPHERAL_ADDRESS_REG;
    volatile uint32_t MEM_0_ADDRESS_REG;
    volatile uint32_t MEM_1_ADDRESS_REG;
    volatile uint32_t FIFO_CONTROL_REG;
} dma_stream_t;

typedef struct
{
    uint32_t channel;                               // DMA_Channel_selection
    uint32_t direction;                             // DMA_Data_transfer_direction
    uint32_t peripheral_address_reg_increment;      // DMA_Peripheral_incremented_mode
    uint32_t memory_address_reg_increment;          // DMA_Memory_incremented_mode
    uint32_t peripheral_data_width;                 // DMA_Peripheral_data_size
    uint32_t memory_data_width;                     // DMA_Memory_data_size
    uint32_t mode;                                  // DMA_mode
    uint32_t priority;                              // DMA_Priority_level
    uint32_t FIFOMode;                              // DMA_FIFO_direct_mode
    uint32_t fifo_threshold_level;                  // DMA_FIFO_threshold_level
    uint32_t memory_burst_transfer_config;          // DMA_Memory_burst
    uint32_t peripheral_burst_transfer_config;      // DMA_Peripheral_burst
}dma_init_t;

typedef enum
{
    DMA_STATE_RESET             = 0x00U,
    DMA_STATE_READY             = 0x01U,
    DMA_STATE_BUSY              = 0x02U,
    DMA_STATE_TIMEOUT           = 0x03U,
    DMA_STATE_ERROR             = 0x04U,
    DMA_STATE_ABORT             = 0x05U,
}dma_state_t;

typedef struct _dma_handle_t
{
    dma_stream_t                *instance;
    dma_init_t                  init;
    hal_lock_t                  lock;
    volatile dma_state_t        state;
    void                        *parent;
    void                        (* transfer_complete_callback)              ( struct _dma_handle_t * hdma);
    void                        (* transfer_half_complete_callback)         ( struct _dma_handle_t * hdma);
    void                        (* transfer_mem_1_complete_callback)        ( struct _dma_handle_t * hdma);
    void                        (* transfer_mem_1_half_complete_callback)   ( struct _dma_handle_t * hdma);
    void                        (* transfer_error_callback)                 ( struct _dma_handle_t * hdma);
    void                        (* transfer_abort_callback)                 ( struct _dma_handle_t * hdma);
    volatile uint32_t           error_code;
    uint32_t                    stream_base_address;
    uint32_t                    stream_index;
}dma_handle_t;
/**************************************************************************************************************/
#endif //MAIN_CONTROLLER_HAL_GENERAL_H
