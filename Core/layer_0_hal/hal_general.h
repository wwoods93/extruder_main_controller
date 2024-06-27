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

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


#define REGISTER_READ(REG)                  ((REG))
#define REGISTER_CHECK_SET_BIT(REG, BIT)    ((((REG) & (BIT)) == (BIT)) ? BIT_SET : BIT_CLEAR)
#define UNUSED_CAST_VOID(X)                 (void) X

//void assert_failed(uint8_t *file, uint32_t line);
//#define ASSERT_PARAM(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))

/*************************************** spi control register 1 definitions *******************************************/

/**********************************************************************************************************************/

#define I2C_CR1_MODULE_ENABLE_POSITION    (0U)
#define I2C_CR1_MODULE_ENABLE_MASK        (0x1UL << I2C_CR1_MODULE_ENABLE_POSITION)
#define I2C_CR1_MODULE_ENABLE             I2C_CR1_MODULE_ENABLE_MASK

typedef enum
{
    HAL_MODULE_UNLOCKED = 0x00U,
    HAL_MODULE_LOCKED   = 0x01U
} hal_lock_t;

typedef enum
{
    FLAG_RESET  = 0x00U,
    FLAG_SET    = 0x01U
} flag_status_t;

typedef enum
{
    BIT_CLEAR = 0x00,
    BIT_SET = 0x01
} bit_status_t;

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

#define I2C_2_BASE_ADDRESS                  (APB1_PERIPHERAL_BASE_ADDRESS + 0x5800UL)
#define I2C_2                               ((hal_i2c_t *) I2C_2_BASE_ADDRESS)

/******************************************************** i2c *********************************************************/
typedef struct
{
    volatile uint32_t CONTROL_REG_1;
    volatile uint32_t CONTROL_REG_2;
    volatile uint32_t OWN_ADDRESS_REG_1;
    volatile uint32_t OWN_ADDRESS_REG_2;
    volatile uint32_t DATA_REG;
    volatile uint32_t STATUS_REG_1;
    volatile uint32_t STATUS_REG_2;
    volatile uint32_t CLOCK_CONTROL_REG;
    volatile uint32_t RISE_TIME_REG;
    volatile uint32_t FILTER_REG;
} hal_i2c_t;

/******************************************************** spi *********************************************************/

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

/******************************************************** dma *********************************************************/
typedef struct
{
    volatile uint32_t STREAM_CONFIG_REG;
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
} dma_handle_t;
/**********************************************************************************************************************/

#define HAL_STM_I2C_7_BIT_ADDRESS_MODE         0x00004000U
#define I2C_ADDRESSINGMODE_10BIT        (I2C_OAR1_ADDMODE | 0x00004000U)
#define HAL_STM_I2C_DUTY_CYCLE_2                 0x00000000U
#define HAL_STM_I2C_DUAL_ADDRESS_DISABLE        0x00000000U
#define HAL_STM_I2C_GENERAL_CALL_DISABLE        0x00000000U
#define HAL_STM_I2C_NO_STRETCH_DISABLE          0x00000000U

#define STM_HAL_I2C_FLAG_BUS_BUSY                         0x00100002U
#define STM_HAL_I2C_FLAG_START_BIT_SET                    0x00010001U
#define STM_HAL_I2C_FLAG_10_BIT_HEADER_SENT               0x00010008U
#define STM_HAL_I2C_FLAG_ADDRESS_SENT                     0x00010002U
#define STM_HAL_I2C_FLAG_ACKNOWLEDGE_FAILED               0x00010400U
#define STM_HAL_I2C_TRANSMIT_BUFFER_EMPTY                 0x00010080U
#define STM_HAL_I2C_FLAG_BYTE_TRANSFER_FINISHED           0x00010004U

#define STM_HAL_I2C_ERROR_NONE                            0x00000000U
#define STM_HAL_I2C_ERROR_BUSY                            0x00000001U
#define STM_HAL_I2C_ERROR_ARBITRATION_LOST                0x00000002U
#define STM_HAL_I2C_ERROR_ACKNOWLEDGE_FAILED              0x00000004U
#define STM_HAL_I2C_ERROR_OVERRUN                         0x00000008U
#define STM_HAL_I2C_ERROR_DMA_TRANSFER                    0x00000010U
#define STM_HAL_I2C_ERROR_TIMEOUT                         0x00000020U
#define STM_HAL_I2C_ERROR_SIZE                            0x00000040U
#define STM_HAL_I2C_ERROR_DMA_PARAMETER                   0x00000080U
#define STM_HAL_I2C_WRONG_START                           0x00000200U
/**********************************************************************************************************************/
#endif //MAIN_CONTROLLER_HAL_GENERAL_H
