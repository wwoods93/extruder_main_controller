/***********************************************************************************************************************
 * Main_Controller
 * hal_spi_definitions.h
 *
 * wilson
 * 6/24/24
 * 10:15 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_SPI_DEFINITIONS_H
#define MAIN_CONTROLLER_HAL_SPI_DEFINITIONS_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* hal includes */
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


// spi cr1 definitions
static constexpr uint32_t SPI_CR1_BIT_CLOCK_PHASE_POSITION                              = 0U;
static constexpr uint32_t SPI_CR1_BIT_CLOCK_PHASE_MASK                                  = 0x1UL << SPI_CR1_BIT_CLOCK_PHASE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_CLOCK_PHASE                                       = SPI_CR1_BIT_CLOCK_PHASE_MASK;

static constexpr uint32_t SPI_CR1_BIT_CLOCK_POLARITY_POSITION                           = 1U;
static constexpr uint32_t SPI_CR1_BIT_CLOCK_POLARITY_MASK                               = 0x1UL << SPI_CR1_BIT_CLOCK_POLARITY_POSITION;
static constexpr uint32_t SPI_CR1_BIT_CLOCK_POLARITY                                    = SPI_CR1_BIT_CLOCK_POLARITY_MASK;

static constexpr uint32_t SPI_CR1_BIT_CONTROLLER_MODE_POSITION                          = 2U;
static constexpr uint32_t SPI_CR1_BIT_CONTROLLER_MODE_MASK                              = 0x1UL << SPI_CR1_BIT_CONTROLLER_MODE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_CONTROLLER_MODE                                   = SPI_CR1_BIT_CONTROLLER_MODE_MASK;

static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_POSITION                                = (3U);
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_MASK                                    = (0x7UL << SPI_CR1_BIT_BAUD_RATE_POSITION);
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE                                         = SPI_CR1_BIT_BAUD_RATE_MASK;
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_0                                       = (0x1UL << SPI_CR1_BIT_BAUD_RATE_POSITION);
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_1                                       = (0x2UL << SPI_CR1_BIT_BAUD_RATE_POSITION);
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_2                                       = (0x4UL << SPI_CR1_BIT_BAUD_RATE_POSITION);

static constexpr uint32_t SPI_CR1_BIT_SPI_ENABLE_POSITION                               = 6U;
static constexpr uint32_t SPI_CR1_BIT_SPI_ENABLE_MASK                                   = 0x1UL << SPI_CR1_BIT_SPI_ENABLE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_SPI_ENABLE                                        = SPI_CR1_BIT_SPI_ENABLE_MASK;

static constexpr uint32_t SPI_CR1_BIT_LSB_FIRST_POSITION                                = 7U;
static constexpr uint32_t SPI_CR1_BIT_LSB_FIRST_MASK                                    = 0x1UL << SPI_CR1_BIT_LSB_FIRST_POSITION;
static constexpr uint32_t SPI_CR1_BIT_LSB_FIRST                                         = SPI_CR1_BIT_LSB_FIRST_MASK;

static constexpr uint32_t SPI_CR1_BIT_INTERNAL_CHIP_SELECT_POSITION                     = 8U;
static constexpr uint32_t SPI_CR1_BIT_INTERNAL_CHIP_SELECT_MASK                         = 0x1UL << SPI_CR1_BIT_INTERNAL_CHIP_SELECT_POSITION;
static constexpr uint32_t SPI_CR1_BIT_INTERNAL_CHIP_SELECT                              = SPI_CR1_BIT_INTERNAL_CHIP_SELECT_MASK;

static constexpr uint32_t SPI_CR1_BIT_SOFTWARE_CHIP_SELECT_POSITION                     = 9U;
static constexpr uint32_t SPI_CR1_BIT_SOFTWARE_CHIP_SELECT_MASK                         = 0x1UL << SPI_CR1_BIT_SOFTWARE_CHIP_SELECT_POSITION;
static constexpr uint32_t SPI_CR1_BIT_SOFTWARE_CHIP_SELECT                              = SPI_CR1_BIT_SOFTWARE_CHIP_SELECT_MASK;

static constexpr uint32_t SPI_CR1_BIT_RECEIVE_ONLY_POSITION                             = 10U;
static constexpr uint32_t SPI_CR1_BIT_RECEIVE_ONLY_MASK                                 = 0x1UL << SPI_CR1_BIT_RECEIVE_ONLY_POSITION;
static constexpr uint32_t SPI_CR1_BIT_RECEIVE_ONLY                                      = SPI_CR1_BIT_RECEIVE_ONLY_MASK;

static constexpr uint32_t SPI_CR1_BIT_DATA_FRAME_FORMAT_POSITION                        = 11U;
static constexpr uint32_t SPI_CR1_BIT_DATA_FRAME_FORMAT_MASK                            = 0x1UL << SPI_CR1_BIT_DATA_FRAME_FORMAT_POSITION;
static constexpr uint32_t SPI_CR1_BIT_DATA_FRAME_FORMAT                                 = SPI_CR1_BIT_DATA_FRAME_FORMAT_MASK;

static constexpr uint32_t SPI_CR1_BIT_SEND_CRC_NEXT_POSITION                            = 12U;
static constexpr uint32_t SPI_CR1_BIT_SEND_CRC_NEXT_MASK                                = 0x1UL << SPI_CR1_BIT_SEND_CRC_NEXT_POSITION;
static constexpr uint32_t SPI_CR1_BIT_SEND_CRC_NEXT                                     = SPI_CR1_BIT_SEND_CRC_NEXT_MASK;

static constexpr uint32_t SPI_CR1_BIT_CRC_ENABLE_POSITION                               = 13U;
static constexpr uint32_t SPI_CR1_BIT_CRC_ENABLE_MASK                                   = 0x1UL << SPI_CR1_BIT_CRC_ENABLE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_CRC_ENABLE                                        = SPI_CR1_BIT_CRC_ENABLE_MASK;

static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE_POSITION              = 14U;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE_MASK                  = 0x1UL << SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE                       = SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE_MASK;

static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_MODE_POSITION                       = 15U;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_MODE_MASK                           = 0x1UL << SPI_CR1_BIT_BIDIRECTIONAL_MODE_POSITION;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_MODE                                = SPI_CR1_BIT_BIDIRECTIONAL_MODE_MASK;


// spi cr2 definitions
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE_POSITION                     = 0U;
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE_MASK                         = 0x1UL << SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE                              = SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE_POSITION                     = 1U;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE_MASK                         = 0x1UL << SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE                              = SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE_POSITION                = 2U;
static constexpr uint32_t SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE_MASK                    = 0x1UL << SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE                         = SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_BIT_FRAME_FORMAT_POSITION                             = 4U;
static constexpr uint32_t SPI_CR2_BIT_FRAME_FORMAT_MASK                                 = 0x1UL << SPI_CR2_BIT_FRAME_FORMAT_POSITION;
static constexpr uint32_t SPI_CR2_BIT_FRAME_FORMAT                                      = SPI_CR2_BIT_FRAME_FORMAT_MASK;

static constexpr uint32_t SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE_POSITION                   = 5U;
static constexpr uint32_t SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE_MASK                       = 0x1UL << SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE                            = SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION     = 6U;
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK         = 0x1UL << SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE              = SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION         = 7U;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK             = 0x1UL << SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE                  = SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK;

static constexpr uint32_t SPI_CR2_RX_BUFFER_DMA_ENABLE                                  = SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE;
static constexpr uint32_t SPI_CR2_TX_BUFFER_DMA_ENABLE                                  = SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE;
static constexpr uint32_t SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE                             = SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE;
static constexpr uint32_t SPI_CR2_FRAME_FORMAT                                          = SPI_CR2_BIT_FRAME_FORMAT;


// spi sr definitions
static constexpr uint32_t SPI_SR_BIT_RX_BUFFER_NOT_EMPTY_POSITION                       = 0U;
static constexpr uint32_t SPI_SR_BIT_RX_BUFFER_NOT_EMPTY_MASK                           = 0x1UL << SPI_SR_BIT_RX_BUFFER_NOT_EMPTY_POSITION;
static constexpr uint32_t SPI_SR_BIT_RX_BUFFER_NOT_EMPTY                                = SPI_SR_BIT_RX_BUFFER_NOT_EMPTY_MASK;

static constexpr uint32_t SPI_SR_BIT_TX_BUFFER_EMPTY_POSITION                           = 1U;
static constexpr uint32_t SPI_SR_BIT_TX_BUFFER_EMPTY_MASK                               = 0x1UL << SPI_SR_BIT_TX_BUFFER_EMPTY_POSITION;
static constexpr uint32_t SPI_SR_BIT_TX_BUFFER_EMPTY                                    = SPI_SR_BIT_TX_BUFFER_EMPTY_MASK;

static constexpr uint32_t SPI_SR_BIT_CHANNEL_SIZE_POSITION                              = 2U;
static constexpr uint32_t SPI_SR_BIT_CHANNEL_SIZE_MASK                                  = 0x1UL << SPI_SR_BIT_CHANNEL_SIZE_POSITION;
static constexpr uint32_t SPI_SR_BIT_CHANNEL_SIZE                                       = SPI_SR_BIT_CHANNEL_SIZE_MASK;

static constexpr uint32_t SPI_SR_BIT_UNDERRUN_FLAG_POSITION                             = 3U;
static constexpr uint32_t SPI_SR_BIT_UNDERRUN_FLAG_MASK                                 = 0x1UL << SPI_SR_BIT_UNDERRUN_FLAG_POSITION;
static constexpr uint32_t SPI_SR_BIT_UNDERRUN_FLAG                                      = SPI_SR_BIT_UNDERRUN_FLAG_MASK;

static constexpr uint32_t SPI_SR_BIT_CRC_ERROR_POSITION                                 = 4U;
static constexpr uint32_t SPI_SR_BIT_CRC_ERROR_MASK                                     = 0x1UL << SPI_SR_BIT_CRC_ERROR_POSITION;
static constexpr uint32_t SPI_SR_BIT_CRC_ERROR                                          = SPI_SR_BIT_CRC_ERROR_MASK;

static constexpr uint32_t SPI_SR_BIT_MODE_FAULT_POSITION                                = 5U;
static constexpr uint32_t SPI_SR_BIT_MODE_FAULT_MASK                                    = 0x1UL << SPI_SR_BIT_MODE_FAULT_POSITION;
static constexpr uint32_t SPI_SR_BIT_MODE_FAULT                                         = SPI_SR_BIT_MODE_FAULT_MASK;

static constexpr uint32_t SPI_SR_BIT_OVERRUN_POSITION                                   = 6U;
static constexpr uint32_t SPI_SR_BIT_OVERRUN_MASK                                       = 0x1UL << SPI_SR_BIT_OVERRUN_POSITION;
static constexpr uint32_t SPI_SR_BIT_OVERRUN                                            = SPI_SR_BIT_OVERRUN_MASK;

static constexpr uint32_t SPI_SR_BIT_RESOURCE_BUSY_POSITION                             = 7U;
static constexpr uint32_t SPI_SR_BIT_RESOURCE_BUSY_MASK                                 = 0x1UL << SPI_SR_BIT_RESOURCE_BUSY_POSITION;
static constexpr uint32_t SPI_SR_BIT_RESOURCE_BUSY                                      = SPI_SR_BIT_RESOURCE_BUSY_MASK;

static constexpr uint32_t SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR_POSITION                = 8U;
static constexpr uint32_t SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR_MASK                    = 0x1UL << SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR_POSITION;
static constexpr uint32_t SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR                         = SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR_MASK;

static constexpr uint32_t   SPI_SR_BITS_MASK                                            = (SPI_SR_BIT_RX_BUFFER_NOT_EMPTY | SPI_SR_BIT_TX_BUFFER_EMPTY | SPI_SR_BIT_RESOURCE_BUSY
                                                                                            | SPI_SR_BIT_CRC_ERROR | SPI_SR_BIT_MODE_FAULT | SPI_SR_BIT_OVERRUN
                                                                                            | SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR);

static constexpr uint32_t SPI_DATA_REG_POSITION                                         = 0U;
static constexpr uint32_t SPI_DATA_REG_MASK                                             = 0xFFFFUL << SPI_DATA_REG_POSITION;
static constexpr uint32_t SPI_DATA_REG                                                  = SPI_DATA_REG_MASK;

static constexpr uint32_t SPI_CRC_POLYNOMIAL_REG_POSITION                               = 0U;
static constexpr uint32_t SPI_CRC_POLYNOMIAL_REG_MASK                                   = 0xFFFFUL << SPI_CRC_POLYNOMIAL_REG_POSITION;
static constexpr uint32_t SPI_CRC_POLYNOMIAL_REG                                        = SPI_CRC_POLYNOMIAL_REG_MASK;

static constexpr uint32_t SPI_DMA_CONFIG_REG_BIT_ENABLE_POSITION                        = 0U;
static constexpr uint32_t SPI_DMA_CONFIG_REG_BIT_ENABLE_MASK                            = 0x1UL << SPI_DMA_CONFIG_REG_BIT_ENABLE_POSITION;
static constexpr uint32_t SPI_DMA_CONFIG_REG_BIT_ENABLE                                 = SPI_DMA_CONFIG_REG_BIT_ENABLE_MASK;

static constexpr uint32_t SPI_DMA_ERROR_NONE                                            = 0x00000000U;
static constexpr uint32_t SPI_DMA_ERROR_TRANSFER                                        = 0x00000001U;
static constexpr uint32_t SPI_DMA_ERROR_FIFO                                            = 0x00000002U;
static constexpr uint32_t SPI_DMA_ERROR_DIRECT_MODE                                     = 0x00000004U;
static constexpr uint32_t SPI_DMA_ERROR_TIMEOUT                                         = 0x00000020U;
static constexpr uint32_t SPI_DMA_ERROR_PARAMETER                                       = 0x00000040U;
static constexpr uint32_t SPI_DMA_ERROR_NO_TRANSFER                                     = 0x00000080U;
static constexpr uint32_t SPI_DMA_ERROR_NOT_SUPPORTED                                   = 0x00000100U;

static constexpr uint32_t SPI_CONFIG_MODE_PERIPHERAL                                    = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_MODE_CONTROLLER                                    = (SPI_CR1_BIT_CONTROLLER_MODE | SPI_CR1_BIT_INTERNAL_CHIP_SELECT);
static constexpr uint32_t SPI_CONFIG_DIRECTION_2_LINE                                   = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_DIRECTION_2_LINE_RX_ONLY                           = SPI_CR1_BIT_RECEIVE_ONLY;
static constexpr uint32_t SPI_CONFIG_DIRECTION_1_LINE                                   = SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE;
static constexpr uint32_t SPI_CONFIG_DATA_SIZE_8_BIT                                    = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_DATA_SIZE_16_BIT                                   = SPI_CR1_BIT_DATA_FRAME_FORMAT;
static constexpr uint32_t SPI_CONFIG_DATA_MSB_FIRST                                     = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_DATA_LSB_FIRST                                     = SPI_CR1_BIT_LSB_FIRST;
static constexpr uint32_t SPI_CONFIG_CLOCK_POLARITY_LOW                                 = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_CLOCK_POLARITY_HIGH                                = SPI_CR1_BIT_CLOCK_POLARITY;
static constexpr uint32_t SPI_CONFIG_CLOCK_PHASE_LEADING_EDGE                           = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_CLOCK_PHASE_TRAILING_EDGE                          = SPI_CR1_BIT_CLOCK_PHASE;
static constexpr uint32_t SPI_CONFIG_CHIP_SELECT_SOFTWARE                               = SPI_CR1_BIT_SOFTWARE_CHIP_SELECT;
static constexpr uint32_t SPI_CONFIG_CHIP_SELECT_HARDWARE_INPUT                         = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_CHIP_SELECT_HARDWARE_OUTPUT                        = (SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE << 16U);

static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_2                              = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_4                              = (SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_8                              = (SPI_CR1_BIT_BAUD_RATE_1);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_16                             = (SPI_CR1_BIT_BAUD_RATE_1 | SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_32                             = (SPI_CR1_BIT_BAUD_RATE_2);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_64                             = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_128                            = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_1);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_256                            = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_1 | SPI_CR1_BIT_BAUD_RATE_0);

static constexpr uint32_t SPI_CONFIG_TI_MODE_ENABLE                                     = SPI_CR2_BIT_FRAME_FORMAT;
static constexpr uint32_t SPI_CONFIG_TX_BUFFER_EMPTY_INTERRUPT_ENABLE                   = SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE;
static constexpr uint32_t SPI_CONFIG_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE               = SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE;
static constexpr uint32_t SPI_CONFIG_ERROR_INTERRUPT_ENABLE                             = SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE;


static constexpr uint8_t  SPI_ERROR_NONE                                                = (0x00000000U);
static constexpr uint8_t  SPI_ERROR_MODE_FAULT                                          = (0x00000001U);
static constexpr uint8_t  SPI_ERROR_DURING_CRC_CALCULATION                              = (0x00000002U);
static constexpr uint8_t  SPI_ERROR_OVERRUN                                             = (0x00000004U);
static constexpr uint8_t  SPI_ERROR_TI_MODE_FRAME_FORMAT                                = (0x00000008U);
static constexpr uint8_t  SPI_ERROR_DMA_TRANSFER                                        = (0x00000010U);
static constexpr uint8_t  SPI_ERROR_WAITING_FOR_FLAG                                    = (0x00000020U);
static constexpr uint8_t  SPI_ERROR_DURING_ABORT                                        = (0x00000040U);
static constexpr uint8_t  SPI_ERROR_CALLBACK_INVALID                                    = (0x00000080U);

static constexpr uint8_t  SPI_PROCEDURE_ERROR_NONE                                      = 0U;
static constexpr uint8_t  SPI_PROCEDURE_STATE_BUS_ERROR                                 = 1U;
static constexpr uint8_t  SPI_PROCEDURE_STATE_DATA_ERROR                                = 2U;

static constexpr uint8_t  SPI_INIT_PROTOCOL_ERROR                                       = 0U;
static constexpr uint8_t  SPI_INIT_REGISTER_CALLBACKS_ERROR                             = 1U;
static constexpr uint8_t  SPI_INIT_DATA_STRUCTURES_ERROR                                = 2U;
static constexpr uint8_t  SPI_INIT_RESET_CHIP_SELECTS_ERROR                             = 3U;

static constexpr uint32_t SPI_DEFAULT_TIMEOUT_100_US                                    = 100U;
static constexpr uint32_t SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US                     = 1000U;
static constexpr uint32_t SPI_MAX_TIMEOUT                                               = (0xFFFFFFFFU);

static constexpr uint32_t SPI_TI_MODE_DISABLE                                           = (0x00000000U);
static constexpr uint32_t SPI_CRC_CALCULATION_DISABLE                                   = (0x00000000U);

static constexpr uint16_t SPI_CRC_POLYNOMIAL_MIN                                        = (0x0001U);
static constexpr uint16_t SPI_CRC_POLYNOMIAL_MAX                                        = (0xFFFFU);

static constexpr uint32_t SPI_REGISTER_CALLBACK_COUNT                                   = 10U;
static constexpr uint32_t SPI_REGISTER_CALLBACK_MIN_ID                                  = 0U;
static constexpr uint32_t SPI_REGISTER_CALLBACK_MAX_ID                                  = 9U;

static constexpr uint8_t  ACTIVE_LOW                                                    = 0U;
static constexpr uint8_t  CHIP_SELECT_LOGIC_LEVEL                                       = ACTIVE_LOW;
static constexpr uint8_t  CHIP_SELECT_SET                                               = CHIP_SELECT_LOGIC_LEVEL;
static constexpr uint8_t  CHIP_SELECT_RESET                                             = !CHIP_SELECT_LOGIC_LEVEL;

static constexpr uint8_t  SPI_TRANSACTION_NOT_IN_PROGRESS                               = 0U;
static constexpr uint8_t  SPI_TRANSACTION_IN_PROGRESS                                   = 1U;
static constexpr uint8_t  SPI_TRANSACTION_COMPLETE                                      = 2U;

static constexpr uint8_t  SPI_BYTE_COUNT_MAX                                            = 8U;

static constexpr uint8_t  CHANNEL_0                                                     = 0U;
static constexpr uint8_t  CHANNEL_1                                                     = 1U;
static constexpr uint8_t  CHANNEL_2                                                     = 2U;
static constexpr uint8_t  CHANNEL_3                                                     = 3U;
static constexpr uint8_t  CHANNEL_4                                                     = 4U;
static constexpr uint8_t  CHANNEL_5                                                     = 5U;
static constexpr uint8_t  CHANNEL_6                                                     = 6U;
static constexpr uint8_t  CHANNEL_7                                                     = 7U;

static constexpr uint8_t  SPI_USER_CHANNELS_MAX                                         = 8U;

#endif //MAIN_CONTROLLER_HAL_SPI_DEFINITIONS_H
