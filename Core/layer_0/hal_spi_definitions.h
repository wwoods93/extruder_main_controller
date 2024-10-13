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

/* layer_0 includes */

/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

#ifndef ID_INVALID
#define ID_INVALID (-1)
#endif

#ifndef UNUSED_CAST_VOID
#define UNUSED_CAST_VOID(X)                 (void) X
#endif

#ifndef PERIPHERAL_BASE_ADDRESS
#define PERIPHERAL_BASE_ADDRESS         0x40000000UL
#endif

#ifndef APB1_PERIPHERAL_BASE_ADDRESS
#define APB1_PERIPHERAL_BASE_ADDRESS    PERIPHERAL_BASE_ADDRESS
#endif

#ifndef APB2_PERIPHERAL_BASE_ADDRESS
#define APB2_PERIPHERAL_BASE_ADDRESS    (PERIPHERAL_BASE_ADDRESS + 0x00010000UL)
#endif

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

#define SPI_1_BASE_ADDRESS                  (APB2_PERIPHERAL_BASE_ADDRESS + 0x3000UL)
#define SPI_2_BASE_ADDRESS                  (APB1_PERIPHERAL_BASE_ADDRESS + 0x3800UL)
#define SPI_3_BASE_ADDRESS                  (APB1_PERIPHERAL_BASE_ADDRESS + 0x3C00UL)
#define SPI_4_BASE_ADDRESS                  (APB2_PERIPHERAL_BASE_ADDRESS + 0x3400UL)
#define SPI_1                               ((hal_spi_t *) SPI_1_BASE_ADDRESS)
#define SPI_2                               ((hal_spi_t *) SPI_2_BASE_ADDRESS)
#define SPI_3                               ((hal_spi_t *) SPI_3_BASE_ADDRESS)
#define SPI_4                               ((hal_spi_t *) SPI_4_BASE_ADDRESS)

static constexpr uint8_t SPI_1_ID = 1U;
static constexpr uint8_t SPI_2_ID = 2U;
static constexpr uint8_t SPI_3_ID = 3U;
static constexpr uint8_t SPI_4_ID = 4U;

// spi cr1 map
static constexpr uint32_t SPI_CR1_BIT_CLOCK_PHASE                                       = 0x00000001U;
static constexpr uint32_t SPI_CR1_BIT_CLOCK_POLARITY                                    = 0x00000002U;
static constexpr uint32_t SPI_CR1_BIT_CONTROLLER_MODE                                   = 0x00000004U;
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE                                         = 0x00000038U;
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_0                                       = 0x00000008U;
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_1                                       = 0x00000010U;
static constexpr uint32_t SPI_CR1_BIT_BAUD_RATE_2                                       = 0x00000020U;
static constexpr uint32_t SPI_CR1_BIT_SPI_ENABLE                                        = 0x00000040U;
static constexpr uint32_t SPI_CR1_BIT_LSB_FIRST                                         = 0x00000080U;
static constexpr uint32_t SPI_CR1_BIT_INTERNAL_CHIP_SELECT                              = 0x00000100U;
static constexpr uint32_t SPI_CR1_BIT_SOFTWARE_CHIP_SELECT                              = 0x00000200U;
static constexpr uint32_t SPI_CR1_BIT_RECEIVE_ONLY                                      = 0x00000400U;
static constexpr uint32_t SPI_CR1_BIT_DATA_FRAME_FORMAT                                 = 0x00000800U;
static constexpr uint32_t SPI_CR1_BIT_SEND_CRC_NEXT                                     = 0x00001000U;
static constexpr uint32_t SPI_CR1_BIT_CRC_ENABLE                                        = 0x00002000U;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_OUTPUT_ENABLE                       = 0x00004000U;
static constexpr uint32_t SPI_CR1_BIT_BIDIRECTIONAL_MODE                                = 0x00008000U;

// spi cr2 map
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_DMA_ENABLE                              = 0x00000001U;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_DMA_ENABLE                              = 0x00000002U;
static constexpr uint32_t SPI_CR2_BIT_CHIP_SELECT_OUTPUT_ENABLE                         = 0x00000004U;
static constexpr uint32_t SPI_CR2_BIT_FRAME_FORMAT                                      = 0x00000010U;
static constexpr uint32_t SPI_CR2_BIT_ERROR_INTERRUPT_ENABLE                            = 0x00000020U;
static constexpr uint32_t SPI_CR2_BIT_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE              = 0x00000040U;
static constexpr uint32_t SPI_CR2_BIT_TX_BUFFER_EMPTY_INTERRUPT_ENABLE                  = 0x00000080U;

// spi sr map
static constexpr uint32_t SPI_SR_BIT_RX_BUFFER_NOT_EMPTY                                = 0x00000001U;
static constexpr uint32_t SPI_SR_BIT_TX_BUFFER_EMPTY                                    = 0x00000002U;
// bit 2 channel side unused by spi
static constexpr uint32_t SPI_SR_BIT_UNDERRUN_FLAG                                      = 0x00000008U;
static constexpr uint32_t SPI_SR_BIT_CRC_ERROR                                          = 0x00000010U;
static constexpr uint32_t SPI_SR_BIT_MODE_FAULT                                         = 0x00000020U;
static constexpr uint32_t SPI_SR_BIT_OVERRUN                                            = 0x00000040U;
static constexpr uint32_t SPI_SR_BIT_RESOURCE_BUSY                                      = 0x00000080U;
static constexpr uint32_t SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR                         = 0x00000100U;

static constexpr uint32_t SPI_SR_BITS_MASK                                              = (SPI_SR_BIT_RX_BUFFER_NOT_EMPTY | SPI_SR_BIT_TX_BUFFER_EMPTY | SPI_SR_BIT_RESOURCE_BUSY
                                                                                            | SPI_SR_BIT_CRC_ERROR | SPI_SR_BIT_MODE_FAULT | SPI_SR_BIT_OVERRUN
                                                                                            | SPI_SR_BIT_TI_MODE_FRAME_FORMAT_ERROR);

static constexpr uint32_t SPI_DATA_REG                                                  = 0xFFFFUL;
static constexpr uint32_t SPI_CRC_POLYNOMIAL_REG                                        = 0xFFFFUL;

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
static constexpr uint32_t SPI_CONFIG_CRC_CALCULATION_DISABLE                            = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_TI_MODE_DISABLE                                    = (0x00000000U);

static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_2                              = (0x00000000U);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_4                              = (SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_8                              = (SPI_CR1_BIT_BAUD_RATE_1);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_16                             = (SPI_CR1_BIT_BAUD_RATE_1 | SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_32                             = (SPI_CR1_BIT_BAUD_RATE_2);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_64                             = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_0);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_128                            = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_1);
static constexpr uint32_t SPI_CONFIG_BAUD_RATE_PRESCALER_256                            = (SPI_CR1_BIT_BAUD_RATE_2 | SPI_CR1_BIT_BAUD_RATE_1 | SPI_CR1_BIT_BAUD_RATE_0);

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

static constexpr uint8_t  ACTIVE_LOW                                                    = 0U;
static constexpr uint8_t  ACTIVE_HIGH                                                   = 1U;

static constexpr uint8_t  CHIP_SELECT_LOGIC_LEVEL                                       = ACTIVE_LOW;
static constexpr uint8_t  CHIP_SELECT_SET                                               = CHIP_SELECT_LOGIC_LEVEL;
static constexpr uint8_t  CHIP_SELECT_RESET                                             = !CHIP_SELECT_SET;

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
static constexpr uint32_t SPI_REGISTER_CALLBACK_COUNT                                   = 8U;
static constexpr uint32_t SPI_REGISTER_CALLBACK_MIN_ID                                  = 0U;
static constexpr uint32_t SPI_REGISTER_CALLBACK_MAX_ID                                  = 7U;

#endif //MAIN_CONTROLLER_HAL_SPI_DEFINITIONS_H
