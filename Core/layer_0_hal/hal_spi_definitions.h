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


static constexpr uint32_t SPI_CR1_CRC_DISABLE                       = 0x00000000U;



// control register 2 (CR2)


static constexpr uint32_t SPI_CR1_CLOCK_PHASE_BIT_POSITION          = 0U;
static constexpr uint32_t SPI_CR1_CLOCK_PHASE_BIT_MASK              = 0x1UL << SPI_CR1_CLOCK_PHASE_BIT_POSITION;
static constexpr uint32_t SPI_CR1_CLOCK_PHASE_BIT                   = SPI_CR1_CLOCK_PHASE_BIT_MASK;

static constexpr uint32_t SPI_CR1_CLOCK_POLARITY_BIT_POSITION       = 1U;
static constexpr uint32_t SPI_CR1_CLOCK_POLARITY_BIT_MASK           = 0x1UL << SPI_CR1_CLOCK_POLARITY_BIT_POSITION;
static constexpr uint32_t SPI_CR1_CLOCK_POLARITY_BIT                = SPI_CR1_CLOCK_POLARITY_BIT_MASK;

static constexpr uint32_t SPI_CR1_CONTROLLER_MODE_BIT_POSITION      = 2U;
static constexpr uint32_t SPI_CR1_CONTROLLER_MODE_BIT_MASK          = 0x1UL << SPI_CR1_CONTROLLER_MODE_BIT_POSITION;
static constexpr uint32_t SPI_CR1_CONTROLLER_MODE_BIT               = SPI_CR1_CONTROLLER_MODE_BIT_MASK;

static constexpr uint32_t SPI_CR1_BAUD_RATE_BIT_POSITION            = (3U);
static constexpr uint32_t SPI_CR1_BAUD_RATE_BIT_MASK                = (0x7UL << SPI_CR1_BAUD_RATE_BIT_POSITION);
static constexpr uint32_t SPI_CR1_BAUD_RATE_BIT                     = SPI_CR1_BAUD_RATE_BIT_MASK;
static constexpr uint32_t SPI_CR1_BAUD_RATE_0_BIT                   = (0x1UL << SPI_CR1_BAUD_RATE_BIT_POSITION);
static constexpr uint32_t SPI_CR1_BAUD_RATE_1_BIT                   = (0x2UL << SPI_CR1_BAUD_RATE_BIT_POSITION);
static constexpr uint32_t SPI_CR1_BAUD_RATE_2_BIT                   = (0x4UL << SPI_CR1_BAUD_RATE_BIT_POSITION);


static constexpr uint32_t SPI_CR1_SPI_ENABLE_BIT_POSITION           = 6U;
static constexpr uint32_t SPI_CR1_SPI_ENABLE_BIT_MASK               = 0x1UL << SPI_CR1_SPI_ENABLE_BIT_POSITION;
static constexpr uint32_t SPI_CR1_SPI_ENABLE_BIT                    = SPI_CR1_SPI_ENABLE_BIT_MASK;

static constexpr uint32_t SPI_CR1_LSB_FIRST_BIT_POSITION            = 7U;
static constexpr uint32_t SPI_CR1_LSB_FIRST_BIT_MASK                = 0x1UL << SPI_CR1_LSB_FIRST_BIT_POSITION;
static constexpr uint32_t SPI_CR1_LSB_FIRST_BIT                     = SPI_CR1_LSB_FIRST_BIT_MASK;

static constexpr uint32_t SPI_CR1_INTERNAL_CHIP_SELECT_BIT_POSITION = 8U;
static constexpr uint32_t SPI_CR1_INTERNAL_CHIP_SELECT_BIT_MASK     = 0x1UL << SPI_CR1_INTERNAL_CHIP_SELECT_BIT_POSITION;
static constexpr uint32_t SPI_CR1_INTERNAL_CHIP_SELECT_BIT          = SPI_CR1_INTERNAL_CHIP_SELECT_BIT_MASK;

static constexpr uint32_t SPI_CR1_SOFTWARE_CHIP_SELECT_BIT_POSITION                     = 9U;
static constexpr uint32_t SPI_CR1_SOFTWARE_CHIP_SELECT_BIT_MASK                         = 0x1UL << SPI_CR1_SOFTWARE_CHIP_SELECT_BIT_POSITION;
static constexpr uint32_t SPI_CR1_SOFTWARE_CHIP_SELECT_BIT                          = SPI_CR1_SOFTWARE_CHIP_SELECT_BIT_MASK;

static constexpr uint32_t SPI_CR1_RECEIVE_ONLY_BIT_POSITION                             = 10U;
static constexpr uint32_t SPI_CR1_RECEIVE_ONLY_BIT_MASK                                 = 0x1UL << SPI_CR1_RECEIVE_ONLY_BIT_POSITION;
static constexpr uint32_t SPI_CR1_RECEIVE_ONLY_BIT                                      = SPI_CR1_RECEIVE_ONLY_BIT_MASK;

static constexpr uint32_t SPI_CR1_DATA_FRAME_FORMAT_BIT_POSITION                        = 11U;
static constexpr uint32_t SPI_CR1_DATA_FRAME_FORMAT_BIT_MASK                            = 0x1UL << SPI_CR1_DATA_FRAME_FORMAT_BIT_POSITION;
static constexpr uint32_t SPI_CR1_DATA_FRAME_FORMAT_BIT                                 = SPI_CR1_DATA_FRAME_FORMAT_BIT_MASK;

static constexpr uint32_t SPI_CR1_SEND_CRC_NEXT_BIT_POSITION                            = 12U;
static constexpr uint32_t SPI_CR1_SEND_CRC_NEXT_BIT_MASK                                = 0x1UL << SPI_CR1_SEND_CRC_NEXT_BIT_POSITION;
static constexpr uint32_t SPI_CR1_SEND_CRC_NEXT_BIT                                     = SPI_CR1_SEND_CRC_NEXT_BIT_MASK;

static constexpr uint32_t SPI_CR1_CRC_ENABLE_BIT_POSITION                               = 13U;
static constexpr uint32_t SPI_CR1_CRC_ENABLE_BIT_MASK                                   = 0x1UL << SPI_CR1_CRC_ENABLE_BIT_POSITION;
static constexpr uint32_t SPI_CR1_CRC_ENABLE_BIT                                        = SPI_CR1_CRC_ENABLE_BIT_MASK;

static constexpr uint32_t SPI_BIDIRECTIONAL_OUTPUT_ENABLE_BIT_POSITION                  = 14U;
static constexpr uint32_t SPI_BIDIRECTIONAL_OUTPUT_ENABLE_BIT_MASK                      = 0x1UL << SPI_BIDIRECTIONAL_OUTPUT_ENABLE_BIT_POSITION;
static constexpr uint32_t SPI_BIDIRECTIONAL_OUTPUT_ENABLE_BIT                           = SPI_BIDIRECTIONAL_OUTPUT_ENABLE_BIT_MASK;

static constexpr uint32_t SPI_CR1_BIDIRECTIONAL_MODE_BIT_POSITION                       = 15U;
static constexpr uint32_t SPI_CR1_BIDIRECTIONAL_MODE_BIT_MASK                           = 0x1UL << SPI_CR1_BIDIRECTIONAL_MODE_BIT_POSITION;
static constexpr uint32_t SPI_CR1_BIDIRECTIONAL_MODE_BIT                                = SPI_CR1_BIDIRECTIONAL_MODE_BIT_MASK;


//static constexpr uint32_t   SPI_CR1_CLOCK_PHASE                         = STM_HAL_SPI_CR1_CLOCK_PHASE;
//static constexpr uint32_t   SPI_CR1_CLOCK_POLARITY                      = STM_HAL_SPI_CR1_CLOCK_POLARITY;
//static constexpr uint32_t   SPI_CR1_MODE_CONTROLLER                     = STM_HAL_SPI_CR1_CONTROLLER;
//static constexpr uint32_t   SPI_CR1_BAUD_RATE_CONTROL_MASK              = STM_HAL_SPI_CR1_BAUD_RATE;
//static constexpr uint32_t   SPI_CR1_SPI_INSTANCE_ENABLE                 = STM_HAL_SPI_CR1_SPI_ENABLE;
//static constexpr uint32_t   SPI_CR1_LSB_FIRST                           = STM_HAL_SPI_CR1_LSB_FIRST;
//static constexpr uint32_t   SPI_CR1_INTERNAL_CHIP_SELECT                = STM_HAL_SPI_CR1_INTERNAL_CHIP_SELECT;
//static constexpr uint32_t   SPI_CR1_SOFTWARE_CHIP_SELECT                = STM_HAL_SPI_CR1_SOFTWARE_CHIP_SELECT;
//static constexpr uint32_t   SPI_CR1_RECEIVE_ONLY                        = STM_HAL_SPI_CR1_RECEIVE_ONLY;
//static constexpr uint32_t   SPI_CR1_DATA_FRAME_FORMAT                   = STM_HAL_SPI_CR1_DATA_FRAME_FORMAT;
//static constexpr uint32_t   SPI_CR1_SEND_CRC_NEXT                       = STM_HAL_SPI_CR1_SEND_CRC_NEXT;
//static constexpr uint32_t   SPI_CR1_CRC_ENABLE                          = STM_HAL_SPI_CR1_CRC_ENABLE;
//static constexpr uint32_t   SPI_CR1_CRC_DISABLE                         = STM_HAL_SPI_CR1_CRC_DISABLE;
//static constexpr uint32_t   SPI_CR1_BIDIRECTIONAL_MODE                  = STM_HAL_SPI_CR1_BIDIRECTIONAL_MODE;



/*************************************** spi control register 2 definitions *******************************************/
#define STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION                   (0U)
#define STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK                       0x1UL << STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_POSITION
#define STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE                            STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE_MASK
#define STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION                   (1U)
#define STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK                       0x1UL << STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_POSITION
#define STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE                            STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE_MASK
#define STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_POSITION              (2U)
#define STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_MASK                  (0x1UL << STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_POSITION)
#define STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE                       STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE_MASK
#define STM_HAL_SPI_CR2_FRAME_FORMAT_POSITION                           (4U)
#define STM_HAL_SPI_CR2_FRAME_FORMAT_MASK                               (0x1UL << STM_HAL_SPI_CR2_FRAME_FORMAT_POSITION)
#define STM_HAL_SPI_CR2_FRAME_FORMAT                                    STM_HAL_SPI_CR2_FRAME_FORMAT_MASK
#define STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_POSITION                 (5U)
#define STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_MASK                     (0x1UL << STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_POSITION)
#define STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE                          STM_HAL_SPI_CR2_ERROR_INTERRUPT_ENABLE_MASK
#define STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION   (6U)
#define STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK       (0x1UL << STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_POSITION)
#define STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE            STM_HAL_SPI_CR2_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE_MASK
#define STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION       (7U)
#define STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK           (0x1UL << STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_POSITION)
#define STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE                STM_HAL_SPI_CR2_TX_BUFFER_EMPTY_INTERRUPT_ENABLE_MASK

static constexpr uint32_t   SPI_CR2_RX_BUFFER_DMA_ENABLE                = STM_HAL_SPI_CR2_RX_BUFFER_DMA_ENABLE;
static constexpr uint32_t   SPI_CR2_TX_BUFFER_DMA_ENABLE                = STM_HAL_SPI_CR2_TX_BUFFER_DMA_ENABLE;
static constexpr uint32_t   SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE           = STM_HAL_SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE;
static constexpr uint32_t   SPI_CR2_FRAME_FORMAT                        = STM_HAL_SPI_CR2_FRAME_FORMAT;


/***************************************** spi status register definitions ********************************************/
#define STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_POSITION                     (0U)
#define STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_MASK                         (0x1UL << STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_POSITION)
#define STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY                              STM_HAL_SPI_SR_RX_BUFFER_NOT_EMPTY_MASK
#define STM_HAL_SPI_SR_TX_BUFFER_EMPTY_POSITION                         (1U)
#define STM_HAL_SPI_SR_TX_BUFFER_EMPTY_MASK                             (0x1UL << STM_HAL_SPI_SR_TX_BUFFER_EMPTY_POSITION)
#define STM_HAL_SPI_SR_TX_BUFFER_EMPTY                                  STM_HAL_SPI_SR_TX_BUFFER_EMPTY_MASK
#define STM_HAL_SPI_SR_CHANNEL_SIZE_POSITION                            (2U)
#define STM_HAL_SPI_SR_CHANNEL_SIZE_MASK                                (0x1UL << STM_HAL_SPI_SR_CHANNEL_SIZE_POSITION)
#define STM_HAL_SPI_SR_CHANNEL_SIZE                                     STM_HAL_SPI_SR_CHANNEL_SIZE_MASK
#define STM_HAL_SPI_SR_UNDERRUN_FLAG_POSITION                           (3U)
#define STM_HAL_SPI_SR_UNDERRUN_FLAG_MASK                               (0x1UL << STM_HAL_SPI_SR_UNDERRUN_FLAG_POSITION)
#define STM_HAL_SPI_SR_UNDERRUN_FLAG                                    STM_HAL_SPI_SR_UNDERRUN_FLAG_MASK
#define STM_HAL_SPI_SR_CRC_ERROR_POSITION                               (4U)
#define STM_HAL_SPI_SR_CRC_ERROR_MASK                                   (0x1UL << STM_HAL_SPI_SR_CRC_ERROR_POSITION)
#define STM_HAL_SPI_SR_CRC_ERROR                                        STM_HAL_SPI_SR_CRC_ERROR_MASK
#define STM_HAL_SPI_SR_MODE_FAULT_POSITION                              (5U)
#define STM_HAL_SPI_SR_MODE_FAULT_MASK                                  (0x1UL << STM_HAL_SPI_SR_MODE_FAULT_POSITION)
#define STM_HAL_SPI_SR_MODE_FAULT                                       STM_HAL_SPI_SR_MODE_FAULT_MASK
#define STM_HAL_SPI_SR_OVERRUN_POSITION                                 (6U)
#define STM_HAL_SPI_SR_OVERRUN_MASK                                     (0x1UL << STM_HAL_SPI_SR_OVERRUN_POSITION)
#define STM_HAL_SPI_SR_OVERRUN                                          STM_HAL_SPI_SR_OVERRUN_MASK
#define STM_HAL_SPI_SR_BUSY_POSITION                                    (7U)
#define STM_HAL_SPI_SR_BUSY_MASK                                        (0x1UL << STM_HAL_SPI_SR_BUSY_POSITION)
#define STM_HAL_SPI_SR_BUSY                                             STM_HAL_SPI_SR_BUSY_MASK
#define STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_POSITION              (8U)
#define STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_MASK                  (0x1UL << STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_POSITION)
#define STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR                       STM_HAL_SPI_SR_TI_MODE_FRAME_FORMAT_ERROR_MASK

#define STM_HAL_SPI_DATA_REG_POSITION                                   (0U)
#define STM_HAL_SPI_DATA_REG_MASK                                       (0xFFFFUL << STM_HAL_SPI_DATA_REG_POSITION)
#define STM_HAL_SPI_DATA_REG                                            STM_HAL_SPI_DATA_REG_MASK

#define STM_HAL_SPI_CRC_POLYNOMIAL_REG_POSITION                         (0U)
#define STM_HAL_SPI_CRC_POLYNOMIAL_REG_MASK                             (0xFFFFUL << STM_HAL_SPI_CRC_POLYNOMIAL_REG_POSITION)
#define STM_HAL_SPI_CRC_POLYNOMIAL_REG                                  STM_HAL_SPI_CRC_POLYNOMIAL_REG_MASK

/**********************************************************************************************************************/

#define STM_HAL_SPI_I2S_MODE_SELECT_POSITION                            (11U)
#define STM_HAL_SPI_I2S_MODE_SELECT_MASK                                (0x1UL << STM_HAL_SPI_I2S_MODE_SELECT_POSITION)
#define STM_HAL_SPI_I2S_MODE_SELECT                                     STM_HAL_SPI_I2S_MODE_SELECT_MASK

/**************************************** dma control register definitions ********************************************/

#define STM_HAL_DMA_SxCR_ENABLE_POSITION                                (0U)
#define STM_HAL_DMA_SxCR_ENABLE_MASK                                    (0x1UL << STM_HAL_DMA_SxCR_ENABLE_POSITION)
#define STM_HAL_DMA_SxCR_ENABLE                                         STM_HAL_DMA_SxCR_ENABLE_MASK

#define STM_HAL_DMA_ERROR_NONE            0x00000000U    /*!< No error                               */
#define STM_HAL_DMA_ERROR_TE              0x00000001U    /*!< Transfer error                         */
#define STM_HAL_DMA_ERROR_FE              0x00000002U    /*!< FIFO error                             */
#define STM_HAL_DMA_ERROR_DME             0x00000004U    /*!< Direct Mode error                      */
#define STM_HAL_DMA_ERROR_TIMEOUT         0x00000020U    /*!< Timeout error                          */
#define STM_HAL_DMA_ERROR_PARAM           0x00000040U    /*!< Parameter error                        */
#define SPI_DMA_ERROR_NO_TRANSFER         0x00000080U    /*!< Abort requested with no Xfer ongoing   */
#define STM_HAL_DMA_ERROR_NOT_SUPPORTED   0x00000100U    /*!< Not supported mode                     */



#endif //MAIN_CONTROLLER_HAL_SPI_DEFINITIONS_H
