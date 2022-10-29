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

//
// Created by wilson on 10/21/22.
//

#ifndef MAIN_CONTROLLER_HAL_GENERAL_H
#define MAIN_CONTROLLER_HAL_GENERAL_H

#include <cstdint>

#define HAL_GENERAL_SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define HAL_GENERAL_CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define HAL_GENERAL_READ_REG(REG)         ((REG))
#define HAL_GENERAL_UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#define HAL_GENERAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
#define HAL_GENERAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define     _IO_    volatile

#if (USE_RTOS == 1U)
/* Reserved for future use */
  #error "USE_RTOS should be 0 in the current HAL release"
#else
    #define HAL_LOCK_MODULE(__HANDLE__)                                                             \
                                    do                                                              \
                                    {                                                               \
                                        if((__HANDLE__)->lock == HAL_MODULE_LOCKED) { return HAL_BUSY; }   \
                                        else { (__HANDLE__)->lock = HAL_MODULE_LOCKED; }                   \
                                    } while (0U)                                                    \

    #define HAL_UNLOCK_MODULE(__HANDLE__) do { (__HANDLE__)->lock = HAL_MODULE_UNLOCKED; } while (0U)

typedef enum
{
    HAL_MODULE_UNLOCKED = 0x00U,
    HAL_MODULE_LOCKED   = 0x01U
} hal_lock_t;


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
    volatile uint32_t CONTROL_REG_1;        /*   CR1     !< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
    volatile uint32_t CONTROL_REG_2;        /*   CR2     !< SPI control register 2,                             Address offset: 0x04 */
    volatile uint32_t STATUS_REG;         /*   SR     !< SPI status register,                                Address offset: 0x08 */
    volatile uint32_t DATA_REG;         /*   DR     !< SPI data register,                                  Address offset: 0x0C */
    volatile uint32_t CRC_POLYNOMIAL_REG;      /*   CRCPR     !< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
    volatile uint32_t CRC_RX_REG;     /*   RXCRCR    !< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
    volatile uint32_t CRC_TX_REG;     /*   TXCRCR     !< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
    volatile uint32_t I2S_CONFIG_REG;    /*   I2SCFGR     !< SPI_I2S configuration register,                     Address offset: 0x1C */
    volatile uint32_t I2S_PRESCALER_REG;      /*   I2SPR     !< SPI_I2S prescaler register,                         Address offset: 0x20 */
} hal_spi_t;

#define PERIPHERAL_BASE_ADDRESS             0x40000000UL
#define APB1_PERIPHERAL_BASE_ADDRESS        PERIPHERAL_BASE_ADDRESS
#define SPI_2_BASE_ADDRESS                   (APB1_PERIPHERAL_BASE_ADDRESS + 0x3800UL)
#define SPI_2                ((hal_spi_t *) SPI_2_BASE_ADDRESS)

#endif /* USE_RTOS */

#endif //MAIN_CONTROLLER_HAL_GENERAL_H
