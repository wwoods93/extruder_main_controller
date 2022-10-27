/***********************************************************************************************************************
 * Main_Controller
 * hal_spi.h
 *
 * wilson
 * 10/16/22
 * 9:41 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_SPI_H
#define MAIN_CONTROLLER_HAL_SPI_H

#include "stm32f4xx.h"

//typedef enum
//{
//    SPI_PROCEDURE_STATE_VERIFY_DATA = 0,
//    SPI_PROCEDURE_STATE_PREPARE_BUS,
//    SPI_PROCEDURE_STATE_SET_TRANSACTION_PARAMETERS,
//    SPI_PROCEDURE_STATE_ENABLE,
//    SPI_PROCEDURE_STATE_BUS_ERROR,
//    SPI_PROCEDURE_STATE_DATA_ERROR,
//    SPI_PROCEDURE_STATE_SUCCESSFUL_TRANSMIT_RECEIVE_REQUEST
//} CONTROLLER_TRANSFER_RECEIVE_STATES;
//
//typedef struct
//{
//    CONTROLLER_TRANSFER_RECEIVE_STATES state;
//} CONTROLLER_TRANSFER_RECEIVE_PROCEDURE;

//CONTROLLER_TRANSFER_RECEIVE_PROCEDURE spi_transfer_receive_procedure;


class spi
{
    public:

        #define HAL_SPI_USE_REGISTER_CALLBACKS                                  1U
        static constexpr uint8_t SPI_PROCEDURE_ERROR_NONE                       = 0U;
        static constexpr uint8_t SPI_PROCEDURE_STATE_BUS_ERROR                  = 1U;
        static constexpr uint8_t SPI_PROCEDURE_STATE_DATA_ERROR                 = 2U;

#define SPI_CR1_MODE_CONTROLLER             SPI_CR1_MSTR
#define SPI_CR1_INTERNAL_CHIP_SELECT        SPI_CR1_SSI
#define SPI_CR1_RECEIVE_ONLY                SPI_CR1_RXONLY
#define SPI_CR1_BIDIRECTIONAL_MODE          SPI_CR1_BIDIMODE
#define SPI_CR1_DATA_FRAME_FORMAT           SPI_CR1_DFF
#define SPI_CR1_CLOCK_POLARITY              SPI_CR1_CPOL
#define SPI_CR1_CLOCK_PHASE                 SPI_CR1_CPHA
#define SPI_CR1_SOFTWARE_CHIP_SELECT        SPI_CR1_SSM
#define SPI_CR1_BAUD_RATE_CONTROL_MASK      SPI_CR1_BR_Msk
#define SPI_CR1_LSB_FIRST                   SPI_CR1_LSBFIRST
#define SPI_CR1_CRC_ENABLE                  SPI_CR1_CRCEN

#define SPI_CR2_CHIP_SELECT_OUTPUT_ENABLE   SPI_CR2_SSOE
#define SPI_CR2_FRAME_FORMAT                SPI_CR2_FRF

        #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
            static constexpr uint8_t SPI_ERROR_NONE                             = (0x00000000U);
            static constexpr uint8_t SPI_ERROR_MODE_FAULT                       = (0x00000001U);
            static constexpr uint8_t SPI_ERROR_DURING_CRC_CALCULATION           = (0x00000002U);
            static constexpr uint8_t SPI_ERROR_OVERRUN                          = (0x00000004U);
            static constexpr uint8_t SPI_ERROR_TI_MODE_FRAME_FORMAT             = (0x00000008U);
            static constexpr uint8_t SPI_ERROR_DMA_TRANSFER                     = (0x00000010U);
            static constexpr uint8_t SPI_ERROR_WAITING_FOR_FLAG                 = (0x00000020U);
            static constexpr uint8_t SPI_ERROR_DURING_ABORT                     = (0x00000040U);
            static constexpr uint8_t SPI_ERROR_CALLBACK_INVALID                 = (0x00000080U);
        #endif

        static constexpr uint32_t SPI_FLAG_RX_BUFFER_NOT_EMPTY                  = SPI_SR_RXNE;
        static constexpr uint32_t SPI_FLAG_TX_BUFFER_EMPTY                      = SPI_SR_TXE;
        static constexpr uint32_t SPI_FLAG_BUSY                                 = SPI_SR_BSY;
        static constexpr uint32_t SPI_FLAG_CRC_ERROR                            = SPI_SR_CRCERR;
        static constexpr uint32_t SPI_FLAG_MODE_FAULT                           = SPI_SR_MODF;
        static constexpr uint32_t SPI_FLAG_OVERRUN                              = SPI_SR_OVR;
        static constexpr uint32_t SPI_FLAG_TI_MODE_FRAME_FORMAT_ERROR           = SPI_SR_FRE;
        static constexpr uint32_t SPI_FLAG_BIT_MASK                             = (SPI_SR_RXNE | SPI_SR_TXE | SPI_SR_BSY | SPI_SR_CRCERR  \
                                                                                 | SPI_SR_MODF | SPI_SR_OVR | SPI_SR_FRE);

        static constexpr uint32_t SPI_MODE_PERIPHERAL                           = (0x00000000U);
        static constexpr uint32_t SPI_MODE_CONTROLLER                           = (SPI_CR1_MSTR | SPI_CR1_SSI);
        static constexpr uint32_t SPI_DIRECTION_2_LINE                          = (0x00000000U);
        static constexpr uint32_t SPI_DIRECTION_2_LINE_RX_ONLY                  = SPI_CR1_RXONLY;
        static constexpr uint32_t SPI_DIRECTION_1_LINE                          = SPI_CR1_BIDIMODE;
        static constexpr uint32_t SPI_DATA_SIZE_8_BIT                           = (0x00000000U);
        static constexpr uint32_t SPI_DATA_SIZE_16_BIT                          = SPI_CR1_DFF;
        static constexpr uint32_t SPI_DATA_MSB_FIRST                            = (0x00000000U);
        static constexpr uint32_t SPI_DATA_LSB_FIRST                            = SPI_CR1_LSBFIRST;
        static constexpr uint32_t SPI_CLOCK_POLARITY_LOW                        = (0x00000000U);
        static constexpr uint32_t SPI_CLOCK_POLARITY_HIGH                       = SPI_CR1_CPOL;
        static constexpr uint32_t SPI_CLOCK_PHASE_LEADING_EDGE                  = (0x00000000U);
        static constexpr uint32_t SPI_CLOCK_PHASE_TRAILING_EDGE                 = SPI_CR1_CPHA;
        static constexpr uint32_t SPI_CHIP_SELECT_SOFTWARE                      = SPI_CR1_SSM;
        static constexpr uint32_t SPI_CHIP_SELECT_HARDWARE_INPUT                = (0x00000000U);
        static constexpr uint32_t SPI_CHIP_SELECT_HARDWARE_OUTPUT               = (SPI_CR2_SSOE << 16U);

        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_2                     = (0x00000000U);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_4                     = (SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_8                     = (SPI_CR1_BR_1);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_16                    = (SPI_CR1_BR_1 | SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_32                    = (SPI_CR1_BR_2);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_64                    = (SPI_CR1_BR_2 | SPI_CR1_BR_0);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_128                   = (SPI_CR1_BR_2 | SPI_CR1_BR_1);
        static constexpr uint32_t SPI_BAUD_RATE_PRESCALER_256                   = (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);

        static constexpr uint32_t SPI_TI_MODE_DISABLE                           = (0x00000000U);
        static constexpr uint32_t SPI_TI_MODE_ENABLE                            = SPI_CR2_FRF;
        static constexpr uint32_t SPI_TX_BUFFER_EMPTY_INTERRUPT_ENABLE          = SPI_CR2_TXEIE;
        static constexpr uint32_t SPI_RX_BUFFER_NOT_EMPTY_INTERRUPT_ENABLE      = SPI_CR2_RXNEIE;
        static constexpr uint32_t SPI_ERROR_INTERRUPT_ENABLE                    = SPI_CR2_ERRIE;

        static constexpr uint32_t SPI_DEFAULT_TIMEOUT_100_US                    = 100U;
        static constexpr uint32_t SPI_BUSY_FLAG_WORK_AROUND_TIMEOUT_1000_US     = 1000U;

        static constexpr uint32_t SPI_CRC_CALCULATION_DISABLE                   = (0x00000000U);
        static constexpr uint32_t SPI_CRC_CALCULATION_ENABLE                    = SPI_CR1_CRCEN;



        #define SPI_ENABLE_MODULE(__HANDLE__)  HAL_GENERAL_SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)
        #define SPI_DISABLE_MODULE(__HANDLE__) HAL_GENERAL_CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)
        #define SPI_ENABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)   HAL_GENERAL_SET_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))
        #define SPI_DISABLE_INTERRUPTS(__HANDLE__, __INTERRUPT__)  HAL_GENERAL_CLEAR_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))
        #define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))
        #define SPI_CHECK_FLAG(__SR__, __FLAG__) ((((__SR__) & ((__FLAG__) & SPI_FLAG_MASK)) == ((__FLAG__) & SPI_FLAG_MASK)) ? SET : RESET)
        #define SPI_VERIFY_DIRECTION_2_LINE(__MODE__) ((__MODE__) == SPI_DIRECTION_2_LINE)
        #define SPI_VERIFY_DIRECTION_2_LINE_RX_ONLY(__MODE__) ((__MODE__) == SPI_DIRECTION_2_LINE_RX_ONLY)
        #define SPI_VERIFY_DIRECTION_1_LINE(__MODE__) ((__MODE__) == SPI_DIRECTION_1_LINE)
        #define SPI_CLEAR_OVERRUN_FLAG(__HANDLE__)                                  \
            do{                                                                     \
                __IO uint32_t tmpreg_ovr = 0x00U;                                   \
                tmpreg_ovr = (__HANDLE__)->Instance->DR;                            \
                tmpreg_ovr = (__HANDLE__)->Instance->SR;                            \
                HAL_GENERAL_UNUSED(tmpreg_ovr);                                     \
            }   while(0U)

#define SPI_CLEAR_MODE_FAULT_FLAG(__HANDLE__)                                       \
            do {                                                                    \
                __IO uint32_t tmpreg_modf = 0x00U;                                  \
                tmpreg_modf = (__HANDLE__)->Instance->SR;                           \
                HAL_GENERAL_CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE);    \
                HAL_GENERAL_UNUSED(tmpreg_modf);                                    \
            }   while(0U)

#define HAL_SPI_CLEAR_FORMAT_ERROR_FLAG(__HANDLE__)                                 \
            do {                                                                    \
                __IO uint32_t tmpreg_fre = 0x00U;                                   \
                tmpreg_fre = (__HANDLE__)->Instance->SR;                            \
                HAL_GENERAL_UNUSED(tmpreg_fre);                                     \
            }   while(0U)

#define SPI_CHECK_IT_SOURCE(__CR2__, __INTERRUPT__) ((((__CR2__) & (__INTERRUPT__)) ==  \
                                                     (__INTERRUPT__)) ? SET : RESET)

        typedef enum
        {
            SPI_STATUS_OK       = 0x00U,
            SPI_STATUS_ERROR    = 0x01U,
            SPI_STATUS_BUSY     = 0x02U,
            SPI_STATUS_TIMEOUT  = 0x03U
        } spi_status_t;

        typedef enum
        {
            SPI_TX_COMPLETE_CALLBACK_ID                 = 0x00U,    /*!< SPI Tx Completed callback ID         */
            SPI_RX_COMPLETE_CALLBACK_ID                 = 0x01U,    /*!< SPI Rx Completed callback ID         */
            SPI_TX_RX_COMPLETE_CALLBACK_ID              = 0x02U,    /*!< SPI TxRx Completed callback ID       */
            SPI_TX_HALF_COMPLETE_CALLBACK_ID            = 0x03U,    /*!< SPI Tx Half Completed callback ID    */
            SPI_RX_HALF_COMPLETE_CALLBACK_ID            = 0x04U,    /*!< SPI Rx Half Completed callback ID    */
            SPI_TX_RX_HALF_COMPLETE_CALLBACK_ID         = 0x05U,    /*!< SPI TxRx Half Completed callback ID  */
            SPI_ERROR_CALLBACK_ID                       = 0x06U,    /*!< SPI Error callback ID                */
            SPI_ABORT_CALLBACK_ID                       = 0x07U,    /*!< SPI Abort callback ID                */
            SPI_MSP_INIT_CALLBACK_ID                    = 0x08U,    /*!< SPI Msp Init callback ID             */
            SPI_MSP_DEINIT_CALLBACK_ID                  = 0x09U     /*!< SPI Msp DeInit callback ID           */
        } spi_callback_id_t;


        typedef struct __SPI_HandleTypeDef
        {
            SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

            SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

            uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

            uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

            __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

            uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

            uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

            __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

            void (*RxISR)(spi spi_object, struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR       */

            void (*TxISR)(spi spi_object, struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR       */

            DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

            DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

            HAL_LockTypeDef            Lock;           /*!< Locking object                           */

            __IO HAL_SPI_StateTypeDef  State;          /*!< SPI communication state                  */

            __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

            #if (USE_HAL_SPI_REGISTER_CALLBACKS == 1U)
                void (* TxCpltCallback)(struct __SPI_HandleTypeDef *hspi);             /*!< SPI Tx Completed callback          */
                void (* RxCpltCallback)(struct __SPI_HandleTypeDef *hspi);             /*!< SPI Rx Completed callback          */
                void (* TxRxCpltCallback)(struct __SPI_HandleTypeDef *hspi);           /*!< SPI TxRx Completed callback        */
                void (* TxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);         /*!< SPI Tx Half Completed callback     */
                void (* RxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);         /*!< SPI Rx Half Completed callback     */
                void (* TxRxHalfCpltCallback)(struct __SPI_HandleTypeDef *hspi);       /*!< SPI TxRx Half Completed callback   */
                void (* ErrorCallback)(struct __SPI_HandleTypeDef *hspi);              /*!< SPI Error callback                 */
                void (* AbortCpltCallback)(struct __SPI_HandleTypeDef *hspi);          /*!< SPI Abort callback                 */
                void (* MspInitCallback)(struct __SPI_HandleTypeDef *hspi);            /*!< SPI Msp Init callback              */
                void (* MspDeInitCallback)(struct __SPI_HandleTypeDef *hspi);          /*!< SPI Msp DeInit callback            */
            #endif  /* USE_HAL_SPI_REGISTER_CALLBACKS */
        } SPI_HandleTypeDef;

        SPI_HandleTypeDef* spi_module_handle;

        void configure_module(SPI_HandleTypeDef* spi_handle);
        HAL_StatusTypeDef spi_transmit_receive_interrupt(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
    #if (HAL_SPI_USE_REGISTER_CALLBACKS == 1U)
        typedef void (*spi_callback_ptr_t)(SPI_HandleTypeDef* spi_module_handle);
        HAL_StatusTypeDef spi_register_callback(spi_callback_id_t CallbackID, spi_callback_ptr_t pCallback);
        HAL_StatusTypeDef spi_unregister_callback(HAL_SPI_CallbackIDTypeDef CallbackID);
    #endif /* HAL_SPI_USE_REGISTER_CALLBACKS */
        void close_rx_tx_isr();
        void set_rx_and_tx_interrupt_service_routines() const;
        friend void spi_rx_2_line_16_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle);
        friend void spi_tx_2_line_16_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle);
        friend void spi_rx_2_line_8_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle);
        friend void spi_tx_2_line_8_bit_isr(spi spi_object, struct spi::__SPI_HandleTypeDef *spi_handle);
        friend void SPI_DMAAbortOnError(DMA_HandleTypeDef *hdma);
        friend void spi_irq_handler(spi* spi_object);
    private:

        HAL_StatusTypeDef initialize_module();
        spi_status_t lock_module() const;
        void unlock_module() const;
        uint8_t get_module_communication_state() const;
        uint32_t get_module_operating_mode() const;
        void verify_communication_direction(uint32_t intended_direction) const;
        void set_transaction_parameters(uint8_t *tx_data_ptr, uint8_t *rx_data_ptr, uint16_t packet_size) const;
        void reset_enabled_crc();
        HAL_StatusTypeDef wait_for_flag_until_timeout(uint32_t flag, FlagStatus flag_status, uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);
        HAL_StatusTypeDef end_rx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);
        HAL_StatusTypeDef end_rx_tx_transaction(uint32_t flag_timeout, uint32_t start_time_from_hal_get_tick);

        void close_rx_isr();
        void close_tx_isr();
        void abort_rx_isr();
        void abort_tx_isr();

};


#endif //MAIN_CONTROLLER_HAL_SPI_H
