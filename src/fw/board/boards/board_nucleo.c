#include "board/board.h"

#include "drivers/exti.h"
#include "drivers/i2c_definitions.h"
#include "drivers/stm32f2/dma_definitions.h"
#include "drivers/stm32f2/i2c_hal_definitions.h"
#include "drivers/stm32f2/spi_definitions.h"
#include "drivers/stm32f2/uart_definitions.h"


// DMA Controllers

static DMAControllerState s_dma1_state;
static DMAController DMA1_DEVICE = {
  .state = &s_dma1_state,
  .periph = DMA1,
  .rcc_bit = RCC_AHB1Periph_DMA1,
};

static DMAControllerState s_dma2_state;
static DMAController DMA2_DEVICE = {
  .state = &s_dma2_state,
  .periph = DMA2,
  .rcc_bit = RCC_AHB1Periph_DMA2,
};

// DMA Streams
CREATE_DMA_STREAM(2, 3); // GC9A01

// DMA Requests

static DMARequestState s_gc9a01_spi_tx_dma_request_state;
static DMARequest GC9A01_SPI_TX_DMA_REQUEST = {
  .state = &s_gc9a01_spi_tx_dma_request_state,
  .stream = &DMA2_STREAM3_DEVICE,
  .channel = 0,
  .irq_priority = 0x0f,
  .priority = DMARequestPriority_High,
  .type = DMARequestType_MemoryToPeripheral,
  .data_size = DMARequestDataSize_Byte,
};
DMARequest * const GC9A01_SPI_TX_DMA = &GC9A01_SPI_TX_DMA_REQUEST;

// I'm actually no entirely sure what this is for yet.
static DMARequestState s_dbg_uart_dma_request_state;
static DMARequest DBG_UART_RX_DMA_REQUEST = {
  .state = &s_dbg_uart_dma_request_state,
  .stream = &DMA2_STREAM2_DEVICE,
  .channel = 4,
  .irq_priority = IRQ_PRIORITY_INVALID, // no interrupts
  .priority = DMARequestPriority_VeryHigh,
  .type = DMARequestType_PeripheralToMemory,
  .data_size = DMARequestDataSize_Byte,
};

// UART DEVICES

static UARTDeviceState s_dbg_uart_state;
static UARTDevice DBG_UART_DEVICE = {
  .state = &s_dbg_uart_state,
  .tx_gpio = {
    .gpio = GPIOA,
    .gpio_pin = GPIO_Pin_9,
    .gpio_pin_source = GPIO_PinSource9,
    .gpio_af = GPIO_AF_USART1
  },
  .rx_gpio = {
    .gpio = GPIOB,
    .gpio_pin = GPIO_Pin_7,
    .gpio_pin_source = GPIO_PinSource7,
    .gpio_af = GPIO_AF_USART1
  },
  .periph = USART1,
  .irq_channel = USART1_IRQn,
  .irq_priority = 13,
  .rcc_apb_periph = RCC_APB2Periph_USART1,
  .rx_dma = &DBG_UART_RX_DMA_REQUEST
};
UARTDevice * const DBG_UART = &DBG_UART_DEVICE;
IRQ_MAP(USART1, uart_irq_handler, DBG_UART);
// NOTE: There should be a *lot* more defined here as Peridot progresses, but for now all we have is the GC9A01 display and UART debug stuff. Get the OS running with the display first before we add anything new.



void board_early_init(void) {
}

void board_init(void) {


}
