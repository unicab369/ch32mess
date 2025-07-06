#include "uart.h"
#include "fsusb.h"
#include "ch32fun.h"
#include <string.h>

UART_config_t uart;
CDC_config_t cdc;
volatile uint8_t uart_debug = 0;

void uart_config(UART_config_t * uart) {
	uint8_t i = uart->number;
#if CH5xx
	UART(i)->IER = RB_IER_TXD_EN;
	UART(i)->FCR = (2 << 6) | RB_FCR_TX_FIFO_CLR | RB_FCR_RX_FIFO_CLR | RB_FCR_FIFO_EN;
	UART(i)->LCR = uart->word_length | uart->parity | uart->tx_stop_bits;
	UART(i)->_DL = ((10 * (FUNCONF_SYSTEM_CORE_CLOCK) / 8 / uart->baud) + 5) / 10 ;
	UART(i)->_DIV = 1;
#else
	UART(i)->CTLR1 = uart->word_length | uart->parity | USART_Mode_Rx | USART_Mode_Tx;
	UART(i)->CTLR2 = uart->stop_bits;
	uint8_t pb1_div = (RCC->CFGR0 & RCC_PPRE1) >> 8;
	switch (pb1_div)
	{
	default:
	case 0:
		UART(i)->BRR = UART_CALC_DIV(uart->baud);
		break;
	case 4:
		UART(i)->BRR = UART_CALC_DIV(uart->baud * 2);
		break;
	case 5:
		UART(i)->BRR = UART_CALC_DIV(uart->baud * 4);
		break;
	case 6:
		UART(i)->BRR = UART_CALC_DIV(uart->baud * 8);
		break;
	case 7:
		UART(i)->BRR = UART_CALC_DIV(uart->baud * 16);
		break;
	}
	
	UART(i)->STATR = (uint16_t)(~USART_FLAG_TC);
	UART(i)->CTLR1 |= CTLR1_UE_Set;
#endif
}

void cdc_init(CDC_config_t * ctx) {

	uart.number = UART_NUMBER;
	uart.baud = UART_DEFALT_BAUD;
	uart.word_length = UART_DEFAULT_WORDL;
	uart.stop_bits = UART_DEFAULT_STOPB;
	uart.parity = UART_DEFAULT_PARITY;
	uart.flow_control = UART_DEFAULT_FLOW;

	cdc.uart = &uart;
	cdc.tx_wrap_pos = UART_TX_BUF_SIZE;

	cdc.cdc_cfg[0] = (uint8_t)(UART_DEFALT_BAUD);
	cdc.cdc_cfg[1] = (uint8_t)(UART_DEFALT_BAUD >> 8);
	cdc.cdc_cfg[2] = (uint8_t)(UART_DEFALT_BAUD >> 16);
	cdc.cdc_cfg[3] = (uint8_t)(UART_DEFALT_BAUD >> 24);
	cdc.cdc_cfg[4] = UART_DEFAULT_STOPB;
	cdc.cdc_cfg[5] = UART_DEFAULT_PARITY;
	cdc.cdc_cfg[6] = 8;
	cdc.cdc_cfg[7] = UART_RX_TIMEOUT * 10;

	uart_config(cdc.uart);

#ifndef CH5xx
	
	// UART Tx-DMA configuration
	UART_TX_DMA->CFGR = DMA_DIR_PeripheralDST | DMA_MemoryInc_Enable | DMA_Priority_Medium;
	UART_TX_DMA->PADDR = (uint32_t)(&UART(ctx->uart->number)->DATAR);
	UART_TX_DMA->MADDR = (uintptr_t)uart_tx_buffer;

	// UART Rx-DMA configuration
	UART_RX_DMA->CFGR = DMA_Mode_Circular | DMA_MemoryInc_Enable | DMA_Priority_Medium;
	UART_RX_DMA->CNTR = UART_RX_BUF_SIZE;
	UART_RX_DMA->PADDR = (uint32_t)(&UART(ctx->uart->number)->DATAR);
	UART_RX_DMA->MADDR = (uintptr_t)uart_rx_buffer;
	UART_RX_DMA->CFGR |= DMA_CFGR1_EN;
	
	UART(ctx->uart->number)->CTLR3 = USART_DMAReq_Tx | USART_DMAReq_Rx;
#endif
	USBFS->UEP2_DMA = (uintptr_t)uart_tx_buffer;
}

void uart_process_rx(CDC_config_t * ctx) {
	uint32_t transferred;
	uint32_t packlen = 0;
	uint32_t current_rx_dma_cnt;
	int ret;

	NVIC_DisableIRQ(USB_IRQn);
	current_rx_dma_cnt = UART_RX_DMA->CNTR;
	if (ctx->rx_dma_cnt != current_rx_dma_cnt) {
		if (uart_debug) printf("uart rx, current_rx_dma_cnt = %ld, rx_dma_cnt= %ld\n", current_rx_dma_cnt, ctx->rx_dma_cnt);
		if (ctx->rx_dma_cnt > current_rx_dma_cnt) {
			transferred = ctx->rx_dma_cnt - current_rx_dma_cnt;
		} else {
			transferred = UART_RX_BUF_SIZE - current_rx_dma_cnt;
			transferred += ctx->rx_dma_cnt;
		}
		ctx->rx_dma_cnt = current_rx_dma_cnt;

		if ((ctx->rx_remain + transferred) > UART_RX_BUF_SIZE) printf("[RX buffer overflow]: %ld\n", ctx->rx_remain);
		else ctx->rx_remain += transferred;

		ctx->rx_timeout = 0;
	}
	NVIC_EnableIRQ(USB_IRQn);

	if (ctx->rx_remain) {
		if (ctx->rxing == 0) {

			if( ctx->rx_remain >= USBFS_PACKET_SIZE ) packlen = USBFS_PACKET_SIZE;
			else if (ctx->rx_timeout >= UART_RX_TIMEOUT) packlen = ctx->rx_remain;
			
			if (packlen > (UART_RX_BUF_SIZE - ctx->rx_pos)) packlen = (UART_RX_BUF_SIZE - ctx->rx_pos);

			if (packlen) {
				NVIC_DisableIRQ(USB_IRQn);
				ctx->usb_timeout = 0;
				ret = USBFS_SendEndpointNEW(3, (uint8_t*)(uart_rx_buffer + ctx->rx_pos), packlen, 1);
				if (ret == 0) {
					ctx->rxing = packlen;
					ctx->rx_remain -= packlen;
					ctx->rx_pos += packlen;
					if (ctx->rx_pos >= UART_RX_BUF_SIZE) ctx->rx_pos = 0;
					// Indicate we are done sending for now, so host can pass data further
					// https://electronics.stackexchange.com/questions/253669/usb-cdc-help-with-zero-packets
					if (packlen == USBFS_PACKET_SIZE) ctx->zero_packet_pending = 1;
				}
				NVIC_EnableIRQ(USB_IRQn);
			}
		} else {
			// Manually clear rxing flag if haven't receive IN request
			if (ctx->usb_timeout >= UART_USB_TIMEOUT) {
				ctx->rxing = 0;
				USBFSCTX.USBFS_Endp_Busy[3] = 0;
			}
		}
	}
	if (ctx->zero_packet_pending && ctx->rxing == 0 && ctx->usb_timeout >= UART_ZERO_TIMEOUT) {
		NVIC_DisableIRQ( USB_IRQn );
		ctx->usb_timeout = 0;
		ctx->rxing = 1;
		USBFS_SendACK(3, 1);
		// USBFS_SendEndpointNEW(3, (uint8_t*)(uart_rx_buffer + ctx->rx_pos), 0, 0);
		ctx->zero_packet_pending = 0;
		NVIC_EnableIRQ( USB_IRQn );
	}
}

void uart_process_tx(CDC_config_t * ctx) {
	if (ctx->txing) {
		// Check for UART Transmission Completed flag
		if (USART2->STATR & USART_FLAG_TC) {
			if (uart_debug) printf("TC\n");
			UART(ctx->uart->number)->STATR = ~(USART_FLAG_TC); // Clear TX end flag
			UART_TX_DMA->CFGR &= (uint16_t)(~DMA_CFGR1_EN); // Disable TX DMA
			
			if (UART_TX_DMA->CNTR) {
				printf("[TX fault] DMA->CNTR = %ld, pos = %d, remain = %ld", UART_TX_DMA->CNTR, ctx->tx_pos, ctx->tx_remain);
			}

			NVIC_DisableIRQ(USB_IRQn);
			ctx->tx_remain -= ctx->txing;
			ctx->tx_pos += ctx->txing;
			if( ctx->tx_pos & 0x3 ) ctx->tx_pos = (ctx->tx_pos + 4) & ~0x3;
			ctx->txing = 0;
			if ((ctx->tx_stop) && (!ctx->tx_remain)) {
				USBFS_SendACK(2, 0);
				ctx->tx_stop = 0;
			}
			
			NVIC_EnableIRQ(USB_IRQn);
		}
		
	}
	if (ctx->txing == 0 && ctx->tx_remain) {
			NVIC_DisableIRQ(USB_IRQn);

			if (ctx->tx_wrap_pos <= ctx->tx_pos) {
				ctx->tx_pos = 0; 
				ctx->tx_wrap_pos = UART_TX_BUF_SIZE;
			}

			UART_TX_DMA->MADDR = (uint32_t)(uart_tx_buffer + ctx->tx_pos);; // Set address for DMA to read from
			uint32_t to_send = (ctx->tx_pos + ctx->tx_remain < ctx->tx_wrap_pos)?ctx->tx_remain:(ctx->tx_wrap_pos - ctx->tx_pos);
			
			if (to_send == 0) return;
			UART_TX_DMA->CNTR = to_send;
			if (uart_debug) printf("tx-pos = %d, to_send = %ld, txing = %ld\n", ctx->tx_pos, to_send, ctx->txing);
			if (uart_debug) printf("transmission started: tx_pos = %d, tx_remain = %ld, to_send = %ld UART_TX_DMA->MADDR = %08lx\n", ctx->tx_pos, ctx->tx_remain, to_send, UART_TX_DMA->MADDR);
			
			UART_TX_DMA->CFGR |= (uint16_t)(DMA_CFGR1_EN);  // Enable TX DMA
			// UART(ctx->uart->number)->CTLR3 |= USART_DMAReq_Tx;  // Enable UART DMA transmission
			ctx->txing = to_send;
			
			NVIC_EnableIRQ(USB_IRQn);
	}
}

void uart_send_break(uint8_t n) {
#ifndef CH5xx
		UART(n)->CTLR1 |= CTLR1_SBK_Set;
#endif
}