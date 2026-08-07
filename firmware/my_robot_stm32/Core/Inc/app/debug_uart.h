#ifndef DEBUG_UART_H
#define DEBUG_UART_H

void debug_uart_init(void);
void debug_output(void);    /* call in main loop, ~50Hz throttled */

#endif /* DEBUG_UART_H */
