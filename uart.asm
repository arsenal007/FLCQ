list P=16F628A
#include <P16F628A.inc>      ; processor specific definitions
              global _uart
              global _uart_id
UD_UART_0        udata  0x59
_uart_id         res 1

UD_UART_1        udata  0x60
_uart            res 8
              END
