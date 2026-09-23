"""Minimal UART2 pin test for the alternate K230 board."""

from machine import FPIOA, UART
import time


fpioa = FPIOA()
print("[UART2] mapping GPIO11 as TX")
fpioa.set_function(11, FPIOA.UART2_TXD)
print("[UART2] mapping GPIO12 as RX")
fpioa.set_function(12, FPIOA.UART2_RXD)

uart = UART(
    UART.UART2,
    baudrate=115200,
    bits=UART.EIGHTBITS,
    parity=UART.PARITY_NONE,
    stop=UART.STOPBITS_ONE,
)

print("[UART2] GPIO11/12 ready")
while True:
    uart.write("K230_UART2_OK\r\n")
    time.sleep_ms(1000)
