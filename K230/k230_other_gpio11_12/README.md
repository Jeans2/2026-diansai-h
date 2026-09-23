# 另一块 K230 串口配置

- UART：UART2
- TX：GPIO11
- RX：GPIO12
- 参数：115200，8N1

接线：GPIO11/TX 接天猛星 PB7/RX，GPIO12/RX 接天猛星 PB6/TX，GND 共地。

`main.py` 是完整钢球识别程序；`uart2_gpio11_12_test.py` 是最小串口测试程序。
