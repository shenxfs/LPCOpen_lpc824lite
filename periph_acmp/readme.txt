Analog Comparator Example
=========================

Example Description
-------------------
The ACMP example demonstrates the analog comparator.

This example configures the comparator positive input to the  potentiometer on the
LPCXpresso base-board (FRDM mini shield). 
The negative input is connected to the internal band-gap
reference voltage. The output of the comparator is used to drive an LED.
The comparator threshold is configured to generate an interrupt.

When the potentiometer is adjusted right and left, the voltage to the positive 
side of the comparator is changed. When the voltage crosses the negative
comparator input, a CMP_IRQ is fired. Based on the voltage reference in relation
to the band-gap, the LED state will change.

Special Connection Requirements
-------------------------------
Board [NXP_LPC824Lite]:
Short connection of P0_6 and P0_23 on the LPCX824Lite board.
Adjust potentiometer on the LPCX824Lite board and find LED1 will blink.

此DEMO演示的是比较器的功能，比较器的正极(P0_6)连接上电位器的分压端（P0_6与P0_23短接），
负极连接到内部分压端(0.9V)。比较器的输出产生中断，驱动LED1的亮灭。


把LPC824Lite板子的P0_6与P0_23短接，下载程序后复位，
调节LPC824Lite板子上的电位器，可以观察到LED1的亮或者灭。