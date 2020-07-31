# DDI Serial Library

This is a custom driver for data acquisition from sensors that communicate using the DDI serial protocol. It has been developed for STM32 microcontrollers using a HAL API.

In the past, many libraries (c.f. page 4 of [this datasheet](http://www.ictinternational.com/content/uploads/2014/03/5TE-Integrators-Guide.pdf)) have implemented DDI using bit-banging. This library reduces the processor overhead of such methods by:

1) leveraging UARTs to implement the 8N1 timing in hardware (see first diagram below), and developing a state machine to implement the setup-hold-reset cycle (see second diagram below).

2) using the integrated DMA controller to facilitate memory transfers at each UART receive-interrupt.

Note that the DDI protocol timing does not conform to a standard amongst manufactures, even though the states do. This implementation is for a particular device described [here](http://www.ictinternational.com/content/uploads/2014/03/5TE-Integrators-Guide.pdf).
