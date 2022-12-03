# STM32F103xx Serial

Serial communication is widely used in embedded systems applications, to read/write data to/from sensors, other micro-controllers, external memory or consoles (PC). And there are different ways to implement serial communication, depending on the application requirements and architecture. In this repo, I'll give some examples to the most popular ways to implement serial communication. 
I'll be using `STM32F103CBT6` (also known as `bluepill`), which has several types of serial interfaces: USART, SPI, I2C, CAN and USB. I'll focus on USART, SPI and I2C. 

## Introduction

