# UART - HAL

UART asynchronous TX/RX using ST HAL drivers. The implementation uses a ring buffer for TX/RX per UART channel, and UART TX/RX interrupts to handle TX/RX asynchronously. Data bytes are added to TX buffer will be transmitted as soon as possible, and received bytes are stored in RX buffer until the data is read.
