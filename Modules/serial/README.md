
## Limitations

The implementation depends on UART TX/RX interrupts. All UART interrupts share the same priority (pre-emption and grouping priority).

Cons:
- Requires careful planning of interrupts
- UART interrupts must not be blocked in order to receive data on UART correctly and avoid data loss
- Processor clock must be fast enough to process interrupts when receiving bursts of data
- A lot of interrupt context switching when receiving/transmiting bursts of data. That might not be a problem when there are few interrupts in the system. But If there are other interrupts (ADC, SysTick, Timer, etc), performance will drop and some received data might be lost

Pros:
- The implementation is simple
- Can be easily integrated into interrupt driven applications (super loop, background-foreground architecture)

## Solutions

There are 3 solutions to interrupts problem:
1. Don't use interrupts, instead poll UART flags and process TX/RX data accordingly
2. Do minimal work in interrupts. Instead of reading/writing TX/RX data to and from ring buffers, the interrupts sets signals/flags and reading/writing RX/TX data is done outside of interrupts.
3. Use DMA to read/write RX/TX data bytes