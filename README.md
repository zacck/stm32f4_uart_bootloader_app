# Bootloader application for STM32F4 

Roughly follows this [ST application note](https://www.st.com/resource/en/application_note/an3155-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)


## Caveats that were a little hard to come by 

1. Your clocks for the Bootloader and application(s) to be loaded should be exactly the same.
2. Take case to prevent reset_handler and app_reset handler addresses from being optimized