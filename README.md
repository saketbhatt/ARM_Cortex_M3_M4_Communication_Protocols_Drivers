**Problem:**
- Driver Development: GPIO, SPI, UART, I2C.
  
**Scope:**
- GPIO driver based on STM32F103C8T6 Cortex-M3. 
- SPI, UART, and I2C drivers based on STM32F407x Cortex-M4.

**Details:**
- Basic structure for the header file includes defined macros, used structures, enum definitions, and prototypes for driver-exposed APIs.
- Basic structure for the source file includes helper functions through which everything is implemented to give a structural way of defining everything. API definitions and other macro usage are explanatory.
- Tx and Rx are done not in the interrupt APIs; rather non-blocking methodology is implemented, and TX and RX are mostly done in interrupt handlers instead of directly in APIs.
