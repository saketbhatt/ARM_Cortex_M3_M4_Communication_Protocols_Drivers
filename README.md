# Driver programmed by saketbhatt.
Driver Development GPIO,SPI,UART,I2C.
GPIO driver based on stm32f103c8t6 cortex m3. 
SPI,UART,I2C drivers based of stm32f407x cortex m4.


Basic structure for the header file includes defined macros, used structures,enum definitionss,prototypes for driver exposed
API 's.

Basic structure for the source file includes helper functions through which everything is implemented so as to give a structural
way of definging everything.API definitions and other macro useage is self explanatory .

Tx and Rx is done not in the inturrupt APIs rather non blocking methodology is implemented and TX and RX are mostly done in 
inturrupt handlers instead of dierectly in APIs.


Looking forward for progressive changes.
