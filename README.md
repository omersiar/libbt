# libbt
Bluetooth support for Circle

Currently not usable, raspberry pi's internal bluetooth chip usually also the same chip for wlan, chip communication bus seems to be UART on all pi boards and driven with different UART devices or software UART implementations, so there is no just one reciepe for them, different boards also have different BT chips and even different revisions of the same chip.

3 B+ and 3 A+ utilizing PL011 UART device to drive BT chip, on Linux scene these UART device interfaces can be changed to use other interfaces like software defined miniUART. (I don't know if Circle has support for it). Pi 4 and 5 has multiple hardware UARTs which would be more convenient for driving BT chips, otherwise software UART interfaces give limited UART feature set 

> The mini-UART has smaller FIFOs. Combined with the lack of flow control, this makes it more prone to losing characters at higher baudrates. It is also generally less capable than a PL011, mainly due to its baud rate link to the VPU clock speed.

