# IR Remote Controller STM32 device with Bare-metal USB 2.0 implementation

A peripheral device that adds extended functionality to a computer with STM3240G-Eval board through a USB 2.0 connection. The STM32 board is programmed as an HID device that can decode IR signals from a IR remote with the help of an IR receiver connected to the board and forward it to the PC. The commands are sent to the PC via a bare-metal USB 2.0 FS implementation and helps the device be enumerated as a USB HID device. There is also a provided LCD with the STM32 board that is used for signalling the user if the device is ready to use after the connection with the computer and what the command sent by the button press on the remote is supposed to do.

Below is the setup for the project:

![alt-text](https://github.com/Hishmann/Remote-Peripheral-for-Computers-using-STM32/blob/main/Images/device_setup.png "device setup")
