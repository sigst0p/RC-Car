# RC-Car
This project is about building a remote controlled RC-Car using a microcontroller and bluetooth communication.

used components:
* a chassis from a old broken RC-Car (including the motors)
* relays (controlling the motors)
* USART bluetooth module
* ATmega328p Microcontroller
* A battery (7.2V)
* Stepdown converter 5V for the microcontroller

The car is controlled with JSON-Strings which are send e.g. by a smartphone over bluetooth and parsed by the microcontroller
