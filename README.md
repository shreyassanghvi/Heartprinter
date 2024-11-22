# Heartprinter
This is a repo for heart printer - contains documents and code

## Version 1
### ESP Board
ESP32 Servo Driver Expansion Board, Built-In WiFi and Bluetooth
link: *https://www.waveshare.com/servo-driver-with-esp32.htm*


### Motor Specs

Name: 30KG Serial Bus Servo, High precision and torque, with Programmable 360 Degrees Magnetic Encoder

Model No: *waveshare ST3215 servo*

link: *https://www.waveshare.com/st3215-servo.htm*

Wiki: *https://www.waveshare.com/wiki/ST3215_Servo*

Obs:
* ESP32 is unstable with the motor driver firmware given by the company.
* Motor would only work if it is on prop firmware and would not give any benefit over the AX-12 motors.

POA:
* Shortlist a few motors that are controller agnostic
* get motor that have feedback control and have better documentation

## Version 2
### Motor Specs
* Voltage
    * Vmax: 12V
    * Vmin: 5V
* Torque (N-m): 0.2-0.3
* Encoder: Absolute
* Comms: TTL/RS-485

*Dynamixel 2.0 Software for motor config:* https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/

### STM32F103RBT6
Link: *https://www.st.com/en/microcontrollers-microprocessors/stm32f103rb.html*

### Variable voltage from ATX PSU
* https://www.amazon.com/CHENBO-Benchtop-Computer-Breakout-Adapter/dp/B07S91NQL3?sr=8-6
* https://www.amazon.com/HLT-Breakout-Adjustable-Voltage-Maximum/dp/B0CKX9QL1G?sr=8-13
