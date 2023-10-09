# Embedded Systems and IoT Mid-Term Project
Ahmad Zaki Akmal - 21/480179/TK/52981

This project is made to fulfill the mid term of Embedded Systems and IoT Class of 2023/2024

## Flowchart of the System
![Flowchart of the system](https://raw.githubusercontent.com/ahmadzaki2975/IOT-MidTerm-Project/main/assets/Flowchart_480179.jpg)

## System Description
This system fulfills the following requirements of the Mid-Term Project.

### 1. The system needs to include at least two sensor/input devices.
This system uses 3 sensor/input devices.
- DHT11 Temperature and Humidity Sensor
- HC-SR04 Ultrasonic Sensor
- Push Button

### 2. The system needs to use an OLED as a display
This system displays data from the sensors to an OLED display.

### 3. The system needs to use an interface to send data to a PC/Server
This system sends the data acquired from the sensors in a JSON format via serial port / UART.

### 4. The system needs to use at least 1 actuator
This system uses a servo motor as an actuator.

### 5. System is programmed using a scheduler/RTOS with minimum 2 kernel objects and 3 tasks
This system is programmed using freeRTOS.

It utilizes 2 kernel objects
- **Queue**, to send data from one task to another.
- **Semaphore/Mutex**, to prevent concurrent access of a resource that may cause conflict.

And has 5 RTOS tasks
- **DHT_Task()**, to run functions that reads temperature and humidity from the DHT11 sensor.
- **Ultrasonic_Task()**, to run functions that reads the distance of the object in front of the ultrasonic sensor.
- **Display_Task()**, to display data acquired from DHT11 sensor and ultrasonic sensor to the OLED and send them via the serial port.
- **Servo_Task()**, to rotate the servo based on a state toggled by the push button.
- **PushButton_Task()**, to toggle the state that controls the servo motor.
