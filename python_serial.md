### To See USB port in Ubuntu
```
lsusb
```
## If serial port does not show in Arduino IDE on Ubuntu
```
sudo apt remove brltty 
```
Plug the arduino or dongle off and on

## Arduino and Python Serial Communication Example
### Python Code
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(0.1)

user_input = "L"

while user_input != 'q':
    user_input = input("H = LED ON, L = LED_OFF, q = Quit : ")

    byte_data = bytes(user_input, 'utf-8')
    byte_data_capitalize = byte_data.capitalize()
   
    ser.write(byte_data_capitalize) 
    time.sleep(0.5) 

for i in range(5):
    ser.write(b'H')   
    time.sleep(0.2)        
    ser.write(b'L')  
    time.sleep(0.2)

print('You entered q so, I am Exiting the program')

ser.close()

```

### Arduino Code
```c++
const int ledPin = 13; 
int incomingByte;      // variable stores incoming serial data

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}
```

