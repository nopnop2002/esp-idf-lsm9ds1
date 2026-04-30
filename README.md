# esp-idf-lsm9ds1
A demo showing the pose of the lsm9ds1 9DoF IMU sensor in 3D using esp-idf.   

You can use the Kalman filter or the Madgwick filter to estimate the Euler angle.   
Euler angles are roll, pitch and yaw.   
It's very intuitive and easy to understand.   
![a-Pitch-yaw-and-roll-angles-of-an-aircraft-with-body-orientation-O-u-v-original](https://user-images.githubusercontent.com/6020549/224452743-d4cf419d-f936-4e46-9ece-a12f21bf2e32.jpg)   
You can view like this.   
![Image](https://github.com/user-attachments/assets/cb9f1408-b372-4e55-84f6-1191f068cd1a)


# Installation overview

- Get Compass offset from IMU.

- Get Euler angles from IMU.

- Display Euler angles in browser.

# Software requirements
ESP-IDF V5.2 or later.   
Because this project uses the new I2C driver.   

# Hardware requirements
LSM9DS1 3D accelerometer, 3D gyroscope, 3D magnetometer Measurement Sensors.

# Wireing
|LSM9DS1||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6||
|:-:|:-:|:-:|:-:|:-:|:-:|
|VDD|--|3.3V|3.3V|3.3V||
|GND|--|GND|GND|GND||
|SCL|--|GPIO22|GPIO12|GPIO5|(*1)|
|SDA|--|GPIO21|GPIO11|GPIO4|(*1)|
|CSM|--|3.3V|3.3V|3.3V|Use i2c|
|CSAG|--|3.3V|3.3V|3.3V|Use i2c|
|SDOM|--|GND/3.3V|GND/3.3V|GND/3.3V|(*2)|
|SDOAG|--|GND/3.3V|GND/3.3V|GND/3.3V|(*3)|

(*1)You can change it to any pin using menuconfig.   

(*2)Magnetometer I2C address selection.   
GND:i2c address is 0x1C.   
3.3V:i2c address is 0x1E.   

(*3)Accelerometer/Gyroscope I2C address selection.   
GND:i2c address is 0x6A.   
3.3V:i2c address is 0x6B.   

# Find the sensor
We can find the sensor using [i2c-tools](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c/i2c_tools).   

- Select SCL and SDA using menuconfig.   
![Image](https://github.com/user-attachments/assets/5a5d46fd-ce1e-4199-acf4-c6b49abc2dd8)

- Detect senser.   
The i2c address for Magnetometer is 0x1C or 0x1E.   
The i2c address for Accelerometer/Gyroscope is 0x6A or 0x6B.   
![Image](https://github.com/user-attachments/assets/3d9fd3ff-86b3-4464-bf67-904091cfafe8)   
![Image](https://github.com/user-attachments/assets/e5e8f291-aec7-43dc-96d2-01a6b4389491)   

- Read register.   
Register 0x0f for Magnetometer is 0x3D.   
Register 0x0f for Accelerometer/Gyroscope is 0x68.   
![Image](https://github.com/user-attachments/assets/01313234-87c2-4529-8367-41191254ba47)

# Get Compass offset from IMU
Use [Calibrate](https://github.com/nopnop2002/esp-idf-lsm9ds1/tree/main/Calibrate) to find the compass offset.   

# Get Euler angles from IMU using Kalman filter
```
git clone https://github.com/nopnop2002/esp-idf-lsm9ds1
cd esp-idf-lsm9ds1/Kalman
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration
<img width="659" height="486" alt="Image" src="https://github.com/user-attachments/assets/1093c263-1dba-4ca6-bd5b-4310db91f322" />
<img width="659" height="486" alt="Image" src="https://github.com/user-attachments/assets/5d9fa0d1-c128-4d61-ace4-379c64953647" />

# Get Euler angles from IMU using Madgwick filter
```
git clone https://github.com/nopnop2002/esp-idf-lsm9ds1
cd esp-idf-lsm9ds1/Madgwick
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration
<img width="659" height="486" alt="Image" src="https://github.com/user-attachments/assets/8a94747d-ae8b-4c89-9ca4-889542f82fed" />
<img width="659" height="486" alt="Image" src="https://github.com/user-attachments/assets/752c16e4-bca6-4c3a-973f-ffb5db0333ca" />

# View Euler angles with built-in web server   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

![browser-roll-pitch-yaw](https://user-images.githubusercontent.com/6020549/232365926-ccc6198b-42ec-44f7-891d-6caa93c3411c.JPG)

WEB pages are stored in the html folder.   
I used [this](https://threejs.org/) for 3D display.   
I used [this](https://canvas-gauges.com/) for gauge display.   
Configuration Options for the gauge display is [here](https://canvas-gauges.com/documentation/user-guide/configuration).   
You can change the design and color according to your preference like this.   
![Image](https://github.com/user-attachments/assets/d0e1ca46-0d46-41ed-bbbc-9f26af28900d)


# View Euler angles using PyTeapot   
You can view Euler angles using [this](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation) tool.   
It works as a UDP display server.   
This is a great application.   

```
+-------------+          +-------------+          +-------------+
|             |          |             |          |             |
|     IMU     |--(i2c)-->|    ESP32    |--(UDP)-->| pyteapot.py |
|             |          |             |          |             |
+-------------+          +-------------+          +-------------+
```

### Installation for Linux
```
$ python3 --version
Python 3.11.2
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install pygame
$ python3 -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python3 pyteapot.py
```

The posture of your sensor is displayed.   
![pyteapot_2023-03-11_09-11-46](https://user-images.githubusercontent.com/6020549/224452173-2350704d-1fc4-4a12-8324-434c11f62c52.png)

### Installation for Windows   
Install Git for Windows from [here](https://gitforwindows.org/).   
Install Python Releases for Windows from [here](https://www.python.org/downloads/windows/).   
Open Git Bash and run:   
```
$ python --version
Python 3.11.9
$ python -m pip install -U pip
$ python -m pip install pygame
$ python -m pip install PyOpenGL PyOpenGL_accelerate
$ git clone https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
$ cd PyTeapot-Quaternion-Euler-cube-rotation
$ python pyteapot.py
```

The posture of your sensor is displayed.   
![PyTeapot-Windows](https://github.com/user-attachments/assets/2b0a1a70-40cb-47e5-8f51-eb4fe3adb1ab)


### String passed over Wifi to pyteapot.py
Yaw angle should be betweem two ```y```.   
Pitch angle should be between two ```p```.   
Roll angles should be between two ```r```.   
```
# yaw = 168.8099
# pitch = 12.7914
# roll = -11.8401
# Euler angles only
y168.8099yp12.7914pr-11.8401r

```

# View Euler angles using panda3d library   
You can view Euler angles using [this](https://www.panda3d.org/) library.   
It works as a UDP display server.   

```
+-------------+          +-------------+          +-------------+
|             |          |             |          |             |
|     IMU     |--(ic2)-->|    ESP32    |--(UDP)-->|  panda.py   |
|             |          |             |          |             |
+-------------+          +-------------+          +-------------+
```

### Installation for Linux
```
$ python3 --version
Python 3.11.2
$ sudo apt install python3-pip python3-setuptools
$ python3 -m pip install -U pip
$ python3 -m pip install panda3d
$ cd esp-idf-mpu6050-dmp/panda3d
$ python3 panda.py --help
usage: panda.py [-h] [--model {jet,biplain,707,fa18}]

options:
  -h, --help            show this help message and exit
  --model {jet,biplain,707,fa18}
```
![Image](https://github.com/user-attachments/assets/83804b5e-3ffe-4e18-966e-0ce180c1ab21)

### Installation for Windows
Install Git for Windows from [here](https://gitforwindows.org/).   
Install Python Releases for Windows from [here](https://www.python.org/downloads/windows/).   
Open Git Bash and run:   
```
$ python --version
Python 3.11.9
$ python -m pip install -U pip
$ python -m pip install panda3d
$ cd esp-idf-mpu6050-dmp/panda3d
$ python panda.py --help
usage: panda.py [-h] [--model {jet,biplain,707,fa18}]

options:
  -h, --help            show this help message and exit
  --model {jet,biplain,707,fa18}
```
![Image](https://github.com/user-attachments/assets/7f2fbdf4-97d9-40c3-87db-9f8386741220)

### How to use   
See [here](https://github.com/nopnop2002/esp-idf-mpu6050-dmp/blob/main/panda3d/README.md)   
