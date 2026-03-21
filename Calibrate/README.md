# Get Compass offset
```
git clone https://github.com/nopnop2002/esp-idf-lsm9ds1
cd esp-idf-lsm9ds1/Calibrate
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

### Configuration   
To find the offset value, set the compass offset to 0.   
![Image](https://github.com/user-attachments/assets/4913fb7e-7625-440e-8c71-b5991fac6550)
![Image](https://github.com/user-attachments/assets/b89beab6-f91b-4201-a5c3-225d69868737)

### Execute calibration   
ESP32 acts as a web server.   
I used [this](https://github.com/Molorius/esp32-websocket) component.   
This component can communicate directly with the browser.   
Enter the following in the address bar of your web browser.   
```
http:://{IP of ESP32}/
or
http://esp32.local/
```

As you move the IMU it plots the X, Y and Z values.   
X, Y, Z offset are displayed.   
![Image](https://github.com/user-attachments/assets/1e0f96aa-bb99-4253-859a-39b0d0a2329f)

### Execute calibration again   
If you set the offset you got from the calibration and run it again, the circle position will change.   
![Image](https://github.com/user-attachments/assets/4d6fd692-a714-4f65-abc1-776754d036c3)
