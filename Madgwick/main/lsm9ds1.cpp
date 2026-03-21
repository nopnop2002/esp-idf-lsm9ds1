#include <cstring>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "cJSON.h"

#include "LSM9DS1.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// Source: https://github.com/arduino-libraries/MadgwickAHRS
#include "MadgwickAHRS.h"

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Create the Madgwick instances
Madgwick madgwick;

// Create the IMU instances
LSM9DS1Class imu(CONFIG_MAIN_I2C_ADDR, CONFIG_MAG_I2C_ADDR);

void getMagData(float *magX, float *magY, float *magZ) {
	while(1){
		if (imu.magneticFieldAvailable()) break;
		vTaskDelay(1);
	}

	float mx=0.0, my=0.0, mz=0.0;
	imu.readMagneticField(mx, my, mz);
	//printf("mag=%f %f %f\n", mx, my, mz);
	*magX = mx + CONFIG_MAG_OFFSET_X;
	*magY = my + CONFIG_MAG_OFFSET_Y;
	*magZ = mz + CONFIG_MAG_OFFSET_Z;
}

// Get scaled value
void getMotion6(float *_ax, float *_ay, float *_az, float *_gx, float *_gy, float *_gz) {
	float ax=0.0, ay=0.0, az=0.0;
	float gx=0.0, gy=0.0, gz=0.0;
	while(1) {
		if (imu.accelerationAvailable()) break;
		vTaskDelay(1);
	}
	imu.readAcceleration(ax, ay, az);

	while(1) {
		if (imu.gyroscopeAvailable()) break;
		vTaskDelay(1);
	}
	imu.readGyroscope(gx, gy, gz);
	//printf("%f %f %f %f %f %f\n", ax, ay, az, gx, gy, gz);

	*_ax = ax;
	*_ay = ay;
	*_az = az;

	*_gx = gx;
	*_gy = gy;
	*_gz = gz;
}

// Get time in seconds since boot
// Compatible with ROS's time.toSec() function
double TimeToSec() {
	int64_t _time = esp_timer_get_time(); // Get time in microseconds since boot
	double __time = (double)_time / 1000000;
	return __time;
}

void lsm9ds1(void *pvParameters){
    // Initialize IMU
    if (imu.begin() == 0) {
        ESP_LOGE(TAG, "Connection fail");
        vTaskDelete(NULL);
    }

    // Calibrate IMU
    ESP_LOGW(TAG, "IMU is currently being calibrated. Please do not move it.");
    float gyroBias[3];
    float accelBias[3];
    imu.getBias(gyroBias, accelBias);
    printf("gyroBias=%f %f %f\n", gyroBias[0], gyroBias[1], gyroBias[2]);
    printf("accelBias=%f %f %f\n", accelBias[0], accelBias[1], accelBias[2]);
    imu.setBias(gyroBias, accelBias);
    vTaskDelay(500);
    ESP_LOGW(TAG, "IMU calibration is complete.");

	double last_time_ = TimeToSec();
	int elasped = 0;
	bool initialized = false;
	float initial_roll = 0.0;
	float initial_pitch = 0.0;

	float roll = 0.0, pitch = 0.0, yaw = 0.0;
	float _roll = 0.0, _pitch = 0.0;

	while(1){
		// Get scaled value
		float ax, ay, az;
		float gx, gy, gz;
		getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		float mx, my, mz;
		getMagData(&mx, &my, &mz);
		ESP_LOGD(TAG, "mag=%d %d %d", mx, my, mz);

		// Get the elapsed time from the previous
		float dt = (TimeToSec() - last_time_);
		ESP_LOGD(TAG, "dt=%f",dt);
		last_time_ = TimeToSec();

		// Get Euler
		madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
		roll = madgwick.getRoll();
		pitch = madgwick.getPitch();
		yaw = madgwick.getYaw();
		ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);

		/* Print Data every 2 times */
		if (elasped > 2) {
			// Set the first data
			TickType_t nowTicks = xTaskGetTickCount();
			if (initialized == false && nowTicks > 6000) {
				initial_roll = roll;
				initial_pitch = pitch;
				initialized = true;
			}
			_roll = roll-initial_roll;
			_pitch = pitch-initial_pitch;
			ESP_LOGD(TAG, "roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
			ESP_LOGD(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);

			if (initialized) {
				ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);
				// Send UDP packet
				POSE_t pose;
				pose.roll = _roll;
				pose.pitch = _pitch;
				pose.yaw = yaw;
				if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
					ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
				}

				// Send WEB request
				cJSON *request;
				request = cJSON_CreateObject();
				cJSON_AddStringToObject(request, "id", "data-request");
				cJSON_AddNumberToObject(request, "roll", _roll);
				cJSON_AddNumberToObject(request, "pitch", _pitch);
				cJSON_AddNumberToObject(request, "yaw", yaw);
				char *my_json_string = cJSON_Print(request);
				ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
				size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
				if (xBytesSent != strlen(my_json_string)) {
					ESP_LOGE(TAG, "xMessageBufferSend fail");
				}
				cJSON_Delete(request);
				cJSON_free(my_json_string);
			} else {
				ESP_LOGW(TAG, "unstable roll:%f pitch=%f yaw=%f", _roll, _pitch, yaw);
			}

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
