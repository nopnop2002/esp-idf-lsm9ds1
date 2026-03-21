#include <cstring>
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "cJSON.h"

#include "LSM9DS1.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())

// Create the Kalman instances
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ;

// Create the IMU instances
LSM9DS1Class imu(CONFIG_MAIN_I2C_ADDR, CONFIG_MAG_I2C_ADDR);

void getMagData(double *magX, double *magY, double *magZ) {
	while(1){
		if (imu.magneticFieldAvailable()) break;
		vTaskDelay(1);
	}

	float mx=0.0, my=0.0, mz=0.0;
	imu.readMagneticField(mx, my, mz);
	//printf("mag=%f %f %f\n", mx, my, mz);
	*magX = (double)(mx + CONFIG_MAG_OFFSET_X);
	*magY = (double)(my + CONFIG_MAG_OFFSET_Y);
	*magZ = (double)(mz + CONFIG_MAG_OFFSET_Z);
}

// Get scaled value
void getMotion6(double *_ax, double *_ay, double *_az, double *_gx, double *_gy, double *_gz) {
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

	*_ax = (double)ax;
	*_ay = (double)ay;
	*_az = (double)az;

	*_gx = (double)gx;
	*_gy = (double)gy;
	*_gz = (double)gz;
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of - to	(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

// See: http://www.freescale.com/files/sensors/doc/app_note/AN4248.pdf
void updateYaw(double magX, double magY, double magZ, double kalAngleX, double kalAngleY, double *yaw) {
	double _magX = magX * -1.0; // Invert axis - this it done here, as it should be done after the calibration
	double _magY = magY;
	double _magZ = magZ * -1.0;

	double rollAngle = kalAngleX * DEG_TO_RAD;
	double pitchAngle = kalAngleY * DEG_TO_RAD;

	//double Bfy = _magZ * sin(rollAngle) - _magY * cos(rollAngle);
	double Bfy = _magY * cos(rollAngle) - _magZ * sin(rollAngle);
	double Bfx = _magX * cos(pitchAngle) + _magY * sin(pitchAngle) * sin(rollAngle) + _magZ * sin(pitchAngle) * cos(rollAngle);
	*yaw = atan2(-Bfy, Bfx) * RAD_TO_DEG;

	//yaw *= -1;
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

	// Set Kalman and gyro starting angle
	double ax, ay, az;
	double gx, gy, gz;
	double mx, my, mz;
	double roll, pitch, yaw; // Roll and pitch are calculated using the accelerometer
	double kalAngleX, kalAngleY, kalAngleZ; // Calculated angle using a Kalman filter

	getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	getRollPitch(ax, ay, az, &roll, &pitch);
	kalAngleX = roll;
	kalAngleY = pitch;
	getMagData(&mx, &my, &mz);
	updateYaw(mx, my, mz, kalAngleX, kalAngleY, &yaw);
	kalAngleZ = yaw;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	kalmanZ.setAngle(yaw);
	uint32_t timer = micros();

	int elasped = 0;
	bool initialized = false;
	double initial_kalAngleX = 0.0;
	double initial_kalAngleY = 0.0;

	while(1){
		getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);
		getRollPitch(ax, ay, az, &roll, &pitch);
		getMagData(&mx, &my, &mz);

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		double gyroXrate = gx;
		double gyroYrate = gy;

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Yaw estimation */
		updateYaw(mx, my, mz, kalAngleX, kalAngleY, &yaw);
		double gyroZrate = gz;
		// This fixes the transition problem when the yaw angle jumps between -180 and 180 degrees
		if ((yaw < -90 && kalAngleZ > 90) || (yaw > 90 && kalAngleZ < -90)) {
			kalmanZ.setAngle(yaw);
			kalAngleZ = yaw;
		} else 
			kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt); // Calculate the angle using a Kalman filter

		/* Print Data every 2 times */
		if (elasped > 2) {
			// Set the first data
			if (!initialized) {
				initial_kalAngleX = kalAngleX;
				initial_kalAngleY = kalAngleY;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("initial_kalAngleX:%f", initial_kalAngleX); printf(" ");
			printf("kalAngleX-initial_kalAngleX:%f", kalAngleX-initial_kalAngleX); printf(" ");
			printf("\n");

			printf("pitch:%f", pitch); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf(" ");
			printf("initial_kalAngleY: %f", initial_kalAngleY); printf(" ");
			printf("kalAngleY-initial_kalAngleY: %f", kalAngleY-initial_kalAngleY); printf(" ");
			printf("\n");

			printf("yaw:%f", yaw); printf(" ");
			printf("kalAngleZ:%f", kalAngleZ); printf(" ");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = kalAngleX-initial_kalAngleX;
			float _pitch = kalAngleY-initial_kalAngleY;
			float _yaw = kalAngleZ;
			ESP_LOGI(TAG, "roll:%f pitch=%f yaw=%f", _roll, _pitch, _yaw);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = _yaw;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", _yaw);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
