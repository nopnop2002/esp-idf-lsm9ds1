#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

#include "LSM9DS1.h"

LSM9DS1Class imu(CONFIG_MAIN_I2C_ADDR, CONFIG_MAG_I2C_ADDR);

void lsm9ds1(void *pvParameters){
	// Initialize IMU
	if (imu.begin() == 0) {
		ESP_LOGE(TAG, "Connection fail");
		vTaskDelete(NULL);
	}

	while(1){
		// Obtain scaled magnetic data
		if (imu.magneticFieldAvailable()) {
			float mx=0.0, my=0.0, mz=0.0;
			imu.readMagneticField(mx, my, mz);
			ESP_LOGI(TAG, "mag=%f %f %f", mx, my, mz);
			mx = mx + CONFIG_MAG_OFFSET_X;
			my = my + CONFIG_MAG_OFFSET_Y;
			mz = mz + CONFIG_MAG_OFFSET_Z;

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", mx);
			cJSON_AddNumberToObject(request, "pitch", my);
			cJSON_AddNumberToObject(request, "yaw", mz);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);
		} // end if
		vTaskDelay(10);
	} // end while

	// Never reach here
	vTaskDelete(NULL);
}
