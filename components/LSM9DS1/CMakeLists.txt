idf_component_register(
	SRCS "LSM9DS1.cpp"
	INCLUDE_DIRS "."
	PRIV_REQUIRES driver esp_driver_i2c esp_timer
)
