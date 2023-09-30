idf_component_register(
  REQUIRES dht11
  REQUIRES esp_timer driver
  SRCS "dht11.c"
  INCLUDE_DIRS "."
)
