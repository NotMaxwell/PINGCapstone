# Adafruit's BNO08x library, vendored unmodified; see VENDORED.md. It bundles
# CEVA's SH-2 driver and implements its HAL on Arduino's Wire. Upstream ships
# no ESP-IDF build file, so this one registers it as a component.
idf_component_register(
    SRCS
        "src/Adafruit_BNO08x.cpp"
        "src/sh2.c"
        "src/sh2_SensorValue.c"
        "src/sh2_util.c"
        "src/shtp.c"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32 Adafruit_BusIO Adafruit_Sensor)
