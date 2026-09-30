# Adafruit Unified Sensor, vendored unmodified; see VENDORED.md. Upstream
# ships no ESP-IDF build file, so this one registers it as a component.
idf_component_register(
    SRCS "Adafruit_Sensor.cpp"
    INCLUDE_DIRS "."
    REQUIRES arduino-esp32)
