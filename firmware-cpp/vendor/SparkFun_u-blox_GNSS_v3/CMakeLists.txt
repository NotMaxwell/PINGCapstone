# SparkFun's u-blox GNSS v3 library, vendored unmodified; see VENDORED.md.
# Upstream ships no ESP-IDF build file, so this one registers it as a
# component.
idf_component_register(
    SRCS "src/u-blox_GNSS.cpp" "src/sfe_bus.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32)
