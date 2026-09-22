# Wraps the vendor Arduino libraries pinned under external/ as one ESP-IDF
# component. Only Adafruit_BusIO ships its own CMakeLists.txt, so rather than
# patch five upstream repos this lists their sources directly. Bumping a
# library is a `git -C external/<lib> checkout <tag>` away.
set(ext "${CMAKE_CURRENT_LIST_DIR}/../../external")

idf_component_register(
    SRCS
        # Adafruit's BNO08x driver, which bundles CEVA's SH-2 reference library.
        "${ext}/Adafruit_BNO08x/src/Adafruit_BNO08x.cpp"
        "${ext}/Adafruit_BNO08x/src/sh2.c"
        "${ext}/Adafruit_BNO08x/src/sh2_SensorValue.c"
        "${ext}/Adafruit_BNO08x/src/sh2_util.c"
        "${ext}/Adafruit_BNO08x/src/shtp.c"
        "${ext}/Adafruit_BusIO/Adafruit_I2CDevice.cpp"
        "${ext}/Adafruit_BusIO/Adafruit_SPIDevice.cpp"
        "${ext}/Adafruit_BusIO/Adafruit_BusIO_Register.cpp"
        "${ext}/Adafruit_BusIO/Adafruit_GenericDevice.cpp"
        "${ext}/Adafruit_Sensor/Adafruit_Sensor.cpp"
        # SparkFun's u-blox GNSS library (SparkFun make the NEO-M9N breakout).
        "${ext}/SparkFun_u-blox_GNSS_v3/src/u-blox_GNSS.cpp"
        "${ext}/SparkFun_u-blox_GNSS_v3/src/sfe_bus.cpp"
        # Benewake frame parser; the TF03 uses the same 9-byte 0x59 0x59 frame.
        "${ext}/TFMini-Plus/src/TFMPlus.cpp"
    INCLUDE_DIRS
        "${ext}/Adafruit_BNO08x/src"
        "${ext}/Adafruit_BusIO"
        "${ext}/Adafruit_Sensor"
        "${ext}/SparkFun_u-blox_GNSS_v3/src"
        "${ext}/TFMini-Plus/src")

# Third-party code: its warnings are not ours to fix, and ESP-IDF promotes
# several of them to errors.
target_compile_options(${COMPONENT_LIB} PRIVATE -w)
