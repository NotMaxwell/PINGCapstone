# Bud Ryerson's TFMini-Plus library, vendored unmodified; see VENDORED.md.
# Upstream ships no ESP-IDF build file, so this one registers it as a
# component.
idf_component_register(
    SRCS "src/TFMPlus.cpp"
    INCLUDE_DIRS "src"
    REQUIRES arduino-esp32)
