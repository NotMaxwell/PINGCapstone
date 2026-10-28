# The one ESP-IDF component of the project. Its sources are the C helpers in
# idf_config.c, the 128-bit integer helpers in int128.c, and ping.c, which `zig build firmware` generates from the
# application chosen with PING_APP and everything under ../src. The Zig build
# needs to know where ESP-IDF's headers are and what macros they expect, so
# this writes a response file of the component's own include directories and
# definitions and passes it along.
#
# Which application to build: the main program, or one of the demos that
# brings up a single peripheral and shows what it reads on the display. Each
# file in ../src/apps is a complete program with its own app_main, as each
# Rust [[bin]] is.
#
#   idf.py -B build/gps_demo -DPING_APP=gps_demo flash monitor
set(PING_APP "main" CACHE STRING "Application to build: main, gps_demo, imu_demo, lidar_demo or lcd_demo")
set_property(CACHE PING_APP PROPERTY STRINGS main gps_demo imu_demo lidar_demo lcd_demo)

get_filename_component(zig_project "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
if(NOT EXISTS "${zig_project}/src/apps/${PING_APP}.zig")
    message(FATAL_ERROR "PING_APP=${PING_APP}: there is no src/apps/${PING_APP}.zig")
endif()

find_program(ZIG zig DOC "The Zig compiler; ../mise.toml pins the version")
if(NOT ZIG)
    message(FATAL_ERROR "zig was not found on PATH. Install it with mise (see ../mise.toml) or pass -DZIG=/path/to/zig")
endif()

# Where the Zig build puts what it makes, inside this build directory so each
# application has its own.
set(zig_out "${CMAKE_CURRENT_BINARY_DIR}/zig")
set(ping_c "${zig_out}/ping.c")
set(zig_h "${zig_out}/include/zig.h")
set(cflags_rsp "${CMAKE_CURRENT_BINARY_DIR}/zig-idf-cflags.rsp")
file(MAKE_DIRECTORY "${zig_out}/include")

idf_component_register(
    SRCS "idf_config.c" "int128.c" "${ping_c}"
    INCLUDE_DIRS "include" "${zig_out}/include"
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_spi esp_driver_uart esp_rom esp_system esp_timer
             freertos log sh2 ubxlib_ubx u8g2)
# ping.c is machine-written, and the C backend's idioms trip ESP-IDF's
# -Werror set: unused static constants, comparisons that are always true for
# a type, its own printf declaration, and pointers into tagged unions' packed
# payloads (which it aligns to 8 with zig_align, so they are not unaligned).
# The Zig compiler has already checked the code, so its C is compiled without
# warnings.
set_source_files_properties("${ping_c}" PROPERTIES GENERATED TRUE COMPILE_OPTIONS "-w")

# The display driver finds its way back from u8g2's callbacks to the Display
# through u8g2's user pointer, which u8g2 only compiles in on request. PUBLIC,
# so the headers are translated for Zig with the same layout.
idf_component_get_property(u8g2_lib u8g2 COMPONENT_LIB)
target_compile_definitions(${u8g2_lib} PUBLIC U8X8_WITH_USER_PTR)

# The response file. The include directories and definitions are this
# component's, which carry everything REQUIRES and ESP-IDF itself add. IDF_VER's
# value is a quoted string, which a response file would unquote, and nothing
# translated uses it.
#
# The rest comes from asking GCC itself, with ESP-IDF's toolchain flags (which
# pick the C library: -specs=picolibc.specs by default):
#
# - Its system include directories, in its order: the C library's, then GCC's
#   own (stddef.h, stdint.h, ...). Zig's translate-c brings its own versions of
#   GCC's headers, which skip the __need_* protocol the C library relies on,
#   and searches them before any -isystem directory, so these go in as -I.
# - The -D flags in the toolchain flags (picolibc's errno function and so on).
# - GCC's Xtensa-specific predefined macros (__XTENSA__, __XTENSA_EL__, the
#   __XCHAL_* core description). The headers test them, and Zig's translate-c
#   does not define them for its Xtensa target.
set(toolchain_cflags "@${IDF_TOOLCHAIN_BUILD_DIR}/cflags")
execute_process(COMMAND "${CMAKE_C_COMPILER}" "${toolchain_cflags}" -E -v -x c /dev/null
                OUTPUT_QUIET ERROR_VARIABLE gcc_verbose)
string(REGEX MATCH "#include <\\.\\.\\.> search starts here:\n(.*)\nEnd of search list" _ "${gcc_verbose}")
string(REGEX REPLACE "\n *" ";" gcc_system_includes "${CMAKE_MATCH_1}")
set(system_includes "")
foreach(dir IN LISTS gcc_system_includes)
    string(STRIP "${dir}" dir)
    if(dir)
        get_filename_component(dir "${dir}" REALPATH)
        string(APPEND system_includes "-I${dir}\n")
    endif()
endforeach()
if(NOT system_includes)
    message(FATAL_ERROR "Could not read the system include directories from ${CMAKE_C_COMPILER} -v")
endif()

file(STRINGS "${IDF_TOOLCHAIN_BUILD_DIR}/cflags" toolchain_flags)
set(toolchain_defines "")
foreach(flag IN LISTS toolchain_flags)
    if(flag MATCHES "^-D")
        string(APPEND toolchain_defines "${flag}\n")
    endif()
endforeach()

execute_process(COMMAND "${CMAKE_C_COMPILER}" "${toolchain_cflags}" -dM -E -x c /dev/null
                OUTPUT_VARIABLE predefined)
string(REGEX MATCHALL "#define __(XTENSA|XCHAL|xtensa)[^\n]*" xtensa_macros "${predefined}")
set(xtensa_defines "")
foreach(macro IN LISTS xtensa_macros)
    string(REGEX REPLACE "^#define ([^ ]+) (.*)$" "-D\\1=\\2" define "${macro}")
    string(APPEND xtensa_defines "${define}\n")
endforeach()
set(includes "$<TARGET_PROPERTY:${COMPONENT_LIB},INCLUDE_DIRECTORIES>")
set(defines "$<FILTER:$<TARGET_PROPERTY:${COMPONENT_LIB},COMPILE_DEFINITIONS>,EXCLUDE,^IDF_VER=>")
file(GENERATE OUTPUT "${cflags_rsp}" CONTENT
"$<$<BOOL:${includes}>:-I$<JOIN:${includes},\n-I>\n>$<$<BOOL:${defines}>:-D$<JOIN:${defines},\n-D>\n>${toolchain_defines}${xtensa_defines}${system_includes}")

file(GLOB_RECURSE zig_sources CONFIGURE_DEPENDS
     "${zig_project}/src/*.zig" "${zig_project}/src/c/*.h")
add_custom_command(
    OUTPUT "${ping_c}" "${zig_h}"
    COMMAND "${ZIG}" build firmware
            "-Dapp=${PING_APP}" -Doptimize=ReleaseSafe "-Didf-cflags=${cflags_rsp}"
            --prefix "${zig_out}"
    WORKING_DIRECTORY "${zig_project}"
    DEPENDS ${zig_sources} "${zig_project}/build.zig" "${zig_project}/build.zig.zon" "${cflags_rsp}"
    COMMENT "Compiling the Zig firmware (${PING_APP}) to C"
    VERBATIM)
