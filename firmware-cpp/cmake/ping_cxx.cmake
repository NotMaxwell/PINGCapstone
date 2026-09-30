# Compiler settings shared by our own components (not the vendored ones).
#
# ESP-IDF compiles C++ as -std=gnu++26 by default. We pin our code to
# ISO C++23, the newest standard the toolchain supports in full, and turn on
# the warnings we want to keep clean. Vendored C libraries keep ESP-IDF's
# defaults so they build exactly as upstream intends.

function(ping_cxx_component target)
    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-std=c++23>
        -Wall
        -Wextra
        -Wshadow
        # Full -Wconversion also fires inside ESP-IDF's own headers, which
        # are not included as system headers; the float half is what
        # matters for the geometry.
        -Wfloat-conversion
        -Wimplicit-fallthrough)
endfunction()
