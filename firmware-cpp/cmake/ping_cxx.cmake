# Compiler settings shared by our own components (not the vendored ones).
#
# ESP-IDF 5.5 compiles C++ as -std=gnu++2b: C++23 with GNU extensions. Our
# own code builds as strict ISO C++23, with the warnings we want to keep
# clean. The vendored code keeps ESP-IDF's defaults, so it builds the way
# upstream expects.

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
