// ubxlib's UBX protocol and GNSS definitions, for translate-c. The build
// turns this into the `ubx` module, for the firmware and for the host tests.
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <u_error_common.h>
#include <u_ubx_protocol.h>
#include <u_gnss_cfg_val_key.h>
#include <u_gnss_dec_ubx_nav_pvt.h>
