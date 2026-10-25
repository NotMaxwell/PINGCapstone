//! The few GPIO operations the drivers need, without ESP-IDF's
//! `gpio_config_t`.

const idf = @import("idf");

pub const Pull = enum { none, up, down };

/// Makes `pin` an output, already at `level` when it becomes one.
pub fn output(pin: idf.gpio_num_t, level: u1) void {
    _ = idf.gpio_reset_pin(pin);
    _ = idf.gpio_set_level(pin, level);
    _ = idf.gpio_set_direction(pin, idf.GPIO_MODE_OUTPUT);
}

/// Makes `pin` an input with the given pull resistor.
pub fn input(pin: idf.gpio_num_t, pull: Pull) void {
    _ = idf.gpio_reset_pin(pin);
    _ = idf.gpio_set_direction(pin, idf.GPIO_MODE_INPUT);
    _ = idf.gpio_set_pull_mode(pin, switch (pull) {
        .none => idf.GPIO_FLOATING,
        .up => idf.GPIO_PULLUP_ONLY,
        .down => idf.GPIO_PULLDOWN_ONLY,
    });
}

/// Returns `pin` to its default state: an input with a weak pull-up.
pub fn release(pin: idf.gpio_num_t) void {
    _ = idf.gpio_reset_pin(pin);
}

pub fn write(pin: idf.gpio_num_t, level: u1) void {
    _ = idf.gpio_set_level(pin, level);
}

pub fn read(pin: idf.gpio_num_t) u1 {
    return @intCast(idf.gpio_get_level(pin));
}
