//! FreeRTOS and ESP-IDF glue: time, delays, tasks, a mutex, ESP-IDF error
//! codes as Zig errors, and the `std.log` and panic hooks every application
//! installs.
//!
//! Several FreeRTOS calls are macros in C (`pdMS_TO_TICKS`,
//! `xSemaphoreCreateMutex`, `xSemaphoreTake`, ...). translate-c cannot turn
//! all of those into Zig, so this calls the functions they expand to.

const std = @import("std");
const idf = @import("idf");

/// Milliseconds as FreeRTOS ticks, rounded down as `pdMS_TO_TICKS` does.
pub fn ticks(ms: u32) idf.TickType_t {
    return @intCast(@as(u64, ms) * idf.configTICK_RATE_HZ / 1000);
}

/// Blocks the calling task for at least `ms` milliseconds.
pub fn delayMs(ms: u32) void {
    idf.vTaskDelay(ticks(ms));
}

/// Microseconds since boot.
pub fn nowUs() i64 {
    return idf.esp_timer_get_time();
}

/// Milliseconds since boot.
pub fn nowMs() i64 {
    return @divTrunc(nowUs(), 1000);
}

/// Starts `function` in a new FreeRTOS task on whichever core is free.
pub fn spawn(function: fn (?*anyopaque) callconv(.c) void, name: [*:0]const u8, stack_bytes: u32, priority: u32) void {
    const created = idf.xTaskCreatePinnedToCore(function, name, stack_bytes, null, priority, null, idf.tskNO_AFFINITY);
    if (created != idf.pdPASS) std.debug.panic("could not start task {s}", .{name});
}

/// A FreeRTOS mutex, which unlike a spinlock lets a task waiting for it
/// sleep.
pub const Mutex = struct {
    handle: idf.SemaphoreHandle_t = null,

    /// Creates the FreeRTOS mutex. Static initialisers cannot call FreeRTOS,
    /// so a mutex that tasks share is created here, in app_main, before those
    /// tasks are started. `lock` creates it too, for a mutex only one task
    /// ever uses.
    pub fn init(mutex: *Mutex) void {
        if (mutex.handle == null) {
            mutex.handle = idf.xQueueCreateMutex(idf.queueQUEUE_TYPE_MUTEX) orelse
                @panic("out of memory for a mutex");
        }
    }

    pub fn lock(mutex: *Mutex) void {
        if (mutex.handle == null) mutex.init();
        _ = idf.xQueueSemaphoreTake(mutex.handle, idf.portMAX_DELAY);
    }

    pub fn unlock(mutex: *Mutex) void {
        _ = idf.xQueueGenericSend(mutex.handle, null, 0, idf.queueSEND_TO_BACK);
    }
};

/// ESP-IDF's error codes, as Zig errors. `@errorName` gives the same names
/// as `esp_err_to_name`, without the `ESP_ERR_` prefix.
pub const EspError = error{
    Fail,
    NoMem,
    InvalidArg,
    InvalidState,
    InvalidSize,
    NotFound,
    NotSupported,
    Timeout,
    InvalidResponse,
    InvalidCrc,
    InvalidVersion,
    InvalidMac,
    NotFinished,
    NotAllowed,
};

/// Turns an `esp_err_t` into a Zig error. Codes without a name of their own
/// above come out as `error.Fail`.
pub fn check(code: idf.esp_err_t) EspError!void {
    return switch (code) {
        idf.ESP_OK => {},
        idf.ESP_ERR_NO_MEM => error.NoMem,
        idf.ESP_ERR_INVALID_ARG => error.InvalidArg,
        idf.ESP_ERR_INVALID_STATE => error.InvalidState,
        idf.ESP_ERR_INVALID_SIZE => error.InvalidSize,
        idf.ESP_ERR_NOT_FOUND => error.NotFound,
        idf.ESP_ERR_NOT_SUPPORTED => error.NotSupported,
        idf.ESP_ERR_TIMEOUT => error.Timeout,
        idf.ESP_ERR_INVALID_RESPONSE => error.InvalidResponse,
        idf.ESP_ERR_INVALID_CRC => error.InvalidCrc,
        idf.ESP_ERR_INVALID_VERSION => error.InvalidVersion,
        idf.ESP_ERR_INVALID_MAC => error.InvalidMac,
        idf.ESP_ERR_NOT_FINISHED => error.NotFinished,
        idf.ESP_ERR_NOT_ALLOWED => error.NotAllowed,
        else => error.Fail,
    };
}

/// `ESP_ERROR_CHECK`: stops the firmware with the error's name if `result`
/// is an error. For setup that cannot fail on a correctly built board.
pub fn must(result: anytype) void {
    result catch |err| std.debug.panic("{s}", .{@errorName(err)});
}

/// At most `capacity` bytes of formatted text, held by value. Text that does
/// not fit is cut short.
pub fn Text(comptime capacity: usize) type {
    return struct {
        const Self = @This();

        buf: [capacity]u8 = undefined,
        len: usize = 0,

        pub fn init(comptime format: []const u8, args: anytype) Self {
            var text: Self = .{};
            var writer: std.Io.Writer = .fixed(&text.buf);
            writer.print(format, args) catch {};
            text.len = writer.end;
            return text;
        }

        pub fn slice(text: *const Self) []const u8 {
            return text.buf[0..text.len];
        }
    };
}

/// `std.options.logFn` for the applications: each message goes through
/// ESP-IDF's logging, tagged with its `std.log` scope, so it is filtered and
/// timestamped like ESP-IDF's own.
pub fn logFn(
    comptime level: std.log.Level,
    comptime scope: @EnumLiteral(),
    comptime format: []const u8,
    args: anytype,
) void {
    const message: Text(256) = .init(format, args);
    const letter: u8, const esp_level = switch (level) {
        .err => .{ 'E', idf.ESP_LOG_ERROR },
        .warn => .{ 'W', idf.ESP_LOG_WARN },
        .info => .{ 'I', idf.ESP_LOG_INFO },
        .debug => .{ 'D', idf.ESP_LOG_DEBUG },
    };
    const tag = @tagName(scope);
    idf.esp_log_write(
        esp_level,
        tag,
        "%c (%lu) %s: %.*s\n",
        @as(c_int, letter),
        @as(c_ulong, idf.esp_log_timestamp()),
        tag.ptr,
        @as(c_int, @intCast(message.len)),
        &message.buf,
    );
}

/// The panic handler every application installs: ESP-IDF prints the message
/// and a backtrace, then restarts the chip.
pub const panic = std.debug.FullPanic(panicFn);

fn panicFn(message: []const u8, _: ?usize) noreturn {
    var details: [128:0]u8 = @splat(0);
    const len = @min(message.len, details.len);
    @memcpy(details[0..len], message[0..len]);
    idf.esp_system_abort(&details);
}
