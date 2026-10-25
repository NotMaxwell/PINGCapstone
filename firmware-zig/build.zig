//! Builds the firmware's Zig code into C for ESP-IDF, and runs the host
//! tests.
//!
//!   zig build test                  host tests: geometry, TF03 and UBX
//!   zig build firmware -Dapp=main -Didf-cflags=FILE
//!                                   one application as C, for ESP-IDF
//!
//! `idf.py build` runs the second for you (see main/CMakeLists.txt); FILE is
//! the response file of include paths and macros it writes for translate-c.
//!
//! # Why C
//!
//! Upstream Zig's LLVM has no Xtensa backend, so Zig cannot produce ESP32-S3
//! machine code itself. Its C backend can, by way of ESP-IDF's GCC: Zig
//! compiles each application to one C file, and the ESP-IDF build compiles
//! that alongside its own components and links it all.
//!
//! The C is generated for `arm-freestanding-eabi`. Zig's Xtensa target caps C
//! type alignment at 4 bytes (`std.Target.cTypeAlignment`), while the Xtensa
//! ABI that GCC follows aligns `long long` and `double` to 8, so C generated
//! for Xtensa fails its own layout checks. The ARM EABI has the same C ABI as
//! GCC's Xtensa one in every respect C code can see (ILP32, little endian,
//! unsigned `char`, 8-byte aligned 64-bit types, 64-bit `long double`), and
//! the generated C asserts each type's size and alignment, so if the two ever
//! differ the ESP-IDF build fails rather than the firmware misbehaving. No ARM
//! code is produced: the C backend emits portable C and GCC compiles it for
//! the ESP32-S3.
//!
//! The C headers themselves are translated for Xtensa, so their
//! preprocessor sees the target it was written for.

const std = @import("std");

const apps = [_][]const u8{ "main", "gps_demo", "imu_demo", "lidar_demo", "lcd_demo" };

pub fn build(b: *std.Build) void {
    const optimize = b.standardOptimizeOption(.{});

    // ------------------------------------------------------------------
    // Firmware
    // ------------------------------------------------------------------

    const app = b.option([]const u8, "app", "Application to build: main, gps_demo, imu_demo, lidar_demo or lcd_demo") orelse "main";
    const idf_cflags = b.option([]const u8, "idf-cflags", "Response file of ESP-IDF include paths and macros, written by main/CMakeLists.txt");

    const firmware_step = b.step("firmware", "Build one application as C for ESP-IDF");
    if (!containsString(&apps, app)) {
        firmware_step.dependOn(&b.addFail(b.fmt("-Dapp={s}: there is no src/apps/{s}.zig", .{ app, app })).step);
    } else if (idf_cflags) |cflags| {
        buildFirmware(b, firmware_step, app, cflags, optimize);
    } else {
        firmware_step.dependOn(&b.addFail("-Didf-cflags is required; idf.py build passes it").step);
    }

    // ------------------------------------------------------------------
    // Host tests
    // ------------------------------------------------------------------

    const host = b.graph.host;
    const test_step = b.step("test", "Run the host tests");

    for ([_][]const u8{ "src/geo.zig", "src/tf03_protocol.zig" }) |path| {
        const tests = b.addTest(.{ .root_module = b.createModule(.{
            .root_source_file = b.path(path),
            .target = host,
            .optimize = optimize,
        }) });
        test_step.dependOn(&b.addRunArtifact(tests).step);
    }

    // The UBX code runs on u-blox's own C, built for the host.
    const ubx_host = b.addTranslateC(.{
        .root_source_file = b.path("src/c/ubx.h"),
        .target = host,
        .optimize = optimize,
    });
    ubx_host.addIncludePath(b.path("components/ubxlib_ubx/include"));
    const ubx_tests_module = b.createModule(.{
        .root_source_file = b.path("src/ubx.zig"),
        .target = host,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{.{ .name = "ubx", .module = ubx_host.createModule() }},
    });
    ubx_tests_module.addIncludePath(b.path("components/ubxlib_ubx/include"));
    ubx_tests_module.addCSourceFile(.{ .file = b.path("components/ubxlib_ubx/src/u_ubx_protocol.c") });
    const ubx_tests = b.addTest(.{ .root_module = ubx_tests_module });
    test_step.dependOn(&b.addRunArtifact(ubx_tests).step);
}

fn containsString(haystack: []const []const u8, needle: []const u8) bool {
    for (haystack) |item| {
        if (std.mem.eql(u8, item, needle)) return true;
    }
    return false;
}

fn buildFirmware(b: *std.Build, step: *std.Build.Step, app: []const u8, cflags: []const u8, optimize: std.builtin.OptimizeMode) void {
    // What the C headers are translated for: the ESP32-S3's core.
    const xtensa = b.resolveTargetQuery(.{
        .cpu_arch = .xtensa,
        .cpu_model = .{ .explicit = &std.Target.xtensa.cpu.esp32 },
        .os_tag = .freestanding,
    });
    // What the Zig code is compiled for: C with the same ABI (see above).
    const c_abi = b.resolveTargetQuery(.{
        .cpu_arch = .arm,
        .os_tag = .freestanding,
        .abi = .eabi,
        .ofmt = .c,
    });

    const translate = struct {
        fn module(owner: *std.Build, header: []const u8, target: std.Build.ResolvedTarget, flags: []const u8, mode: std.builtin.OptimizeMode, c_target: std.Build.ResolvedTarget) *std.Build.Module {
            const tc = owner.addTranslateC(.{
                .root_source_file = owner.path(header),
                .target = target,
                .optimize = mode,
                .link_libc = false,
            });
            tc.addCFlags(&.{owner.fmt("@{s}", .{flags})});
            return owner.createModule(.{
                .root_source_file = tc.getOutput(),
                .target = c_target,
                .optimize = mode,
            });
        }
    }.module;

    const idf = translate(b, "src/c/idf.h", xtensa, cflags, optimize, c_abi);
    const u8g2 = translate(b, "src/c/u8g2.h", xtensa, cflags, optimize, c_abi);
    const sh2 = translate(b, "src/c/sh2.h", xtensa, cflags, optimize, c_abi);
    const ubx = translate(b, "src/c/ubx.h", xtensa, cflags, optimize, c_abi);

    const ping = b.createModule(.{
        .root_source_file = b.path("src/ping.zig"),
        .target = c_abi,
        .optimize = optimize,
        .imports = &.{
            .{ .name = "idf", .module = idf },
            .{ .name = "u8g2", .module = u8g2 },
            .{ .name = "sh2", .module = sh2 },
            .{ .name = "ubx", .module = ubx },
        },
    });

    const obj = b.addObject(.{
        .name = app,
        .root_module = b.createModule(.{
            .root_source_file = b.path(b.fmt("src/apps/{s}.zig", .{app})),
            .target = c_abi,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "ping", .module = ping },
                .{ .name = "idf", .module = idf },
            },
        }),
    });

    step.dependOn(&b.addInstallFile(obj.getEmittedBin(), "ping.c").step);
    step.dependOn(&b.addInstallFile(b.graph.path(.zig_lib, "zig.h"), "include/zig.h").step);
}
