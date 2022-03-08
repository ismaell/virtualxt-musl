// Copyright (c) 2019-2022 Andreas T Jonsson
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.

const std = @import("std");
const builtin = @import("builtin");

const Builder = std.build.Builder;
const print = std.debug.print;

const expected_zig_version: u32 = 9;

const c_options = &[_][]const u8{
    "-Wall",
    "-Wextra",
    "-Werror",
};

const CTest = struct {
    name: []const u8,
    offset: usize
};

fn find_test(buffer: []const u8, start: usize) ?CTest {
    const prefix = std.mem.indexOfPos(u8, buffer, start, "\nTEST(") orelse return null;
    const sufix = std.mem.indexOfPos(u8, buffer, prefix, ",") orelse return null;
    return CTest{.name = buffer[prefix+6..sufix], .offset = sufix};
}

fn create_test_files() anyerror!void {
    const cwd = std.fs.cwd();
    try cwd.makePath("test");

    var h_out = try cwd.createFile("test/test.h", .{.truncate = true});
    defer h_out.close();
    try h_out.writer().print("#include <testing.h>\n\n", .{});

    var zig_out = try cwd.createFile("test/test.zig", .{.truncate = true});
    defer zig_out.close();
    try zig_out.writer().print("const std = @import(\"std\");\nconst c = @cImport({{\n    @cInclude(\"testing.h\");\n    @cInclude(\"test.h\");\n}});\n\n", .{});
}

fn parse_file(name: []const u8) anyerror!void {
    var gp = std.heap.GeneralPurposeAllocator(.{.safety = true}){};
    defer _ = gp.deinit();
    const allocator = gp.allocator();

    const cwd = std.fs.cwd();

    var file = try cwd.openFile(name, .{});
    defer file.close();

    const megabyte = 1024*1024;
    const file_buffer = try file.readToEndAlloc(allocator, megabyte);
    defer allocator.free(file_buffer);

    var h_out = try cwd.openFile("test/test.h", .{.write = true});
    defer h_out.close();
    try h_out.seekFromEnd(0);

    var zig_out = try cwd.openFile("test/test.zig", .{.write = true});
    defer zig_out.close();
    try zig_out.seekFromEnd(0);

    var start: usize = 0;
    while (true) {
        const test_name = find_test(file_buffer, start) orelse break;
        try h_out.writer().print("extern int test_{s}(struct Test T);\n", .{test_name.name});
        try zig_out.writer().print("test \"{s}\" {{\n    try std.testing.expect(c.run_test(c.test_{s}));\n}}\n", .{test_name.name, test_name.name});
        start = test_name.offset;
    }
}

fn build_libvxt(b: *Builder, mode: std.builtin.Mode, target: std.zig.CrossTarget, testing: bool) *std.build.LibExeObjStep {
    const libname = b.fmt("vxt.{s}.{s}", .{@tagName(target.getOsTag()), @tagName(target.getCpuArch())});
    const lib = b.addStaticLibrary(libname, null);

    lib.addIncludeDir("lib/vxt/include");
    lib.setTarget(target);

    if (testing) {
        lib.defineCMacroRaw("TESTING");
        lib.defineCMacroRaw("VXT_CPU_286");
        lib.linkLibC();
        lib.setBuildMode(.ReleaseSafe);
        create_test_files() catch unreachable;
    } else {
        lib.setBuildMode(mode);
        if (!target.toTarget().isWasm()) {
            lib.setOutputDir("build/lib");
            lib.install();
        }
    }

    const opt = c_options ++ &[_][]const u8{
        "-Wstrict-prototypes",
        "-Wcast-align",
        "-Wno-unused-function", // Needed for unit tests.
        "-std=c11",
        "-pedantic",
    };

    const files = &[_][]const u8{
        "lib/vxt/system.c",
        "lib/vxt/dummy.c",
        "lib/vxt/memory.c",
        "lib/vxt/cpu.c",
        "lib/vxt/pic.c",
        "lib/vxt/debugger.c",
        //"lib/vxt/testsuit.c",
    };

    for (files) |file| {
        lib.addCSourceFile(file, opt);
        if (testing) {
            parse_file(file) catch unreachable;
        }
    }
    return lib;
}

fn assert_version(major: u32, minor: u32) void {
    const a = builtin.zig_version.major;
    const b = builtin.zig_version.minor;
    if (a != major or b != minor) {
        std.debug.panic("invalid Zig version, expected {}.{} but got {}.{}", .{ major, minor, a, b });
    }
}

pub fn build(b: *Builder) void {
    assert_version(0, expected_zig_version);

    //const textmode = b.option(bool, "textmode", "Build for textmode only") orelse false;
    const validator = b.option(bool, "validator", "Enable PI8088 hardware validator") orelse false;
    const sdl_path = b.option([]const u8, "sdl-path", "Path to SDL2 headers and libs") orelse null;

    const mode = b.standardReleaseOptions();
    const target = b.standardTargetOptions(.{});
    //const wasm = target.toTarget().isWasm();

    // -------- libvxt --------

    const libvxt = build_libvxt(b, mode, target, false);
    b.step("lib", "Build libvxt").dependOn(&libvxt.step);

    // -------- termbox --------

    const termbox = b.addStaticLibrary("termbox", null);
    termbox.setBuildMode(mode);
    termbox.setTarget(target);
    termbox.linkLibC();
    termbox.addIncludeDir("lib/termbox/src");

    termbox.addCSourceFile("lib/termbox/src/termbox.c", c_options);
    termbox.addCSourceFile("lib/termbox/src/utf8.c", c_options);

    // -------- microui --------

    const microui = b.addStaticLibrary("microui", null);
    microui.setBuildMode(mode);
    microui.setTarget(target);
    microui.linkLibC();
    microui.addIncludeDir("lib/microui/src");
    microui.addCSourceFile("lib/microui/src/microui.c", c_options ++ &[_][]const u8{"-std=c11", "-pedantic"});

    // -------- virtualxt sdl --------

    const exe_sdl = b.addExecutable("virtualxt", "front/sdl/main.zig");
    exe_sdl.addIncludeDir("front/sdl");
    exe_sdl.defineCMacroRaw("ENTRY=c_main");

    exe_sdl.setBuildMode(mode);
    exe_sdl.setTarget(target);
    exe_sdl.setOutputDir("build/bin");
    exe_sdl.defineCMacroRaw(b.fmt("PLATFORM={s}", .{@tagName(target.getOsTag())}));

    if (sdl_path != null) {
        const p = .{sdl_path};
        print("SDL2 location: {s}\n", p);

        exe_sdl.addIncludeDir(b.fmt("{s}/include", p));

        if (target.isWindows() and target.getAbi() == .gnu) {
            exe_sdl.addLibPath(b.fmt("{s}/lib/x64", p));
        } else {
            exe_sdl.addLibPath(b.fmt("{s}/lib", p));
        }
    }

    exe_sdl.linkSystemLibrary("SDL2");

    exe_sdl.linkLibrary(libvxt);
    exe_sdl.addIncludeDir("lib/vxt/include");

    exe_sdl.linkLibC();

    const opt = c_options ++ &[_][]const u8{"-std=c11", "-pedantic"};
    exe_sdl.addCSourceFile("front/sdl/main.c", opt);
    exe_sdl.addCSourceFile("front/sdl/docopt.c", &[_][]const u8{"-std=c11", "-Wno-unused-variable", "-Wno-unused-parameter"});

    if (validator) {
        exe_sdl.linkSystemLibrary("gpiod");
        exe_sdl.defineCMacroRaw("PI8088");
        exe_sdl.addCSourceFile("tools/validator/pi8088/pi8088.c", opt);
    }

    // -------- virtualxt libretro --------

    {
        const libname = b.fmt("vxt-libretro.{s}.{s}", .{@tagName(target.getOsTag()), @tagName(target.getCpuArch())});
        const libretro = b.addSharedLibrary(libname, null, .unversioned);
        libretro.setBuildMode(mode);
        libretro.setTarget(target);
        libretro.setOutputDir("build/lib");

        libretro.linkLibrary(build_libvxt(b, mode, target, false));
        libretro.addIncludeDir("lib/vxt/include");
        libretro.addIncludeDir("lib/vxt");

        libretro.addIncludeDir("lib/libretro");

        libretro.addCSourceFile("front/libretro/core.c", c_options ++ &[_][]const u8{"-std=c11", "-pedantic"});

        b.step("libretro", "Build libretro core").dependOn(&libretro.step);
    }

    // -------- scrambler --------

    {
        const scrambler = b.addExecutable("scrambler", null);
        scrambler.setBuildMode(mode);
        scrambler.setTarget(target);
        scrambler.setOutputDir("build/bin");
        scrambler.linkLibC();

        scrambler.linkSystemLibrary("gpiod");
        scrambler.defineCMacroRaw("PI8088");

        scrambler.linkLibrary(build_libvxt(b, mode, target, false));
        scrambler.addIncludeDir("lib/vxt/include");
        scrambler.addIncludeDir("lib/vxt");        

        scrambler.addCSourceFile("tools/validator/pi8088/scrambler.c", opt);
        scrambler.addCSourceFile("tools/validator/pi8088/pi8088.c", opt);

        b.step("scrambler", "Build scrambler for RaspberryPi").dependOn(&scrambler.step);
    }

    // -------- test --------

    {
        const tests = b.addTest("test/test.zig");
        tests.setBuildMode(mode);
        tests.setTarget(target);

        tests.linkLibC();
        tests.defineCMacroRaw("TESTING");
        tests.defineCMacroRaw("VXT_CPU_286");
        tests.addIncludeDir("test");

        tests.linkLibrary(build_libvxt(b, mode, target, true));
        tests.addIncludeDir("lib/vxt/include");
        tests.addIncludeDir("lib/vxt");

        b.step("test", "Run all libvxt tests").dependOn(&tests.step);

        // -------- check --------

        {
            const check = b.addSystemCommand(&[_][]const u8{"cppcheck", "--enable=style", "lib/vxt"});
            check.step.dependOn(&tests.step);
            b.step("check", "Run CppCheck on libvxt").dependOn(&check.step);
        }
    }

    // -------- doc --------

    {
        const doc = b.addSystemCommand(&[_][]const u8{"doxygen", ".doxygen"});
        doc.cwd = "lib/vxt/include";
        b.step("doc", "Generate libvxt API documentation").dependOn(&doc.step);
    }

    // -------- run --------

    const run_cmd = exe_sdl.run();
    run_cmd.step.dependOn(&exe_sdl.step);
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    b.step("run", "Run VirtualXT").dependOn(&run_cmd.step);

    // -------- artifact --------

    b.default_step.dependOn(&exe_sdl.step);
    b.installArtifact(exe_sdl);
}
