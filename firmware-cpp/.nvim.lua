-- Project config for Neovim 0.11+, loaded when 'exrc' is on (`vim.o.exrc = true`
-- in your init.lua; Neovim asks once whether to trust this file).
--
-- Start Neovim from a shell that sourced ESP-IDF's export.sh, so `clangd` is
-- Espressif's esp-clangd and idf.py, openocd and the Xtensa GDB are on PATH.
-- BUILD.md, "Editors", has the details.
--
-- What it sets up:
--   * clangd for C/C++ (uses .clangd and the compile_commands.json from the
--     firmware and host builds),
--   * :Ping* commands for building, flashing, monitoring, testing, debugging,
--   * nvim-dap (if installed): the configurations in .vscode/launch.json,
--     through the cpptools adapter (mason: `:MasonInstall cpptools`), plus a
--     native GDB-DAP fallback that needs nothing but the Xtensa GDB.

local root = vim.fn.fnamemodify(debug.getinfo(1, "S").source:sub(2), ":p:h")

-- clangd ---------------------------------------------------------------------
if vim.lsp.config then
  vim.lsp.config("ping_clangd", {
    cmd = { "clangd", "--background-index", "--clang-tidy", "--header-insertion=never" },
    filetypes = { "c", "cpp" },
    root_markers = { ".clangd" },
    root_dir = root,
  })
  vim.lsp.enable("ping_clangd")
end

-- Commands -------------------------------------------------------------------
local function term(cmd)
  vim.cmd("botright split | resize 15")
  vim.fn.jobstart(cmd, { cwd = root, term = true })
  vim.cmd("startinsert")
end

local presets = { "default", "gps_demo", "imu_demo", "lidar_demo", "lcd_demo", "main-debug", "main-gcc" }
local function preset_arg(opts)
  return (opts.args ~= "" and opts.args) or "default"
end
local function complete_presets()
  return presets
end

vim.api.nvim_create_user_command("PingBuild", function(o)
  term("idf.py --preset " .. preset_arg(o) .. " build")
end, { nargs = "?", complete = complete_presets, desc = "Build firmware for a preset" })

vim.api.nvim_create_user_command("PingFlash", function(o)
  term("idf.py --preset " .. preset_arg(o) .. " flash monitor")
end, { nargs = "?", complete = complete_presets, desc = "Flash a preset and open the monitor" })

vim.api.nvim_create_user_command("PingMonitor", function(o)
  term("idf.py --preset " .. preset_arg(o) .. " monitor")
end, { nargs = "?", complete = complete_presets, desc = "Serial monitor (Ctrl-] quits)" })

vim.api.nvim_create_user_command("PingTest", function(o)
  local p = (o.args ~= "" and o.args) or "host"
  term(("cmake --preset %s && cmake --build --preset %s && ctest --preset %s"):format(p, p, p))
end, { nargs = "?", complete = function() return { "host", "host-asan" } end, desc = "Host unit tests" })

vim.api.nvim_create_user_command("PingOpenOCD", function()
  term("openocd -f board/esp32s3-builtin.cfg")
end, { desc = "GDB server on :3333 through the built-in USB-JTAG" })

-- Debugging ------------------------------------------------------------------
local ok, dap = pcall(require, "dap")
if ok then
  local elf = root .. "/build/main-debug/ping_capstone.elf"

  -- cpptools' OpenDebugAD7 understands .vscode/launch.json as VS Code does.
  if not dap.adapters.cppdbg then
    local mason = vim.fn.stdpath("data") .. "/mason/packages/cpptools/extension/debugAdapters/bin/OpenDebugAD7"
    if vim.fn.executable(mason) == 1 then
      dap.adapters.cppdbg = { id = "cppdbg", type = "executable", command = mason }
    end
  end
  local vscode = require("dap.ext.vscode")
  if vscode.load_launchjs then
    -- Recent nvim-dap reads .vscode/launch.json by itself; older ones need this.
    pcall(vscode.load_launchjs, root .. "/.vscode/launch.json", { cppdbg = { "c", "cpp" } })
  end

  -- Fallback without cpptools: GDB 14+ speaks DAP itself. Run :PingOpenOCD
  -- first, then type `monitor reset halt` in the REPL to restart the chip.
  dap.adapters.esp_gdb = {
    type = "executable",
    command = "xtensa-esp32s3-elf-gdb",
    args = { "--interpreter=dap", "--eval-command", "set remotetimeout 20" },
  }
  dap.configurations.cpp = dap.configurations.cpp or {}
  table.insert(dap.configurations.cpp, {
    name = "ESP32-S3: attach via GDB-DAP (OpenOCD :3333)",
    type = "esp_gdb",
    request = "attach",
    program = elf,
    target = "localhost:3333",
    cwd = root,
  })
end
