## Overview
This project implements a **parameterizable QSPI Controller IP Core** in Verilog that bridges a host processor to QSPI flash memory devices (e.g., Macronix MX25L6436F). The IP supports two mutually exclusive modes: Command Mode (with optional DMA) for programmable transactions, and XIP Mode (Execute-In-Place) for memory-mapped access via an AXI slave interface.

The design is modular, synthesis-ready for FPGA/ASIC, and follows a fully synchronous reset style with an APB CSR slave interface, AXI master for DMA, and AXI slave for XIP.

---

## Agent Roles

### 1. **CSR Agent** (`csr.v`)
- Implements AMBA 3/4 APB-compliant CSR slave (configurable via HAS_PSTRB, APB_WINDOW_LSB).
- Handles register access for:
  - CTRL: Enable, XIP_EN, QUAD_EN, CMD_TRIGGER (W1S), DMA_EN, etc.
  - STATUS: Busy, levels, errors.
  - INT_EN/INT_STAT: Interrupts for CMD_DONE, DMA_DONE, ERR, FIFO events.
  - CLK_DIV, CS_CTRL, XIP_CFG, CMD_CFG, DMA_CFG, etc.
- Parameterizable address width and optional WP bit.
- Ensures synchronous read/write timing with reserved-bit masks and RO validation.

---

### 2. **Command Engine (CE) Agent** (`cmd_engine.v`)
- Interprets CMD_* registers and triggers QSPI transactions.
- Handles non-DMA flows: CPU → CSR → CE → QSPI FSM → Flash.
- With DMA: Coordinates FIFO transfers via AXI master.
- Supports commands: READ, WRITE, ERASE, READ STATUS.
- Raises cmd_done_set_i on completion.

---

### 3. **QSPI FSM Agent** (`qspi_fsm.v`)
- Generates flash protocol sequences (opcode, address, dummy, data phases).
- Supports SPI/Dual/Quad modes with configurable lanes and dummy cycles.
- Manages CPOL/CPHA, clock divider, and bit shifting.
- Interfaces with FIFO for data streaming.
- Adheres to flash specs (e.g., MX25L6436F timings).

---

### 4. **QSPI IO Agent** (`qspi_io.v`)
- Physical IO shifter for QSPI pins (SCLK, CS#, IO[3:0]).
- Handles single/dual/quad shifting, tri-state control, and HOLD/WP if enabled.
- Synchronizes with QSPI FSM for phase transitions.

---

### 5. **FIFO Agents**
#### **TX FIFO** (`fifo_tx.v`)
- Buffers write data from CPU/DMA to QSPI FSM.
- Signals: fifo_tx_we_o (write enable), tx_level_i, tx_empty_i.

#### **RX FIFO** (`fifo_rx.v`)
- Buffers read data from QSPI FSM to CPU/DMA/XIP.
- Signals: fifo_rx_re_o (read enable), rx_level_i, rx_full_i.
- CDC handling for clock domain differences (AXI 4x faster than QSPI).

---

### 6. **DMA Engine Agent** (`dma_engine.v`)
- AXI4-Lite master for offloading FIFO ↔ memory transfers.
- Configurable burst size, direction (dma_dir_o), address/length.
- Avoids underrun/overrun with FIFO levels.
- Raises dma_done_set_i on completion.

---

### 7. **XIP Engine Agent** (`xip_engine.v`)
- AXI4-Lite slave for memory-mapped flash access (reads; writes optional).
- Translates AXI reads to continuous QSPI fetches.
- Supports dummy cycles, mode bits, and continuous read.
- No CMD_TRIGGER needed; triggered by AXI activity.

---

## Simulation Agent
- Self-checking testbench for:
  - Command Mode: READ/WRITE/ERASE with/without DMA.
  - XIP Mode: Burst reads, waveform verification.
  - FIFO overflows, error injection (timeout, overrun).
  - Flash model integration (qspi_device.v, MX25L6436F.v).
- Automatic pass/fail scoreboard with coverage for lanes, dummies, and modes.

---

## Deliverables
- **RTL:** Verilog `.v` source files (csr.v, qspi_fsm.v, etc.).
- **Testbench:** Self-checking `.v` files with flash model.
- **Docs:** PDF with block diagrams, FSM diagrams, register map, timing (from QSPI_Controller_Design_Summary.pdf).
- **Constraints:** FPGA synthesis constraints.
- **Scripts:** ModelSim/Xcelium simulation scripts.
- **Notes on Project Structure:** `folder rtl/ is old verilog files, the current working folders are src/ for RTL codes and tb/ for testbenches`.

---

## Notes
- All modules are **parameterized** for reuse (e.g., ADDR_WIDTH, FIFO_DEPTH).
- Only synchronous resets are used.
- APB interface is AMBA 3/4 compliant; AXI is AXI4-Lite.
- Modes are exclusive: CMD_TRIGGER for Command, XIP_EN for XIP.

---

## Coding Style Guidelines 

All RTL modules in this project should strictly follow the coding conventions:

1. **Indentation & Formatting**
   - Use consistent indentation.
   - Align parameters, port lists, and signal declarations for readability.

2. **Module Declaration**
   - Parameter list on top, then port list.
   - Parameters use uppercase names with default values.
   - All ports explicitly declared with direction, type, and width.

3. **Naming Conventions**
   - Lowercase with underscores for signals and instances.
   - Uppercase for parameters, constants, and state encodings.
   - `_i` suffix for inputs, `_o` suffix for outputs, `_n` for active-low signals.

4. **Comments**
   - Use block comments for module descriptions.
   - Use inline comments for important lines of code or signal meaning.

5. **Reset Style**
   - Synchronous reset only, active-high by default.
   - All registers initialized in reset block.

6. **Coding Practices**
   - Avoid using `function` and `generate`.
   - Use `localparam` for state machine encodings.
   - Avoid hard-coded values; use parameters where possible.
   - No inferred latches; all sequential logic inside `always @(posedge clk)`.

This ensures all modules (`csr.v`, `qspi_fsm.v`, `dma_engine.v`, `xip_engine.v`, etc.) remain consistent, maintainable, and synthesis-friendly.

---

## Simulation and Linting Flow

For this QSPI Controller IP Core project, both **Verilator** and **Icarus Verilog (iverilog)** are used in a complementary workflow:

1. **Run Verilator lint** early to catch width mismatches, unused signals, and synthesis issues before simulation. (`timescale only needed in testbench)
2. **Use iverilog** to run functional simulations, especially for protocol testbenches (Command/XIP) and FIFO/DMA verification.
3. Maintain a single **file list** (`.f` file or Makefile variable) so both tools compile the exact same RTL set.
4. Optionally run **Verilator simulation** for large-scale or long-run tests where speed is critical, while iverilog is preferred for mixed behavioral testbenches.
5. Always check protocol compliance, flash timing, and FIFO/DMA consistency in both simulators.

