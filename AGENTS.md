# AGENTS.md

## Overview
This project implements a **parameterizable QSPI Flash Controller IP Core** in Verilog, designed to interface a host processor with external QSPI flash memory devices (e.g., Macronix MX25L6436F). The IP supports two mutually exclusive modes: Command Mode (with optional DMA) for programmable transactions and XIP Mode (Execute-In-Place) for memory-mapped access via an AXI slave interface.

The design is modular, synthesis-ready for FPGA/ASIC, and uses a fully synchronous reset style with an APB CSR slave interface, AXI master for DMA, and AXI slave for XIP.

---

## Architecture & Dataflows

The QSPI Flash Controller consists of interconnected sub-modules: the CSR register bank, Command Engine (CE), XIP Engine, DMA Engine, QSPI FSM, QSPI IO, and RX FIFO buffers, plus external interfaces (APB, AXI, and QSPI flash pins). The data flow varies by operating mode:

- **Command mode (no DMA)**: CSR → CE → QSPI FSM → FIFO RX → CSR  
  CPU-driven PIO transfers. The CPU configures the operation via the APB CSR interface and sets a trigger bit to start the command. The Command Engine (CE) interprets CSR settings and initiates a transaction. Incoming read data from the flash is captured into the FIFO RX and read back by the CPU via the CSR interface.

- **Command mode (with DMA)**: CSR → CE → QSPI FSM → FIFO RX → DMA  
  Hardware-accelerated transfers. The CPU programs the command in CSR and enables DMA for the data phase. The CE triggers the QSPI FSM to execute the command, and the DMA engine transfers data between the FIFO RX and system memory via AXI. For a flash read operation, the FSM fills the RX FIFO with incoming data, and the DMA engine streams this data to a designated memory buffer, offloading the CPU for large transfers.

- **XIP mode**: CSR → XIP → QSPI FSM → FIFO RX → XIP  
  Memory-mapped flash reads. The CPU (or bus master) reads directly from a predefined flash address range via the QSPI controller’s AXI slave interface. The XIP Engine generates the appropriate flash read sequence (using pre-configured settings in the XIP registers) and triggers the QSPI FSM. The FSM performs the read from the flash, and data is captured in the FIFO RX and returned over the AXI bus. From the system’s perspective, the flash is memory-mapped (typically read-only; writes are optional).

**Notes on Data Paths**: In all modes, the QSPI FSM uses the FIFO RX for incoming data from the flash. Only one front-end engine (CE or XIP) is active at a time. The FIFO RX serves as a Clock Domain Crossing (CDC) buffer between the faster system clock domain (e.g., AXI) and the slower QSPI clock domain.

---

## Agent Roles

### 1. **CSR Agent** (`csr.v`)
- Implements AMBA APB v2.0-compliant CSR slave.
- Handles register access for:
  - `CTRL`: enable, mode select (XIP_EN, CMD_TRIGGER (self-clearing), DMA_EN), etc.
  - `STATUS`: busy flags, error codes, FIFO levels.
  - Interrupt handling: `INT_EN`, `INT_STAT` for CMD_DONE, DMA_DONE, ERR, FIFO events.
  - Timing and mode registers: `CLK_DIV`, `CS_CTRL`, `XIP_CFG`, `CMD_CFG`, `DMA_CFG`.
- Ensures synchronous read/write timing with reserved-bit masks and RO validation.

### 2. **Command Engine (CE) Agent** (`cmd_engine.v`)
- Interprets CMD_* registers and triggers QSPI transactions.
- Handles non-DMA flows: CPU → CSR → CE → QSPI FSM → Flash.
- With DMA: Coordinates FIFO transfers via AXI master.
- Supports commands: READ, WRITE, ERASE, READ STATUS.
- Signals completion via `cmd_done_set_i`.

### 3. **QSPI FSM Agent** (`qspi_fsm.v`)
- Generates flash protocol sequences (opcode, address, dummy, data phases).
- Supports SPI/Dual/Quad modes with configurable lanes and dummy cycles.
- Manages CPOL/CPHA, clock divider, and bit shifting.
- Interfaces with FIFO RX for data streaming.
- Adheres to flash specs (e.g., MX25L6436F timings).

### 4. **QSPI IO Agent** (`qspi_io.v`)
- Physical IO shifter for QSPI pins (SCLK, CS#, IO[3:0]).
- Handles single/dual/quad shifting, tri-state control, and HOLD/WP if enabled.
- Synchronizes with QSPI FSM for phase transitions.

### 5. **RX FIFO Agent** (`fifo_rx.v`)
- Buffers read data from QSPI FSM to CPU/DMA/XIP.
- Signals: fifo_rx_re_o (read enable), rx_level_i, rx_full_i.
- Handles CDC for clock domain differences (AXI 4x faster than QSPI).

### 6. **DMA Engine Agent** (`dma_engine.v`)
- AXI4 master for offloading FIFO RX ↔ memory transfers.
- Configurable burst size, direction (dma_dir_o), address/length.
- Prevents FIFO underflow/overflow.
- Raises dma_done_set_i on completion.

### 7. **XIP Engine Agent** (`xip_engine.v`)
- AXI4 slave for memory-mapped flash access (reads; writes optional).
- Translates AXI reads to continuous QSPI fetches.
- Supports dummy cycles, mode bits, and continuous read.
- Operates without explicit command trigger.

---

## Simulation Agent
- Self-checking testbench environment using Icarus Verilog (iverilog) and GTKWave.
- Covers:
  - Command Mode: READ/WRITE/ERASE with/without DMA.
  - XIP Mode: Burst reads, waveform verification.
  - FIFO overflows, error injection (timeout, overrun).
  - Flash model integration (MX25L6436F.v).
- Automatic pass/fail scoreboard with coverage for lanes, dummies, and modes.
- Uses GTKWave for waveform visualization and debugging.

---

## Project Structure
- `src/` → Current RTL implementation.
- `tb/` → Testbenches and flash model integration.
- `rtl/` → Old/deprecated RTL files (for reference only).
- `docs/` → Specifications, diagrams, register maps, timing.
- `reference/` → Supporting reference materials.

---

## Deliverables
- **RTL:** Verilog `.v` source files (csr.v, qspi_fsm.v, etc.).
- **Testbench:** Self-checking `.v` files with flash model for iverilog.
- **Docs:** PDF with block diagrams, FSM diagrams, register map, timing (from QSPI Flash Controller – Technical Design Report.pdf).
- **Constraints:** FPGA synthesis constraints.
- **Scripts:** Icarus Verilog simulation scripts.
- **Notes on Project Structure:** `rtl/` contains old Verilog files; current working folders are `src/` for RTL codes and `tb/` for testbenches.

---

## Notes
- All modules are **parameterized** for reuse (e.g., ADDR_WIDTH, FIFO_DEPTH).
- Only synchronous resets are used.
- APB interface is AMBA APB v2.0 compliant; AXI is AXI4.
- Modes are exclusive: CMD_TRIGGER for Command, XIP_EN for XIP.
- AXI and APB interfaces follow standard AMBA compliance.

---

## Coding Style Guidelines 

All RTL modules in this project should strictly follow these coding conventions:

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
   - Synchronous reset only, active-low by default.
   - All registers initialized in reset block.

6. **Coding Practices**
   - Avoid using `function` and `generate`.
   - Use `localparam` for state machine encodings.
   - Avoid hard-coded values; use parameters where possible.
   - No inferred latches; all sequential logic inside `always @(posedge clk)`.
   - Add timer for timeout case for every testbench

This ensures all modules (`csr.v`, `qspi_fsm.v`, `dma_engine.v`, `xip_engine.v`, etc.) remain consistent, maintainable, and synthesis-friendly.

---

## Simulation and Linting Flow

For this QSPI Controller IP Core project, **Icarus Verilog (iverilog)** and **GTKWave** are used for simulation and waveform analysis:

1. **Run iverilog** for functional simulations, covering protocol testbenches (Command/XIP) and FIFO/DMA verification.
2. Use **GTKWave** to visualize waveforms and debug simulation results.
3. Maintain a single **file list** (`.f` file or Makefile variable) to ensure consistent RTL compilation.
4. Check protocol compliance, flash timing, and FIFO/DMA consistency in simulations.