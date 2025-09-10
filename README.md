# QSPI Flash Device Controller

The QSPI Flash Device Controller is a parameterizable Verilog IP core that bridges a host processor to off-chip QSPI flash memories. It supports command mode transactions with optional DMA offload and execute-in-place (XIP) reads through an AXI4‑Lite slave interface. The design is fully synchronous and suitable for FPGA or ASIC integration.

## Current Status
- Unit + integration tests: PASS (`make test-fast`)
- XIP extended tests (quad IO, continuous read, 4‑byte addr): PASS (`make test-extended`)
- Top/system tests (including flash model): PASS (`make test-all`)
- New scenarios: command‑mode multiword DMA burst (read/program/erase), quad‑output XIP, 4B+QIO, mode‑bits variations, and invalid opcode handling.
- VCD and logs are under `.sim/`; recent VCDs are also kept at repo root for convenience.

## Features
- Command mode with CPU-driven or DMA-assisted transfers
- XIP mode for memory-mapped flash reads
- AXI4‑Lite master for DMA and AXI4‑Lite slave for XIP
- QSPI FSM supporting single, dual and quad lanes with dummy cycles and mode bits
- Separate TX/RX FIFOs for clock-domain crossing
- Parameterizable address widths and FIFO depths

## What’s Recently Fixed/Improved
- Command mode stability and coverage: “Passed all for CMD mode”
- XIP path brought up and verified: “XIP working”
- WEL/WIP handling aligned to flash behavior: “Verify WEL bit and wait for WIP to clear”
- DMA robustness: “Fix DMA engine disable handling and FIFO TX read”

See `git log` for details and additional incremental fixes.

## Repository Structure
- `src/` – current RTL modules:
  - `csr.v`, `cmd_engine.v`, `dma_engine.v`, `qspi_fsm.v`, `xip_engine.v`,
    `fifo_tx.v`, `fifo_rx.v`, `qspi_device.v`, `axi4_ram_slave.v`, `qspi_controller.v`, `apb_master.v`
- `tb/` – self-checking testbenches and integration benches
- `docs/` – specifications/design notes and the Macronix MX25L6436F behavioral model
- `reference/` – supporting reference material
- `rtl/` – legacy RTL kept for comparison (not used)

Note: there is no separate `qspi_io.v` in the current implementation; IO handling is integrated into `qspi_fsm.v`.

## How The Design Works
- Command mode (no DMA): APB CSR programs op → `cmd_engine` asserts start → `qspi_fsm` performs opcode/address/dummy/data → data captured into `fifo_rx` → CPU reads via CSR.
- Command mode (with DMA): Same front‑end as above, but `dma_engine` moves data between `fifo_rx`/`fifo_tx` and memory over AXI4‑Lite, preventing under/over‑run via FIFO level checks.
- XIP mode: AXI reads on the slave port go to `xip_engine`, which latches XIP config from CSR, triggers `qspi_fsm` reads, pops `fifo_rx`, and returns 32‑bit words on AXI. Optional single‑word writes are supported when enabled.

### Module Roles
- `csr.v`: APB3/4 CSR bank. Controls enable/mode, clock/CS settings, command and XIP configuration, DMA settings, FIFO windows, and interrupts. Enforces mutual exclusion of CMD and XIP.
- `cmd_engine.v`: Latches CMD_* fields and sequences a single transaction. Generates `start`/`done` for the FSM and raises CMD_DONE.
- `qspi_fsm.v`: Protocol engine for SPI/Dual/Quad. Handles CPOL/CPHA, clock divider, opcode/address/mode/dummy/data phases, and IO tri‑state. Adds CS# setup/hold extension and read/write warm‑ups to satisfy device timing.
- `fifo_tx.v` / `fifo_rx.v`: 32‑bit FIFOs for write/read paths and CDC buffering.
- `dma_engine.v`: AXI4‑Lite master to move data between memory and FIFOs. Splits read/write into dedicated blocks, respects FIFO levels, and asserts DMA_DONE.
- `xip_engine.v`: AXI4‑Lite slave translating memory reads (and optional writes) into QSPI transactions with fixed 4‑byte beats.
- `qspi_controller.v`: Top‑level that wires CSR, engines, FIFOs, and FSM to APB/AXI and pads, arbitrating between CMD and XIP paths.
- `qspi_device.v`: Lightweight behavioral QSPI flash for unit/integration tests. Full Macronix model is also available under `docs/` for top/system tests.

## Running Tests
Prerequisites: Icarus Verilog (iverilog) and GTKWave.

Common targets:
```bash
# Fast unit + integration suite
make test-fast

# Full suite including top/system tests
make test-all

# Extended XIP scenarios (quad IO, continuous read, 4B addressing)
make test-extended
```
Artifacts:
- Build logs: `.sim/<tb>.build.log`
- Run logs: `.sim/<tb>.run.log`
- Waveforms: `.sim/*.vcd` (some tests also emit VCDs in repo root)

### New Test Scenarios
- `tb/cmd_dma_burst_tb.v`: Command‑mode multiword DMA read (0x0B fast‑read, 64B) with `burst_size=8`; verifies AXI RAM receives sixteen 0xFFFF_FFFF words. Exercises CSR→CE→FSM→FIFO RX→DMA path and FIFO‑level gating.
- `tb/xip_engine_quad_output_tb.v`: XIP Quad‑Output fast read (1‑1‑4, 0x6B) with dummy cycles; validates IO lane switching per MX25L6436F spec.
- `tb/xip_engine_4b_quad_io_tb.v`: XIP 4‑byte addressing + Quad I/O (0xEB + 0xA0 mode bits); checks 1‑4‑4 path with extended address.
- `tb/xip_engine_multiword_burst_tb.v`: XIP continuous read with CS hold (cs_auto=0); issues 8 sequential AXI reads to confirm multiword fetch behavior.
- `tb/xip_engine_quad_io_modebits_tb.v`: Quad I/O with non‑A0 mode bits (variation); ensures robust reads despite mode bits differences.
- `tb/xip_engine_invalid_opcode_tb.v`: Negative test using unsupported opcode (0x00); ensures no hang and AXI read completes.
- `tb/cmd_dma_program_burst_tb.v`: Command‑mode multiword DMA program (0x02, 64B) from AXI RAM into flash with robust readback checks (0x03/0x0B) and WEL verification.
- `tb/cmd_dma_erase_burst_tb.v`: Command‑mode sector erase (0x20) after a DMA program, then DMA read back (64B) to verify all 0xFFFF_FFFF.

### Quick Runs (DMA Burst Tests)
- Read burst: `make -s _run_suite TESTS="cmd_dma_burst_tb"`
- Program burst: `make -s _run_suite TESTS="cmd_dma_program_burst_tb"`
- Erase + readback burst: `make -s _run_suite TESTS="cmd_dma_erase_burst_tb"`

## Latest Test Summary

Totals
- Test-All: 20 tests, 20 PASS, 0 FAIL
- Test-Extended: 9 tests, 9 PASS, 0 FAIL

Test-All
- csr_tb: PASS
- qspi_fsm_tb: PASS
- qspi_fsm_quad_eb_tb: PASS
- fifo_tx_tb: PASS
- fifo_rx_tb: PASS
- qspi_device_tb: PASS
- tb_axi4_ram_slave: PASS
- error_csr_tb: PASS
- dma_engine_tb: PASS
- int_csr_ce_tb: PASS
- int_csr_ce_fsm_tb: PASS
- int_csr_ce_fsm_dma_tb: PASS
- irq_dma_tb: PASS
- clk_div_tb: PASS
- top_tb: PASS
- top_cmd_tb: PASS
- apb_master_tb: PASS
- cmd_dma_burst_tb: PASS
- cmd_dma_program_burst_tb: PASS
- cmd_dma_erase_burst_tb: PASS

Test-Extended
- xip_engine_tb: PASS
- xip_engine_quad_io_tb: PASS
- xip_engine_cont_read_tb: PASS
- xip_engine_4b_tb: PASS
- xip_engine_quad_output_tb: PASS
- xip_engine_4b_quad_io_tb: PASS
- xip_engine_multiword_burst_tb: PASS
- xip_engine_invalid_opcode_tb: PASS
- xip_engine_quad_io_modebits_tb: PASS

These benches follow the project’s QSPI Controller Specification and basic Macronix MX25L6436F command behavior as modeled in `src/qspi_device.v`.

## Known Gaps / Next Work
- AXI is AXI4‑Lite style (single‑beat). If multi‑beat bursts are required, extend DMA and XIP to full AXI4 with burst parameters and alignment handling.
- Broaden protocol coverage for additional vendor‑specific commands (deep‑power‑down, SFDP reads, protection registers) as needed by the target flash.
- Hardware bring‑up: add FPGA/ASIC timing constraints and CDC sign‑off; verify IO timing against selected device and clocking.
- Performance tuning: increase FIFO depths and prefetching as needed for higher throughput, and add optional interrupt/coalescing policies.

## License
MIT
