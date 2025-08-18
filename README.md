# QSPI Flash Device Controller

The QSPI Flash Device Controller is a parameterizable Verilog IP core that bridges a host processor to off-chip QSPI flash memories. It supports command mode transactions with optional DMA offload and execute-in-place (XIP) reads through an AXI slave interface. The design is fully synchronous and suitable for FPGA or ASIC integration.

## Features
- Command mode with CPU-driven or DMA-assisted transfers
- XIP mode for memory-mapped flash reads
- AXI4 master for DMA and AXI4 slave for XIP
- QSPI FSM supporting single, dual and quad lanes
- Separate TX/RX FIFOs for clock-domain crossing
- Parameterizable address widths and FIFO depths

## Repository Structure
- `src/` – current RTL modules (`csr.v`, `cmd_engine.v`, `dma_engine.v`, `qspi_fsm.v`, `qspi_io.v`, `xip_engine.v`, `fifo_tx.v`, `fifo_rx.v`, `qspi_controller.v`)
- `tb/` – self-checking testbenches and flash model
- `docs/` – specifications and design notes
- `reference/` – supporting reference material
- `rtl/` – legacy RTL kept for comparison

## Getting Started
Install [Verilator](https://www.veripool.org/verilator/) and [Icarus Verilog](http://iverilog.icarus.com/). Run lint and a sample simulation:

```bash
# Lint all RTL
verilator --lint-only src/*.v

# Compile and run the QSPI FSM testbench
iverilog -g2012 -s qspi_fsm_tb -o /tmp/qspi_fsm_tb.vvp src/*.v tb/qspi_fsm_tb.v
vvp /tmp/qspi_fsm_tb.vvp
```

These commands verify that the RTL compiles cleanly and exercise the QSPI FSM.

## License
MIT
