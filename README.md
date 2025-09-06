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
Install [Icarus Verilog](http://iverilog.icarus.com/) and [GTKWave](https://gtkwave.sourceforge.net/). Below are example commands to build and run sample testbenches:

```bash
# QSPI FSM testbench
iverilog -g2012 -s qspi_fsm_tb -o .sim/qspi_fsm_tb.vvp src/*.v tb/qspi_fsm_tb.v
vvp .sim/qspi_fsm_tb.vvp

# CSR testbench
iverilog -g2012 -s csr_tb -o .sim/csr_tb.vvp src/*.v tb/csr_tb.v
vvp .sim/csr_tb.vvp

# FIFO unit tests (TX and RX)
iverilog -g2012 -s fifo_tx_tb -o .sim/fifo_tx_tb.vvp src/*.v tb/fifo_tx_tb.v
vvp .sim/fifo_tx_tb.vvp

iverilog -g2012 -s fifo_rx_tb -o .sim/fifo_rx_tb.vvp src/*.v tb/fifo_rx_tb.v
vvp .sim/fifo_rx_tb.vvp
```

To view waveforms with GTKWave, ensure your testbench emits a VCD and then:

```bash
gtkwave dump.vcd
```

These tests exercise the key blocks (FSM, CSR, and FIFOs) using Icarus Verilog.

## License
MIT
