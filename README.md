# QSPI Flash Device Controller

This project provides Verilog building blocks for a parameterizable Quad-SPI (QSPI) flash controller. The IP core bridges a host processor to external QSPI flash memories and is intended for FPGA or ASIC integration.

## RTL Modules
- **csr.v** – APB-accessible control/status register block.
- **qspi_fsm.v** – Finite state machine that issues flash command sequences.
- **fifo.v** – Simple 32-bit wide FIFO used for buffering data.
- **axi_read_block.v** – AXI4-Lite helper to read a block of memory into the FIFO.
- **axi_write_block.v** – AXI4-Lite helper to write FIFO contents to memory.
- **qspi_device.v** – Behavioural flash memory model for simulation.

## Development Notes
The repository follows a synchronous-reset coding style. To run basic lint and compilation checks:

```bash
verilator --lint-only -Wno-fatal rtl/axi_read_block.v rtl/axi_write_block.v rtl/csr.v rtl/fifo.v rtl/qspi_fsm.v
iverilog -g2012 -s qspi_fsm -o /tmp/qspi_fsm.vvp rtl/axi_read_block.v rtl/axi_write_block.v rtl/csr.v rtl/fifo.v rtl/qspi_fsm.v
```

These commands perform Verilator linting and compile the FSM with Icarus Verilog.
