SHELL := /bin/bash

# Tools
IVERILOG ?= iverilog
VVP      ?= vvp

# Sources
SRCF := filelist.f
SRC  := $(shell tr '\n' ' ' < $(SRCF))
FLASH_MODEL := docs/MX25L6436F.txt

# Testbenches
UNIT_TESTS := \
  csr_tb \
  qspi_fsm_tb \
  qspi_fsm_quad_eb_tb \
  fifo_tx_tb \
  fifo_rx_tb \
  qspi_device_tb \
  tb_axi4_ram_slave

INTEG_TESTS := \
  dma_engine_tb \
  int_csr_ce_tb \
  int_csr_ce_fsm_tb \
  int_csr_ce_fsm_dma_tb

TOP_TESTS := \
  top_tb \
  top_cmd_tb

FLASH_TESTS := $(TOP_TESTS)

ALL_TESTS := $(UNIT_TESTS) $(INTEG_TESTS) $(TOP_TESTS)

SIMDIR := .sim

.PHONY: test test-fast test-all clean

# Default 'test' runs the fast suite; use 'test-all' for full suite
test: test-fast

test-fast:
	@$(MAKE) --no-print-directory _run_suite TESTS="$(UNIT_TESTS) $(INTEG_TESTS)"

test-all:
	@$(MAKE) --no-print-directory _run_suite TESTS="$(ALL_TESTS)"

_run_suite:
	@mkdir -p $(SIMDIR)
	@: > $(SIMDIR)/summary.txt
	@for t in $(TESTS); do \
	  echo "[BUILD] $$t"; \
	  extra=""; \
	  case " $(FLASH_TESTS) " in *" $$t "*) extra="$(FLASH_MODEL)";; esac; \
	  $(IVERILOG) -g2012 -s $$t -o $(SIMDIR)/$$t.vvp $(SRC) tb/$$t.v $$extra \
	    > $(SIMDIR)/$$t.build.log 2>&1; \
	  if [ $$? -ne 0 ]; then \
	    echo "$$t: BUILD_FAIL" | tee -a $(SIMDIR)/summary.txt; \
	    echo "  see $(SIMDIR)/$$t.build.log"; \
	    continue; \
	  fi; \
	  echo "[RUN] $$t"; \
	  set +e; $(VVP) $(SIMDIR)/$$t.vvp > $(SIMDIR)/$$t.run.log 2>&1; st=$$?; set -e; \
	  if grep -q "Global timeout reached" $(SIMDIR)/$$t.run.log; then \
	    status=TIMEOUT; \
	  elif [ $$st -ne 0 ]; then \
	    status=FAIL; \
	  elif grep -i -q "passed" $(SIMDIR)/$$t.run.log; then \
	    status=PASS; \
	  else \
	    status=OK; \
	  fi; \
	  echo "$$t: $$status" | tee -a $(SIMDIR)/summary.txt; \
	done; \
	echo "----------------"; \
	cat $(SIMDIR)/summary.txt; \
	grep -q "FAIL" $(SIMDIR)/summary.txt && exit 1 || true

clean:
	rm -rf $(SIMDIR) *.vcd tb/*.vcd
