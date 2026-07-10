# Unit tests: build and run (make -C tests).  The e2e test is separate:
# ./tests/test_e2e.sh
CC     ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror
BUILD  := build
FW     := ../firmware

FW_SRC := $(FW)/detector.c $(FW)/dsp.c $(FW)/fft_tables.c $(FW)/protocol.c
FW_HDR := $(FW)/detector.h $(FW)/dsp.h $(FW)/fft_tables.h $(FW)/protocol.h

.PHONY: all run clean

all: run

$(BUILD)/test_dsp: test_dsp.c $(FW_SRC) $(FW_HDR)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(FW) -o $@ test_dsp.c $(FW_SRC) -lm

run: $(BUILD)/test_dsp
	./$(BUILD)/test_dsp

clean:
	rm -rf $(BUILD)
