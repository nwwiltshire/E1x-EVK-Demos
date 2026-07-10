# tests/Makefile — build and run the C unit tests against the actual
# firmware sources (the guard macro in kernels.h lets gcc compile the
# exact fabric kernels).

CC     ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror
BUILD  ?= build

FW := ../firmware
FW_SRC := $(FW)/gauge.c $(FW)/kernels.c $(FW)/ray_tables.c $(FW)/protocol.c

.PHONY: all clean

all: $(BUILD)/test_gauge
	./$(BUILD)/test_gauge

$(BUILD)/test_gauge: test_gauge.c $(FW_SRC)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I$(FW) -o $@ test_gauge.c $(FW_SRC) -lm

clean:
	rm -rf $(BUILD)
