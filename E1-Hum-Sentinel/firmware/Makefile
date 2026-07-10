# E1 acoustic-sentinel firmware
#
#   make sim       — Linux simulator build (gcc): firmware runs as a
#                    process, serial link over a loopback TCP socket.
#   make sim-echo  — simulator build of the link-echo bring-up app.
#   make e1        — hardware build via the Efficient SDK CMake flow
#                    (needs the SDK at ~/effcc; see CMakeLists.txt).
#                    Flash the results per the SDK quickstart.

CC     ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror
BUILD  ?= build

CORE_SRC := main.c detector.c dsp.c fft_tables.c protocol.c
HDRS     := hal.h detector.h dsp.h fft_tables.h protocol.h

.PHONY: sim sim-echo e1 clean

sim: $(BUILD)/firmware_sim

$(BUILD)/firmware_sim: $(CORE_SRC) hal_posix.c $(HDRS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ $(CORE_SRC) hal_posix.c

sim-echo: $(BUILD)/echo_sim

$(BUILD)/echo_sim: echo_main.c hal_posix.c hal.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -o $@ echo_main.c hal_posix.c

e1:
	cmake -B bld
	cmake --build bld

clean:
	rm -rf $(BUILD) bld
