# firmware/Makefile — gcc simulator build + cmake passthrough for the
# E1 hardware build.
#
#   make sim       -> build/firmware_sim   (TCP-loopback "serial link")
#   make sim-echo  -> build/echo_sim
#   make e1        -> bld/fabric/meter_reader.hex etc. (needs ~/effcc)

CC     ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror
BUILD  ?= build

CORE_SRC := main.c gauge.c kernels.c ray_tables.c protocol.c
HDRS     := hal.h protocol.h gauge.h kernels.h ray_tables.h

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
