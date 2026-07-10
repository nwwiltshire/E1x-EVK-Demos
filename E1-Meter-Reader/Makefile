# Root convenience wrapper.
#
#   make sim    build the Linux-simulator firmware (no hardware, no SDK)
#   make e1     build the E1 hardware images (needs ~/effcc)
#   make test   unit tests + end-to-end tests against the sim binary
#   make clean

.PHONY: sim e1 test clean

sim e1:
	$(MAKE) -C firmware $@

test:
	$(MAKE) -C tests
	./tests/test_e2e.sh

clean:
	$(MAKE) -C firmware clean
	$(MAKE) -C tests clean
