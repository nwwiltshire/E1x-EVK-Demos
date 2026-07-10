# Convenience wrapper so the acceptance command works from the repo root:
#   make sim && make -C tests && ./tests/test_e2e.sh
.PHONY: sim e1 test clean

sim e1:
	$(MAKE) -C firmware $@

test:
	$(MAKE) -C tests
	./tests/test_e2e.sh

clean:
	$(MAKE) -C firmware clean
	$(MAKE) -C tests clean
