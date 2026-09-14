# test/Makefile — host-side unit tests for the DSP modules.
# Build & run with:   make -C test
CXX      ?= g++
CXXFLAGS  = -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -I../src -I.

OUT       = ../build_host

# Default test list — extended as new modules are added.
TESTS = test_smoke

all: $(addprefix $(OUT)/, $(TESTS))
	@for t in $(TESTS); do \
	  echo "===== $$t ====="; \
	  $(OUT)/$$t || exit 1; \
	done

$(OUT):
	mkdir -p $(OUT)

$(OUT)/test_smoke: test_smoke.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -rf $(OUT)

.PHONY: all clean
