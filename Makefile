CXX      ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O2
BUILD    := build

.PHONY: all test demo run-tests clean

all: test demo

$(BUILD):
	mkdir -p $(BUILD)

test: $(BUILD)
	$(CXX) $(CXXFLAGS) tests/test_main.cpp -o $(BUILD)/tests

demo: $(BUILD)
	$(CXX) $(CXXFLAGS) sim/demo.cpp -o $(BUILD)/demo

run-tests: test
	./$(BUILD)/tests

clean:
	rm -rf $(BUILD)
