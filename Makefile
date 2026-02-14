.PHONY: all build clean run

all: build

build:
	@mkdir -p build
	@cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
	@cmake --build build -j$$(nproc)

clean:
	@rm -rf build

run: build
	@./build/drone_sim

help:
	@echo "Targets:"
	@echo "  make        - Build project"
	@echo "  make clean  - Remove build directory"
	@echo "  make run    - Build and run"