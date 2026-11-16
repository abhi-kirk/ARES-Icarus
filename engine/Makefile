BUILD_DIR := build
CMAKE := cmake

.PHONY: all configure build clean rebuild run regress

all: build

configure:
	$(CMAKE) -S . -B $(BUILD_DIR) -G "Unix Makefiles" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -Wno-dev

build: configure
	$(CMAKE) --build $(BUILD_DIR) --parallel

clean:
	$(CMAKE) --build $(BUILD_DIR) --target clean

rebuild: clean build

run: build
	./$(BUILD_DIR)/Icarus

regress: build
	node scripts/mpc_regress.mjs
