.DEFAULT_GOAL 	 := build
PROJECT_NAME     := max17262

.PHONY: clean build

build:
	@cargo build

clean:
	@cargo clean
