# Host-side unit tests for the platform-independent firmware modules.
#
#   make -C tests          build and run
#   make -C tests clean
#
# Only the CAN codecs and the vehicle controller are compiled here; the
# HAL is replaced by stubs/stm32f4xx_hal.h and a fake HAL_GetTick().

CC      ?= cc
CFLAGS  ?= -std=c11 -O1 -g -Wall -Wextra -Werror
SAN     ?= -fsanitize=address,undefined -fno-omit-frame-pointer

FW_SRC  := ../Core/Src
INCLUDE := -Istubs -I. -I../Core/Inc

SOURCES := test_main.c test_can_codecs.c test_vehicle_control.c fake_hal.c \
           $(FW_SRC)/bms_can.c $(FW_SRC)/motor_can.c $(FW_SRC)/charger_can.c \
           $(FW_SRC)/vehicle_control.c

BIN     := build/run_tests

.PHONY: all test clean

all: test

test: $(BIN)
	./$(BIN)

$(BIN): $(SOURCES) $(wildcard *.h stubs/*.h ../Core/Inc/*.h)
	@mkdir -p build
	$(CC) $(CFLAGS) $(SAN) $(INCLUDE) $(SOURCES) -o $@

clean:
	rm -rf build
