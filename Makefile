# Project Name
TARGET = drifter

# Sources — each DSP task appends its .cpp here.
CPP_SOURCES = src/main.cpp src/clock.cpp src/bezier_random.cpp

# Pull in the float-printf code from full newlib so PrintLine("%f") actually
# emits the float (newlib-nano strips this out by default to save ~10KB).
LDFLAGS += -u _printf_float

# Library Locations (no DaisySP: this app is pure float math + libDaisy HAL)
LIBDAISY_DIR = lib/libDaisy

# Project source includes
C_INCLUDES += -Isrc

# Use Daisy's stock build system
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
