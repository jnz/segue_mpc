# Run 'make'

# Call with 'make Q=' to enable verbose output
Q ?= @

# Custom config (optional)
-include config.mk

#We try to detect the OS we are running on, and adjust commands as needed
ifeq ($(OS),Windows_NT)
	CLEANUP = rm -f
	MKDIR = mkdir
	TARGET_EXTENSION=.dll
else
	CLEANUP = rm -f
	MKDIR = mkdir -p
	TARGET_EXTENSION=.so
endif
# Target executable configuration
TARGET_BASE=libctrl
TARGET = $(TARGET_BASE)$(TARGET_EXTENSION)

# Compiler settings
CC=$(TOOLCHAIN)g++
# -nostdinc Disable standard #include directories
# -fno-exceptions Disable support for exception handling
# -fvisibility=<arg> Set the default symbol visibility for all global declarations
# -Wpedantic Enable tons of warnings
# -ffreestanding Assert that the compilation takes place in a freestanding environment
# -fno-builtin Disable implicit builtin knowledge of a specific function
CFLAGS += -Wno-deprecated-declarations -Wno-ignored-attributes
CFLAGS += -O2
# -fno-common: This has the effect that if the same variable is declared (without extern) in two different compilations, you get a multiple-definition error when you link them
# -MMD: to autogenerate dependencies for make
# -MP: These dummy rules work around errors make gives if you remove header files without updating the Makefile to match.
# -MF: When used with the driver options -MD or -MMD, -MF overrides the default dependency output file.
CFLAGS += -fno-common -MMD -MP -MF"$(@:%.o=%.d)"
CFLAGS += -flto -fPIC
# Linker settings
LINK:=$(CC)
LINK_FLAGS += -shared

# Header files
INCLUDE_PATH := -I"src/"

# Source folders
SRC_SUBDIRS := ./src
# SRC_SUBDIRS += ./src

# Add all files from the folders in SRC_SUBDIRS to the build
OBJDIR           := build
SOURCES          = $(foreach dir, $(SRC_SUBDIRS), $(wildcard $(dir)/*.cpp))
C_SRCS           = $(SOURCES)
VPATH            = $(SRC_SUBDIRS)
OBJ_NAMES        = $(notdir $(C_SRCS))
OBJS             = $(addprefix $(OBJDIR)/,$(OBJ_NAMES:%.cpp=%.o))
C_DEPS           = $(OBJS:%.o=%.d)
C_INCLUDES       = $(INCLUDE_PATH)
# LOCAL_LIBRARIES = -lm

# flag -c: Compile without linking
$(OBJDIR)/%.o: %.cpp
	@echo 'CC: $<'
	$(Q)$(CC) $(CFLAGS) $(C_INCLUDES) -c -o "$@" "$<"

default: $(TARGET)

all: $(TARGET)

$(TARGET): $(OBJDIR) $(OBJS)
	@echo 'Link target: '$@
	$(Q)$(LINK) $(LINK_FLAGS) -o "$@" $(OBJS) $(LOCAL_LIBRARIES)

clean:
	@echo 'Cleanup...'
	$(CLEANUP) $(OBJDIR)/*.d
	$(CLEANUP) $(OBJDIR)/*.o
	$(CLEANUP) $(TARGET)
	$(CLEANUP) $(TARGET_BASE).dmp

# Make sure that we recompile if a header file was changed
-include $(C_DEPS)

post-build:

.FORCE: dump

.PHONY: all .FORCE

.SECONDARY: post-build

