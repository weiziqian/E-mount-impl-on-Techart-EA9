# LM-EA9 app-image rebuild.
#
# Target: SAMD21E17A, 128 KB flash.  A 20 KB bootloader owns 0x0000-0x5000 and
# jumps to 0x5000 without validating anything (EA9.md 11.3), so the app image is
# linked at 0x5000 and flashed there by tools/ea9flash.py.
#
#   make            DEBUG image   -> build/ea9-rebuild.bin
#   make release    RELEASE image -> build/ea9-rebuild-release.bin
#   make hosttest   run the host test suite (emount, servo, no-body,
#                   init, focus map/distance/prediction)
#   make clean
#
# TWO configurations, and they differ in exactly two things:
#
#   | | optimisation | flash trail |
#   | debug   | -O1 -DDEBUG    | compiled IN  -- diag.c + trail.c    |
#   | release | -O2 -DEA9_NO_LOG -DNDEBUG | compiled OUT -- not built at all |
#
# Why release writes NOTHING to flash: every trail page is an erase/program
# cycle on a part with a finite number of them.  A bench session's worth is
# nothing; a camera used for years is not.  So release does not "disable
# logging at runtime" -- diag.c and trail.c are not in SRCS, the headers
# replace their interface with inline no-ops (see src/diag.h), and the `all`
# rule then CHECKS that no diag_*/trail_* symbol survived.  Nothing else in
# the image writes flash: clock.c and startup_samd21.c touch NVMCTRL only for
# wait states and the MANW errata bit, both reads-modify-writes of CTRLB.
#
# Each configuration has its OWN object directory.  Sharing one is how a
# build system starts handing you the wrong image while reporting success --
# it has happened here (NOTES.md §21, §43, §51).  If you add a third
# configuration, give it its own directory too, or do not add it.
#
# (The noshut / nopa23 / nobusoff / pins-early / dry-run variants were the
# boot-regression bisect; it concluded (NOTES.md §52) and they are gone.)

TOOLCHAIN ?= $(HOME)/opt/arm-gcc/bin/arm-none-eabi-
HOSTCC    ?= gcc
ASF4      ?= $(HOME)/opt/asf4/samd21

CC      := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE    := $(TOOLCHAIN)size

PART    := __SAMD21E17A__
OUT     := build
# Only gen_packets.py reads this -- it is where the 20 stored reply frames were
# lifted from.  src/emount_packets.{c,h} are checked in, so a tree without the
# stock image still builds; it is needed only to regenerate them.
STOCK   := ../fw/EA9-VER-1-8-0.bin

# The configuration.  `make release` re-enters with BUILD=release; everything
# below is written once and parameterised, so the two images cannot drift.
BUILD ?= debug

ifeq ($(BUILD),debug)
OBJDIR  := $(OUT)/obj-debug
TARGET  := $(OUT)/ea9-rebuild
# -O1 is what every hardware result in NOTES.md was obtained at.
OPT     := -O1
# -DDEBUG matches the stock build, which has ASF4's __FILE__ assert strings in
# .rodata (rebuild/NOTES.md 6).
CONFIG  := -DDEBUG
LOGSRCS := src/diag.c src/trail.c
else ifeq ($(BUILD),release)
OBJDIR  := $(OUT)/obj-release
TARGET  := $(OUT)/ea9-rebuild-release
OPT     := -O2
# EA9_NO_LOG turns the whole diag/trail interface into inline no-ops, so the
# callers' page-building code is dead and goes with it.
CONFIG  := -DNDEBUG -DEA9_NO_LOG
LOGSRCS :=
else
$(error BUILD must be debug or release, not "$(BUILD)")
endif

# -MMD -MP: emit header dependencies.  Without them, editing a .h leaves every
# .o that includes it stale, and the link succeeds -- so the image silently
# mixes objects compiled against different versions of the same constant.  That
# happened: diag.h's DIAG_PAGES went 16 -> 32, diag.o was not rebuilt, and
# diag_page() rejected every page above 15 without a word.  A whole camera run
# was spent on it (NOTES.md §21).
CFLAGS  := -mthumb -mcpu=cortex-m0plus $(OPT) $(CONFIG) -D$(PART) \
           -ffunction-sections -fdata-sections -fno-common \
           -Wall -Wextra -std=gnu99 -g -MMD -MP

INCLUDES := -Isrc \
            -I$(ASF4) -I$(ASF4)/config -I$(ASF4)/include \
            -I$(ASF4)/CMSIS/Include \
            -I$(ASF4)/hal/include -I$(ASF4)/hal/utils/include \
            -I$(ASF4)/hpl/core -I$(ASF4)/hpl/pm -I$(ASF4)/hpl/port \
            -I$(ASF4)/hri

LDFLAGS  = -mthumb -mcpu=cortex-m0plus -Tsrc/ea9.ld -Wl,--gc-sections \
           -Wl,-Map=$(TARGET).map --specs=nano.specs --specs=nosys.specs

SRCS := src/startup_samd21.c src/system_samd21.c src/board.c \
        src/clock.c src/delay.c \
        src/emount.c src/emount_packets.c src/em_uart.c src/em_eic.c \
        $(LOGSRCS) \
        src/motor.c src/abs_encoder.c src/servo.c src/focus_map.c \
        src/focus_dist.c \
        src/focus_pred.c src/vdd.c \
        src/main.c

OBJS := $(patsubst src/%.c,$(OBJDIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)
HOSTTEST := $(OUT)/hosttest

all: $(TARGET).bin
ifeq ($(BUILD),debug)
	@grep -q BOFF $(TARGET).bin \
	  && echo "  verified: full stock shutdown incl. bus off (BOFF marker)" \
	  || { echo "  FAIL: no BOFF marker in the image"; exit 1; }
	@grep -q DRY0 $(TARGET).bin \
	  && echo "  verified: motor drive ENABLED (DRY0 marker)" \
	  || { echo "  FAIL: no DRY0 marker in the image"; exit 1; }
else
	@# The release image is checked for the OPPOSITE of the debug image: the
	@# marker page is written by the trail, so its absence says the trail
	@# really is gone from the binary and not merely quiet at runtime.
	@if grep -q BOFF $(TARGET).bin || grep -q DRY0 $(TARGET).bin; then \
	    echo "  FAIL: build-marker strings still in the image -- the trail is compiled in"; \
	    exit 1; \
	 fi
	@echo "  verified: no build-marker page (the trail is compiled out)"
	@# The load-bearing check.  Every flash erase and program in this
	@# firmware goes through diag.c; if no diag_* or trail_* symbol was
	@# linked, the image has no instruction that can write flash.
	@if $(TOOLCHAIN)nm $(TARGET).elf | grep -qE ' (diag_|trail_)'; then \
	    echo "  FAIL: diag/trail symbols linked into a release image:"; \
	    $(TOOLCHAIN)nm $(TARGET).elf | grep -E ' (diag_|trail_)'; \
	    exit 1; \
	 fi
	@echo "  verified: no diag_*/trail_* symbol -- the image cannot write flash"
endif
	@echo "  source id: $$(cat src/*.c src/*.h | md5sum | cut -c1-6)"

# `make release` is the same build with BUILD=release; `make debug` is a name
# for the default so both configurations can be asked for the same way.
release:
	@$(MAKE) --no-print-directory BUILD=release all

debug:
	@$(MAKE) --no-print-directory BUILD=debug all

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# diag.c carries the build stamp that tells the device "this is a new image,
# start the boot tally over".  It has to be rebuilt every time or the stamp
# tracks the last change to diag.c instead of the last change to the image --
# which it did, and two runs' boots were read as one.
.PHONY: FORCE
FORCE:
$(OBJDIR)/diag.o: src/diag.c FORCE | $(OBJDIR)
	$(CC) $(CFLAGS) -DDIAG_BUILD_STAMP=$$(date +%s) \
	  -DDIAG_SRC_ID=0x$$(cat src/*.c src/*.h | md5sum | cut -c1-6) \
	  $(INCLUDES) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	@$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
	@echo "built $@ ($$(stat -c%s $@) bytes)"

# The packet table is lifted from the stock image, so regenerate it whenever
# that image or the generator changes rather than checking in a stale copy.
#
# Guarded on the stock image existing.  src/emount_packets.{c,h} are checked
# in, so a tree without a stock image still builds -- the rule simply does not
# apply there.  Without the guard make refuses to do anything at all: "No rule
# to make target '../fw/EA9-VER-1-8-0.bin'".
ifneq ($(wildcard $(STOCK)),)
src/emount_packets.c src/emount_packets.h: tools/gen_packets.py $(STOCK)
	python3 tools/gen_packets.py $(STOCK) \
	        src/emount_packets.c src/emount_packets.h
endif

$(OBJS): | src/emount_packets.c

# The responder, compiled for the host and driven through scripted body
# traffic.  It compiles src/emount.c unmodified, so what passes here is the
# code that ships -- see tools/hosttest/.
hosttest: src/emount_packets.c
	@mkdir -p $(HOSTTEST)
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_emount.c tools/hosttest/shim.c \
	  src/emount.c src/emount_packets.c -o $(HOSTTEST)/test_emount
	@$(HOSTTEST)/test_emount
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_servo.c tools/hosttest/sim_motor.c \
	  src/servo.c -o $(HOSTTEST)/test_servo
	@$(HOSTTEST)/test_servo
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_nobody.c tools/hosttest/shim.c \
	  src/emount.c src/emount_packets.c -o $(HOSTTEST)/test_nobody
	@$(HOSTTEST)/test_nobody nobody
	@$(HOSTTEST)/test_nobody slowbody
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_init.c tools/hosttest/shim.c \
	  src/emount.c src/emount_packets.c -o $(HOSTTEST)/test_init
	@$(HOSTTEST)/test_init
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_focus.c src/focus_map.c -o $(HOSTTEST)/test_focus
	@$(HOSTTEST)/test_focus
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_focus_dist.c src/focus_dist.c -lm \
	  -o $(HOSTTEST)/test_focus_dist
	@$(HOSTTEST)/test_focus_dist
	$(HOSTCC) -std=gnu99 -Wall -Wextra -O1 -Itools/hosttest -Isrc \
	  tools/hosttest/test_focus_pred.c src/focus_pred.c \
	  -o $(HOSTTEST)/test_focus_pred
	@$(HOSTTEST)/test_focus_pred

clean:
	rm -rf $(OUT)

-include $(DEPS)

.PHONY: all debug release hosttest clean
