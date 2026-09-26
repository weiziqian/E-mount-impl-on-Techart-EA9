# LM-EA9 app-image rebuild.
#
# Target: SAMD21E17A, 128 KB flash.  A 20 KB bootloader owns 0x0000-0x5000 and
# jumps to 0x5000 without validating anything (EA9.md 11.3), so the app image is
# linked at 0x5000 and flashed there by tools/ea9flash.py.
#
#   make            build build/ea9-rebuild.bin
#   make hosttest   run the host test suite (emount, servo, init)
#   make clean
#
# ONE configuration.  The noshut / nopa23 / nobusoff / pins-early / dry-run
# variants were the boot-regression bisect; it concluded (NOTES.md §52) and they
# are gone.  If another A/B is ever needed, add it as a *temporary* target and
# delete it when it answers -- five overlapping variants is how a build system
# starts handing you the wrong image while reporting success (§21, §43, §51).

TOOLCHAIN ?= $(HOME)/opt/arm-gcc/bin/arm-none-eabi-
HOSTCC    ?= gcc
ASF4      ?= $(HOME)/opt/asf4/samd21

CC      := $(TOOLCHAIN)gcc
OBJCOPY := $(TOOLCHAIN)objcopy
SIZE    := $(TOOLCHAIN)size

PART    := __SAMD21E17A__
OUT     := build
TARGET  := $(OUT)/ea9-rebuild
# Only gen_packets.py reads this -- it is where the 20 stored reply frames were
# lifted from.  src/emount_packets.{c,h} are checked in, so a tree without the
# stock image still builds; it is needed only to regenerate them.
STOCK   := ../fw/EA9-VER-1-8-0.bin

# -DDEBUG matches the stock build, which has ASF4's __FILE__ assert strings in
# .rodata (rebuild/NOTES.md 6).
# -MMD -MP: emit header dependencies.  Without them, editing a .h leaves every
# .o that includes it stale, and the link succeeds -- so the image silently
# mixes objects compiled against different versions of the same constant.  That
# happened: diag.h's DIAG_PAGES went 16 -> 32, diag.o was not rebuilt, and
# diag_page() rejected every page above 15 without a word.  A whole camera run
# was spent on it (NOTES.md §21).
CFLAGS  := -mthumb -mcpu=cortex-m0plus -O1 -DDEBUG -D$(PART) \
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
        src/diag.c src/trail.c \
        src/motor.c src/abs_encoder.c src/servo.c src/focus_map.c \
        src/focus_dist.c \
        src/focus_pred.c src/vdd.c \
        src/main.c

OBJS := $(patsubst src/%.c,$(OUT)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)
HOSTTEST := $(OUT)/hosttest

all: $(TARGET).bin
	@grep -q BOFF $(TARGET).bin \
	  && echo "  verified: full stock shutdown incl. bus off (BOFF marker)" \
	  || { echo "  FAIL: no BOFF marker in the image"; exit 1; }
	@grep -q DRY0 $(TARGET).bin \
	  && echo "  verified: motor drive ENABLED (DRY0 marker)" \
	  || { echo "  FAIL: no DRY0 marker in the image"; exit 1; }
	@echo "  source id: $$(cat src/*.c src/*.h | md5sum | cut -c1-6)"

$(OUT):
	@mkdir -p $(OUT)

$(OUT)/%.o: src/%.c | $(OUT)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# diag.c carries the build stamp that tells the device "this is a new image,
# start the boot tally over".  It has to be rebuilt every time or the stamp
# tracks the last change to diag.c instead of the last change to the image --
# which it did, and two runs' boots were read as one.
.PHONY: FORCE
FORCE:
$(OUT)/diag.o: src/diag.c FORCE | $(OUT)
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

.PHONY: all hosttest clean
