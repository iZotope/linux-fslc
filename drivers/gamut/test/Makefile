TEST_SRC := test-gamut-ui.c
RUNNER_SRC := $(basename $(TEST_SRC))_Runner.c

SRCS := \
	$(TEST_SRC) \
	$(RUNNER_SRC) \
	unity/unity.c \
	crc16.c \
	../izspi.c \
	../izcbuf.c \

OBJS :=  $(SRCS:.c=.o)

DEPFLAGS = -MT $@ -MMD -MP

BLD_DIR := obj/test
OBJS := $(addprefix $(BLD_DIR)/,$(OBJS))
$(warning $(OBJS))
CFLAGS += $(DEPFLAGS) -g -I. -Iunity -std=gnu99 -Werror=implicit-function-declaration

EXEC := test-gamut-ui

all: $(EXEC)
	@echo
	@echo
	@echo
	./$(EXEC)

$(BLD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	gcc $(CFLAGS) -c $< -o $@

$(RUNNER_SRC): $(TEST_SRC)
	ruby unity/generate_test_runner.rb $(TEST_SRC)

$(EXEC): $(OBJS)
	gcc $(OBJS) -o $@

clean:
	-rm -f gamut-ui-test
	-rm -rf obj

-include $(OBJS:.o=.d)
