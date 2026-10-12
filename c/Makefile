CC      ?= clang
CFLAGS  ?= -O2 -std=c99 -Wall -Wextra
SRC      = ekf7_math.c ekf7_jacobian.c ekf7_filter.c ekf7_motion.c ekf7_pipeline.c

all: ekf7_test

ekf7_test: test_runner.c $(SRC) ekf7.h
	$(CC) $(CFLAGS) -o $@ test_runner.c $(SRC) -lm

static: $(SRC) ekf7.h          # 嵌入式工程直接取这些文件
	@echo "嵌入到固件: 复制 ekf7.h + $(SRC)"

clean:
	rm -f ekf7_test
