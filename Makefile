MAKEFLAGS += -j$(shell nproc)

# 编译器设置
CC := gcc
CXX := g++

# 默认构建模式（可被覆盖）
BUILD_MODE ?= release
ifeq ($(BUILD_MODE), debug)
	CFLAGS := -Wall -g -Iinclude -Iopencv -Iyolov5 -MMD -MP
	CXXFLAGS := -Wall -g -Iinclude -Iopencv -Iyolov5 -MMD -MP `pkg-config --cflags opencv4`
else
	CFLAGS := -Wall -O2 -Iinclude -Iopencv -Iyolov5 -MMD -MP
	CXXFLAGS := -Wall -O2 -Iinclude -Iopencv -Iyolov5 -MMD -MP `pkg-config --cflags opencv4`
endif

LDFLAGS :=
LDLIBS := -lm -lpthread -lreadline -lhistory `pkg-config --libs opencv4` -lrknnrt \
          `pkg-config --libs libwebsockets openssl`

# C 源文件与目标文件
C_SRC := $(wildcard src/*.c drivers/*.c)
C_OBJ := $(patsubst %.c, build/%.o, $(C_SRC))

# C++ 源文件与目标文件
CPP_SRC := $(wildcard opencv/*.cpp yolov5/*.cpp drivers/*.cpp)
CPP_OBJ := $(patsubst %.cpp, build/%.o, $(CPP_SRC))

# 所有对象文件
OBJ := $(C_OBJ) $(CPP_OBJ)
DEP := $(OBJ:.o=.d)

# 目标名
TARGET := app

# 默认目标
all: $(TARGET)

# C 编译规则
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# C++ 编译规则
build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 链接目标
$(TARGET): $(OBJ)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

# 自动依赖
-include $(DEP)

# 快捷目标 gdb，用于调试版本编译
gdb:
	$(MAKE) BUILD_MODE=debug
# sudo gdbserver :1234 ./app /dev/i2c-5 /dev/i2c-5 /dev/video11
run:
	$(MAKE)
	sudo ./app /dev/i2c-5 /dev/i2c-5 /dev/video11	
	
# 清理
clean:
	@echo "[CLEAN]"
	rm -rf build app

.PHONY: all clean
