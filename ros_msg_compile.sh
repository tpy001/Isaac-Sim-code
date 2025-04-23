#!/bin/bash

cd ./catkin_ws
# 获取 Python 路径参数
PYTHON_PATH="$1"
CLEAN_OPTION="$2"

# 如果没有传入路径，提示用法
if [ -z "$PYTHON_PATH" ]; then
    echo "❗ Usage: ./ros_msg_compile.sh <path_to_isaac_sim_python>"
    echo "Example: ./ros_msg_compile.sh ~/software/isaac_sim/kit/python/bin/python3"
    exit 1
fi

# 获取当前目录（catkin 工作空间）
WS_DIR=$(pwd)

# 如果传入了 --clean 参数
if [ "$CLEAN_OPTION" == "--clean" ]; then
    echo "� Cleaning workspace at: $WS_DIR"
    rm -rf "$WS_DIR/build" "$WS_DIR/devel" "$WS_DIR/CMakeCache.txt"
else
    echo "ℹ️  Skipping workspace cleanup. To clean, use: --clean"
fi

# 编译
echo "🚀 Running catkin_make with PYTHON_EXECUTABLE=$PYTHON_PATH ..."
catkin_make -DPYTHON_EXECUTABLE="$PYTHON_PATH" -DCATKIN_WHITELIST_PACKAGES="act_dp_service" 

# 检查构建是否成功
if [ $? -eq 0 ]; then
    echo "🎉 Build successful."
else
    echo "❌ Build failed."
    exit 1
fi
