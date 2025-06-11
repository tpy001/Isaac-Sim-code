#!/bin/bash

NUM_TRIALS=100
SUCCESS_COUNT=0
OUTPUT_BASE="./mutli_task_results"
CONFIG_YAML="./configs/stack_cube_dp_v3_depth_mutli_task.yaml"

# 加载 conda 初始化
source ~/miniconda3/etc/profile.d/conda.sh

for ((i=1; i<=NUM_TRIALS; i++)); do
    echo "Running trial $i/$NUM_TRIALS"
    # roscore
    # sleep 5  # 等待节点启动
    
    TRIAL_DIR="$OUTPUT_BASE/trial_$(printf "%03d" $i)"
    mkdir -p $TRIAL_DIR
    chmod -R 777 $TRIAL_DIR

    # 复制基础配置文件
    cp $CONFIG_YAML "$TRIAL_DIR/config.yaml"

    # 使用sed替换save_dir字段
    sed -i "s|save_dir:.*|save_dir: \"$TRIAL_DIR\"|" "$TRIAL_DIR/config.yaml"

    # 环境激活与dp算法执行
    (
        cd umi 
        conda activate umi  
        ./test.sh > "../$TRIAL_DIR/dp.log" 2>&1 &  # dp算法后台运行
        DP_PID=$!
    )  # 使用子shell避免目录切换影响
    
    # 使用生成的配置文件
    ./run.sh $TRIAL_DIR config > "$TRIAL_DIR/isaac.log" 2>&1 &
    ISAAC_PID=$!
    
    # ./run.sh  stack_cube_dp_v3_depth_mutli_task > "$TRIAL_DIR/isaac.log" 2>&1 &
    # ISAAC_PID=$!
    
    sleep 10


    # 等待120秒后终止所有相关进程
    echo "Trial $i: Waiting 60s before terminating processes..."
    sleep 60
    
    # 终止Isaac进程
    kill $ISAAC_PID 2>/dev/null
    sleep 1
    kill -9 $ISAAC_PID 2>/dev/null
    
    # 强制终止dp算法进程
    echo "Trial $i: Force killing dp algorithm process"
    kill -9 $DP_PID 2>/dev/null  # 直接发送SIGKILL
    sleep 1
    # echo "now in $(pwd)"
    
    # 清理所有相关进程（模糊匹配进程名，处理可能的子进程）
    # echo "Trial $i: Cleaning up related processes"
    # pkill -9 -f "test.sh" 2>/dev/null    # 终止test.sh相关进程
    # pkill -9 -f "run.sh" 2>/dev/null    # 终止run.sh相关进程

    # GPU显存强制清理
    echo "Trial $i: Cleaning GPU processes"
    nvidia-smi --query-compute-apps=pid --format=csv,noheader | xargs -I{} kill -9 {} 2>/dev/null
    
    # 等待所有进程退出
    wait $ISAAC_PID 2>/dev/null
    wait $DP_PID 2>/dev/null
    
    # 检查进程终止状态
    ISAAC_STATUS=$?
    DP_STATUS=$?
    
    if [ $ISAAC_STATUS -eq 0 ]; then
        echo "Trial $i: Isaac process completed normally"
    else
        echo "Trial $i: Isaac process terminated with status: $ISAAC_STATUS"
    fi
    
    if [ $DP_STATUS -eq 0 ]; then
        echo "Trial $i: dp algorithm completed normally"
    else
        echo "Trial $i: dp algorithm terminated with status: $DP_STATUS"
    fi

    # 检查成功状态
    if [ -f $TRIAL_DIR/meta/success.txt ]; then
        SUCCESS=$(cat $TRIAL_DIR/meta/success.txt)
        if [ "$SUCCESS" -eq 1 ]; then
            ((SUCCESS_COUNT++))
            echo "Trial $i: Success"
        else
            echo "Trial $i: Failed"
        fi
    else
        echo "Trial $i: No result file"
    fi
    
    # 资源监控（新增）
    echo "Memory status:"
    free -h
    echo "GPU status:"
    nvidia-smi --query-gpu=memory.used --format=csv

    # 清理缓存
    sync
    sleep 2
done

# 计算成功率
SUCCESS_RATE=$(echo "scale=2; $SUCCESS_COUNT/$NUM_TRIALS*100" | bc)
echo "=============================="
echo "Total trials: $NUM_TRIALS"
echo "Success count: $SUCCESS_COUNT"
echo "Success rate: $SUCCESS_RATE%"

