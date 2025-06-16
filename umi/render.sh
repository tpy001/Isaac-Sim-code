#!/bin/bash

# 批量处理Isaac Sim数据的脚本
# 适用于处理多个epo目录下的相机数据

# 配置参数
BASE_DATA_DIR="/home/pinkman/projects/dp_depth/data_collect"  # 数据根目录
OUTPUT_BASE_DIR="/home/pinkman/projects/dp_depth/data_collect"  # 输出根目录
ENCODER="vitl"  # 编码器类型
INPUT_SIZE="480"  # 输入图像尺寸
PRED_ONLY=true
GRAYSCALE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --base-data-dir)
            BASE_DATA_DIR="$2"
            shift 2
            ;;
        --output-base-dir)
            OUTPUT_BASE_DIR="$2"
            shift 2
            ;;
        --encoder)
            ENCODER="$2"
            shift 2
            ;;
        --input-size)
            INPUT_SIZE="$2"
            shift 2
            ;;
        --pred-only)
            PRED_ONLY=true  # 当参数出现时设为true
            shift
            ;;
        --grayscale)
            GRAYSCALE=true  # 当参数出现时设为true
            shift
            ;;
        *)
            echo "错误: 未知参数 $1" >&2
            exit 1
            ;;
    esac
done

# 确保输出目录存在
mkdir -p "$OUTPUT_BASE_DIR"

# 遍历所有epo目录
for epo_dir in "$BASE_DATA_DIR"/epo_*; do
    if [ -d "$epo_dir" ]; then
        epo_name=$(basename "$epo_dir")
        img_path="$epo_dir/camera"
        outdir="$OUTPUT_BASE_DIR/${epo_name}/depth_map"
        mkdir -p "$outdir"
        
        echo "处理目录: $epo_name"
        
        # 构建基础命令
        cmd="python render_multipe_raw_img.py --img-path \"$img_path\" \
                          --input-size $INPUT_SIZE \
                          --outdir \"$outdir\" \
                          --encoder $ENCODER" \
        
        # 根据变量值添加布尔参数（核心逻辑）
        if [ "$PRED_ONLY" == true ]; then
            cmd+=" --pred-only"
        fi
        
        if [ "$GRAYSCALE" == true ]; then
            cmd+=" --grayscale"
        fi
        
        # 执行命令并打印调试信息
        echo "执行: $cmd"
        if eval "$cmd"; then
            echo "✓ $epo_name 处理完成"
        else
            echo "✗ $epo_name 处理失败"
        fi
        
        echo "------------------------"
    fi
done

echo "所有目录处理完毕"





