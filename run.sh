isaac_sim_root=/home/pinkman/isaacsim

workspaceFolder="$(pwd)"

# 1. 设置环境变量
export RESOURCE_NAME="IsaacSim"

# 2. 读取 .env 文件并加载环境变量
ENV_FILE="${PWD}/.vscode/.standalone_examples.env"
if [[ -f "$ENV_FILE" ]]; then
    while IFS= read -r line; do
        # 跳过空行和以 # 开头的注释行
        [[ -z "$line" || "$line" == \#* ]] && continue

        # 提取键值对
        VAR_NAME=$(echo "$line" | cut -d= -f1)
        VAR_VALUE=$(echo "$line" | cut -d= -f2-)

        # 处理可能包含空格的值
        export "$VAR_NAME=$VAR_VALUE"
    done < "$ENV_FILE"
fi

# 3. 指定 Python 解释器
PYTHON_EXECUTABLE="${isaac_sim_root}/kit/python/bin/python3"

# 4. 运行前置任务
export CARB_APP_PATH=${isaac_sim_root}/kit 
export ISAAC_PATH=${isaac_sim_root} 
export EXP_PATH=${isaac_sim_root}/apps 
source ${isaac_sim_root}/setup_python_env.sh 
printenv >${workspaceFolder}/.vscode/.standalone_examples.env

echo "$PYTHON_EXECUTABLE"

source ./catkin_ws/devel/setup.bash
# 5. 运行 Python 脚本

# config=stack_cube_dp_v3_depth
config_path=$1
config_name=$2


"$PYTHON_EXECUTABLE" main.py --config-path=$config_path --config-name=$config_name