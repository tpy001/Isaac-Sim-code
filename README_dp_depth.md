## Isaac Sim + DP 深度
环境安装：获取源代码，然后下载 dp+depth 的模型权重，放在 umi/ckpt 目录下，名字为 latest.ckpt
#### 1. 编译 ROS 命令，支持自定义的消息
```
./ros_msg_compile.sh /path/to/isaacsim/root # 替换为 isaac sim 的安装目录
```
#### 2. 运行数据采集代码
```
./run.sh stack_cube_data_collect
```
采集得到的数据位于 data_collect/目录下
#### 3. 运行数据转化的代码
```
    conda activate umi
    python to_umi_format.py
```
需要修改这些参数:
```
    input_dir = "./data/stack_cube_v3_with_depth"  # CSV 和 PNG 文件所在的目录
    output_zarr = "./train_data.zarr.zip"
    epoch_nums = 51
```
#### 4. 运行模型训练的代码
```
    conda activate umi
    cd umi
    ./train.sh
```
#### 5. 运行推理的代码
(1) 启动 isaac sim 客户端
```
    ./run.sh stack_cube_dp_v3_depth
```
(2) 启动 DP 算法作为服务端
```
    conda activate umi
    ./test.sh
```