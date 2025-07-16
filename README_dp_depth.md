## Isaac Sim + DP 深度
环境安装：获取源代码，然后下载 dp+depth 的模型权重，放在 umi/ckpt 目录下，名字为 latest.ckpt
切换分支到dp_depth: 
```
git checkout dp_depth
```
#### 1. 编译 ROS 命令，支持自定义的消息
```
./ros_msg_compile.sh /path/to/isaacsim/root # 替换为 isaac sim 的安装目录
```
#### 2. 运行数据采集代码
```
./run.sh stack_cube_data_collect
```
采集得到的数据位于 data_collect/目录下

#### 3. 运行渲染深度图的代码
```
cd umi
./render.sh
```
需要修改这些参数:
```
    BASE_DATA_DIR="/home/pinkman/projects/dp_depth/data_collect"  # 数据根目录
    OUTPUT_BASE_DIR="/home/pinkman/projects/dp_depth/data_collect"  # 输出根目录
    ENCODER="vitl"  # 编码器类型
    INPUT_SIZE="480"  # 输入图像尺寸
    PRED_ONLY=true 
    GRAYSCALE=false  # 是否灰度化
```
采集得到的数据位于 data_collect/ep0_xxx/depth_maps/ 目录下;

在得到深度图之前，你需要先下载深度渲染算法的权重以及之后推理用到的dp模型权重:

| 文件名 | 下载地址 | 存放路径 | 说明 | 
| ----|  ----|   ----| ----| 
| depth_anything_v2_vits.pth | [下载链接]() | umi/ckpt/ | 较小参数量的模型权重|
| depth_anything_v2_vitb.pth | [下载链接](https://drive.usercontent.google.com/download?id=19iq23YNKJyqH0QQpBtrWrAGDbQPTnhpP&export=download&authuser=0&confirm=t&uuid=78d5d1ff-1d75-43b7-8a7b-5088339c33e9&at=AN8xHoqNCYLQKDmNaCrsE_I5XTxy:1750074426888) | umi/ckpt/ | 中等参数量的模型权重 |
| depth_anything_v2_vitl.pth | [下载链接](https://drive.usercontent.google.com/download?id=1kWv2Gs9bHmLU9p1oIKseEzatSuYMYMUz&export=download&authuser=0&confirm=t&uuid=89776719-968c-4280-a076-98ce331a905e&at=AN8xHorPFA0istfeSh2u8KNYsE6O:1750074479684) | umi/ckpt/ | 较大参数量的模型权重，通常意义上较大的参数量渲染效果更好，但具体的模型选择也取决于你的算力
| latest.ckpt/ | [下载链接](https://drive.usercontent.google.com/download?id=1xZNJ5pPyTqQu1hmmh9RcEE4usbBWvE7x&export=download&authuser=0&confirm=t&uuid=e1eb9133-8d94-4d52-bf05-5f6ecc95448e&at=AN8xHopFvgNqsViMmfaDR-_FjuZ-:1750074354360) | umi/ckpt | 纳入深度信息的dp模型权重 


#### 4. 运行数据转化的代码
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
#### 5. 运行模型训练的代码
```
    conda activate umi
    cd umi
    ./train.sh
```
#### 6. 运行推理的代码
(1) 启动 isaac sim 客户端
```
    ./run.sh stack_cube_dp_v3_depth
```
(2) 启动 DP 算法作为服务端
```
    conda activate umi
    ./test.sh
```
