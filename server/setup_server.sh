#!/bin/bash

# 遇到错误立即停止
set -e

# --- 1. 安装 Miniconda (如果尚未安装) ---
if ! command -v conda &> /dev/null; then
    echo "未检测到 Conda，正在下载并安装 Miniconda..."
    wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh -O miniconda.sh
    bash miniconda.sh -b -p $HOME/miniconda
    rm miniconda.sh
    # 初始化当前 shell 的 conda 环境
    source "$HOME/miniconda/etc/profile.d/conda.sh"
    conda init bash
    echo "Conda 安装完成。"
else
    echo "Conda 已安装，跳过此步。"
    # 确保脚本能使用 conda 指令
    CONDA_PATH=$(which conda)
    source $(dirname $CONDA_PATH)/../etc/profile.d/conda.sh
fi

# --- 先安装编译依赖，再创建 Conda 环境 ---

echo "正在安装编译 lxml 所需的系统依赖 (libxml2, libxslt)..."
# 加上 -y 确保全自动运行
sudo apt-get update || true
sudo apt-get install -y --fix-missing libxml2-dev libxslt-dev python3-dev

# --- 2. 根据 environment.yaml 创建并启动环境 ---
if [ ! -f "environment.yaml" ]; then
    echo "错误: 当前目录下未找到 environment.yaml 文件！"
    exit 1
fi



# 提取 yaml 中的环境名称 (假设格式为 name: env_name)
ENV_NAME=$(grep 'name:' environment.yaml | head -n 1 | awk '{print $2}')
echo "正在创建/更新 Conda 环境: $ENV_NAME ..."
conda env create -f environment.yaml || conda env update -f environment.yaml

# 激活环境
conda activate $ENV_NAME
echo "环境 $ENV_NAME 已激活。"

# --- 3. 安装 PM2 (需要 Node.js 环境) ---
if ! command -v pm2 &> /dev/null; then
    echo "正在安装 PM2 (需要 sudo 权限安装 Node/NPM)..."
    # 这里假设是 Debian/Ubuntu 系统，如果不是请手动先装 node
    if ! command -v npm &> /dev/null; then
        sudo apt-get update && sudo apt-get install -y nodejs npm
    fi
    sudo npm install -g pm2
else
    echo "PM2 已安装。"
fi

# --- 4. 用 PM2 启动 server.py ---
echo "正在使用 PM2 启动 $ENV_NAME 环境下的 server.py ..."
# 注意：这里使用 --interpreter 指向 conda 环境中的 python 路径，避免路径混
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON_PATH=$(which python)  
pm2 start server.py --name "vla-server-prod" --interpreter "$PYTHON_PATH" --cwd "$SCRIPT_DIR"

# --- 5. 设置开机自启并保存 ---
echo "设置 PM2 开机自启动..."
# 生成自启动脚本（会自动执行并获取需要的指令）
pm2 startup | grep "sudo" | bash
pm2 save

echo "------------------------------------------------"
echo "部署完成！"
echo "环境名称: $ENV_NAME"
echo "服务状态: pm2 list"
echo "查看日志: pm2 logs vla-server-prod"
echo "------------------------------------------------"
