#!/bin/bash
# 脚本功能：清理out目录缓存 -> cmake编译 -> 编译项目 -> 仅保存可执行文件到Ubuntu本地测试目录
# 适配：Ubuntu x86_64 原生编译

# ===================== 核心配置区（可根据需要修改，一目了然） =====================
WORK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"  # 脚本所在绝对路径
OUT_DIR="${WORK_DIR}/../out"                              # out目录绝对路径
USER_DIR="${WORK_DIR}"                                    # user目录绝对路径
MAKE_JOBS=4                                               # make编译线程数（可根据CPU核心数调整）
RESERVE_FILE="本文件夹作用.txt"                            # 保留的文件，清理时不删除
LOCAL_TEST_PATH="${WORK_DIR}/../ubuntu_test_bin"          # Ubuntu本地测试目录
EXECUTABLE_NAME="project_test_in_ubuntu"                  # 编译产物：可执行文件名称

# ===================== 全局通用函数 =====================
# 错误退出函数：打印错误信息 + 退出脚本（退出码1）
error_exit() {
    echo -e "\033[31m[ERROR] $1\033[0m"
    exit 1
}

# 成功提示函数：打印普通成功信息
info_echo() {
    echo -e "\033[32m[INFO] $1\033[0m"
}

# 检查依赖函数：校验编译器+OpenCV
check_dependency() {
    info_echo "开始检查编译依赖..."
    # 检查gcc/g++是否安装（Ubuntu原生编译核心依赖）
    if ! command -v gcc &> /dev/null || ! command -v g++ &> /dev/null; then
        error_exit "gcc/g++未安装！请执行：sudo apt install gcc g++"
    fi
    # 检查OpenCV
    if ! pkg-config --exists opencv4 && ! pkg-config --exists opencv; then
        error_exit "OpenCV未安装！请执行：sudo apt install libopencv-dev"
    fi
    info_echo "✅ 所有编译依赖检查通过（gcc/g++ + OpenCV）"
}

# ===================== 脚本主逻辑 =====================
# 1. 前置依赖检查
check_dependency

# 2. 进入out目录并校验
info_echo "准备进入目录: ${OUT_DIR}"
mkdir -p "${OUT_DIR}" || error_exit "无法创建 ${OUT_DIR} 目录，请检查权限！"
cd "${OUT_DIR}" || error_exit "无法进入 ${OUT_DIR} 目录，请检查目录是否存在！"

# # 3. 清理out目录下cmake缓存，仅保留指定文件
# info_echo "开始清理cmake缓存，仅保留 ${RESERVE_FILE}"
# find . -mindepth 1 \( ! -name "${RESERVE_FILE}" -a \( -name "CMakeCache.txt" -o -name "CMakeFiles" -o -name "cmake_install.cmake" -o -name "Makefile" \) \) -exec rm -rf {} + || error_exit "缓存清理失败，请检查目录权限！"

# 4. 执行cmake编译（使用系统默认编译器，不指定交叉编译）
info_echo "执行cmake编译: cmake ${USER_DIR}"
cmake "${USER_DIR}" || error_exit "cmake 编译失败，请检查CMakeLists.txt或编译依赖！"

# 5. 执行make多线程编译
info_echo "cmake执行成功，开始执行 make -j${MAKE_JOBS} 编译项目..."
make -j${MAKE_JOBS} || error_exit "make 编译失败，请查看编译日志！"

# 6. 查找可执行文件
info_echo "开始查找可执行文件: ${EXECUTABLE_NAME}"
EXEC_FILE_PATH=""
if [ -f "${OUT_DIR}/${EXECUTABLE_NAME}" ]; then
    EXEC_FILE_PATH="${OUT_DIR}/${EXECUTABLE_NAME}"
elif [ -f "${EXECUTABLE_NAME}" ]; then
    EXEC_FILE_PATH="${EXECUTABLE_NAME}"
else
    error_exit "未找到可执行文件 ${EXECUTABLE_NAME}，请检查编译产物名称！"
fi
info_echo "✅ 找到可执行文件：${EXEC_FILE_PATH}"

# 7. 创建本地测试目录并拷贝文件
info_echo "开始保存文件到Ubuntu本地测试目录: ${LOCAL_TEST_PATH}"
mkdir -p "${LOCAL_TEST_PATH}" || error_exit "本地测试目录创建失败！"
cp -f "${EXEC_FILE_PATH}" "${LOCAL_TEST_PATH}/" || error_exit "可执行文件拷贝失败，请检查权限！"

# 8. 打印文件信息（验证是否为x86_64架构）
info_echo "==================== 可执行文件信息 ===================="
ls -lh "${LOCAL_TEST_PATH}/${EXECUTABLE_NAME}"
file "${LOCAL_TEST_PATH}/${EXECUTABLE_NAME}"
info_echo "========================================================"

# 9. 全部执行完成
info_echo "✅ 所有操作执行完成！可执行文件已保存到：${LOCAL_TEST_PATH}/${EXECUTABLE_NAME}"
info_echo "🔧 现在可以直接运行：${LOCAL_TEST_PATH}/${EXECUTABLE_NAME}"
exit 0