# ===================== 注释/删除交叉编译相关配置 =====================
# 注释掉交叉编译工具链（核心修改1：不再使用龙芯交叉编译）
# include(cross.cmake)

# 设置CMake的最低版本要求
cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -Wall")  # 对于 C++ 编译器
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall")      # 对于 C 编译器

# 获取当前 CMakeLists.txt 文件所在目录的完整路径
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})

# 获取上上级目录的完整路径（保留，但不再用于项目名称）
get_filename_component(PARENT_PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)
get_filename_component(PARENT_PARENT_FOLDER_NAME ${PARENT_PARENT_DIR} NAME)
message("Parent parent folder name: ${PARENT_PARENT_FOLDER_NAME}")

# ===================== 固定可执行文件名称（核心修改2） =====================
# 不再动态用文件夹名，直接指定你要的名称 project_test_in_ubuntu
project(project_test_in_ubuntu VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ===================== 使用Ubuntu系统的OpenCV（核心修改3） =====================
# 注释掉龙芯架构的OpenCV路径，使用Ubuntu系统默认安装的OpenCV
# set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 

# 查找OpenCV库（自动找Ubuntu系统的x86_64版本）
find_package(OpenCV REQUIRED)

# 包含OpenCV的头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

# 包含项目的自定义库路径（路径不变，确保源码存在）
include_directories(../user)
include_directories(../code)
include_directories(../../libraries/zf_common)
include_directories(../../libraries/zf_device)
include_directories(../../libraries/zf_driver)
include_directories(../../libraries/zf_components)

# 从目录下查找C/C++文件
aux_source_directory(../user DIR_SRCS)
aux_source_directory(../code DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components DIR_SRCS)

# 创建可执行文件（使用固定名称 project_test_in_ubuntu）
add_executable(${PROJECT_NAME} ${DIR_SRCS})

# 链接OpenCV库到可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})