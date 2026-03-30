#ifndef CROSS_HPP
#define CROSS_HPP
#include <cstdint>
#include "zf_common_headfile.hpp"
#include "image_process.hpp"   // 已包含 BorderLine 定义，无需重复定义


// ====================== 远线追踪参数配置（无魔法值，统一管理）======================
#define FAR_SEARCH_Y            40      // 远线搜索/绘制 同步Y坐标
#define FAR_X_LEFT              30      // 左远线搜索X坐标
#define FAR_X_RIGHT             130     // 右远线搜索X坐标

#define NEAR_LINE_SHORT_TH      15      // 近线过短阈值（进入十字IN）
#define FAR_LINE_VALID_TH       8       // 远线有效点数阈值
#define NO_LINE_MAX_CNT         5       // 丢线最大计数（退出十字）

// ====================== 十字状态枚举（合并完整版，唯一定义）======================
enum class CrossType {
    CROSS_NONE,    // 正常循线
    CROSS_BEGIN,   // 检测到双L角点，进入十字
    CROSS_IN       // 远线追踪模式
};

// ====================== 十字信息结构体（两份代码完全合并，唯一定义）======================
struct CrossInfo {
public:
    CrossType type = CrossType::CROSS_NONE;
    int no_line_cnt = 0;

    // 远线搜索坐标（使用宏初始化，无魔法值，搜索/绘制完全同步）
    int far_x_left  = FAR_X_LEFT;
    int far_x_right = FAR_X_RIGHT;

    BorderLine far_left;   // 十字远左线
    BorderLine far_right;  // 十字远右线
    bool far_L = false;    // 远左L角点
    bool far_R = false;    // 远右L角点

    // 清空状态（合并新增，保证状态重置干净）
    void clear() {
        type = CrossType::CROSS_NONE;
        no_line_cnt = 0;
        far_left.clear();
        far_right.clear();
        far_L = false;
        far_R = false;
    }
};

extern CrossInfo cross_info;
// ====================== 十字功能函数声明 ======================
void check_cross(TrackImageProcessor& proc);
void run_cross(TrackImageProcessor& proc, uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]);
void cross_farline(TrackImageProcessor& proc, uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]);

#endif