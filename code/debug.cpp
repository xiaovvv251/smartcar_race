#include "cross.hpp"
#include "image_process.hpp"
#include "zf_common_headfile.hpp"
#include <string>
#include <iostream>

// 调试文字配色
const cv::Scalar COLOR_DEBUG_TITLE = cv::Scalar(0, 255, 255);   // 黄标题
const cv::Scalar COLOR_DEBUG_NORMAL = cv::Scalar(255, 255, 255); // 白文字
const cv::Scalar COLOR_DEBUG_WARN = cv::Scalar(0, 160, 255);    // 橙警告
const cv::Scalar COLOR_DEBUG_SUCCESS = cv::Scalar(0, 255, 0);     // 绿成功

// 十字状态转字符串
static std::string cross_type_to_str(CrossType type) {
    switch(type) {
        case CrossType::CROSS_NONE: return "NONE";
        case CrossType::CROSS_BEGIN: return "BEGIN";
        case CrossType::CROSS_IN: return "IN_CROSS";
        default: return "UNKNOWN";
    }
}

// 绘制调试信息（完全保留你原版正常布局，只补关键信息）
void draw_debug_info(cv::Mat& canvas, 
                    TrackImageProcessor& proc, 
                    CrossInfo& cross_info,
                    bool regrow_flag,
                    int frame_count) {
    // 绘制起始坐标（你原版正常位置，完全不动）
    int x = 1000;
    int y = 30;
    int line_height = 25;

    // 标题（不动）
    cv::putText(canvas, "=== DEBUG INFO ===", cv::Point(x, y), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, COLOR_DEBUG_TITLE, 1);
    y += line_height;

    // 1. 基础帧信息（不动）
    cv::putText(canvas, "Frame: " + std::to_string(frame_count), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;
    cv::putText(canvas, "Threshold: " + std::to_string(proc.threshold), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 2. 【新增】近线点数（一眼看近线有没有断）
    cv::putText(canvas, "Near-L pts: " + std::to_string(proc.left_border.resampled.size()), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;
    cv::putText(canvas, "Near-R pts: " + std::to_string(proc.right_border.resampled.size()), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 3. 边界角点状态（不动）
    cv::putText(canvas, "Left L-Corner: " + std::string(proc.left_border.l_corners.empty() ? "NO" : "YES"), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                proc.left_border.l_corners.empty() ? COLOR_DEBUG_WARN : COLOR_DEBUG_SUCCESS, 1);
    y += line_height;
    cv::putText(canvas, "Right L-Corner: " + std::string(proc.right_border.l_corners.empty() ? "NO" : "YES"), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                proc.right_border.l_corners.empty() ? COLOR_DEBUG_WARN : COLOR_DEBUG_SUCCESS, 1);
    y += line_height;

    // 4. 重生长状态（不动）
    cv::putText(canvas, "Regrow: " + std::string(regrow_flag ? "YES" : "NO"), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                regrow_flag ? COLOR_DEBUG_SUCCESS : COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 5. 十字状态（不动）
    cv::putText(canvas, "Cross: " + cross_type_to_str(cross_info.type), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_TITLE, 1);
    y += line_height;

    // 6. 十字丢线计数（不动）
    cv::putText(canvas, "Cross No-Line: " + std::to_string(cross_info.no_line_cnt), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cross_info.no_line_cnt > 0 ? COLOR_DEBUG_WARN : COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 7. 视觉关键数据（不动）
    cv::putText(canvas, "Hightest Row: " + std::to_string(proc.hightest), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;
    cv::putText(canvas, "Track Error: " + std::to_string(proc.error), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 8. 【新增】远线点数（一眼看远线有没有生成）
    cv::putText(canvas, "Far-L pts: " + std::to_string(cross_info.far_left.resampled.size()), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;
    cv::putText(canvas, "Far-R pts: " + std::to_string(cross_info.far_right.resampled.size()), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_DEBUG_NORMAL, 1);
    y += line_height;

    // 9. 远线角点状态（不动）
    cv::putText(canvas, "Far-Left-line: " + std::string(cross_info.far_L ? "YES" : "NO"), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cross_info.far_L ? COLOR_DEBUG_SUCCESS : COLOR_DEBUG_NORMAL, 1);
    y += line_height;
    cv::putText(canvas, "Far-Right-line: " + std::string(cross_info.far_R ? "YES" : "NO"), 
                cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                cross_info.far_R ? COLOR_DEBUG_SUCCESS : COLOR_DEBUG_NORMAL, 1);
}