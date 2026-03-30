#include "cross.hpp"
#include <algorithm>
#include "zf_common_headfile.hpp"
#include "image_process.hpp"
// 检测双L角点 → 进入十字

CrossInfo cross_info;

void check_cross(TrackImageProcessor& proc) {
    if (cross_info.type != CrossType::CROSS_NONE) return;

    bool has_left_L = !proc.left_border.l_corner_indices.empty();
    bool has_right_L = !proc.right_border.l_corner_indices.empty();

    if (has_left_L && has_right_L) {
        cross_info.type = CrossType::CROSS_BEGIN;
    }
}

// 运行十字状态机
void run_cross(TrackImageProcessor& proc, uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]) {
    auto& cross = cross_info;

    // 开始阶段：切远线
    if (cross.type == CrossType::CROSS_BEGIN) {
        bool left_short = proc.left_border.resampled.size() < 15;
        bool right_short = proc.right_border.resampled.size() < 15;
        if (left_short || right_short) {
            cross.type = CrossType::CROSS_IN;
            cross.no_line_cnt = 0;
        }
    }

    // 远线追踪阶段
    if (cross.type == CrossType::CROSS_IN) {
        cross_farline(proc, bin_img);

        bool left_ok = cross.far_left.resampled.size() > 8;
        bool right_ok = cross.far_right.resampled.size() > 8;

        // 计数：左右都没线才计数
        if (!left_ok && !right_ok) {
            cross.no_line_cnt++;
        } else {
            // 只要有一边有线，就重置丢线计数
            cross.no_line_cnt = 0;
        }


        // 1. 连续丢线 >5 帧 → 直接退出（已过十字）
        // 2. 近线恢复正常（左右都>15点）→ 退出十字，回到普通循线
        bool near_line_ok = (proc.left_border.resampled.size() > 15 && proc.right_border.resampled.size() > 15);
        if (cross.no_line_cnt > 5 || near_line_ok) {
            cross.type = CrossType::CROSS_NONE;
            cross.no_line_cnt = 0;
        }
    }
}

// 远线追踪（完全复用你现有 process_border）
void cross_farline(TrackImageProcessor& proc, uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]) {
    auto& cross = cross_info;
    cross.far_left.clear();
    cross.far_right.clear();
    cross.far_L = false;
    cross.far_R = false;

    // 找远线起点（上半部分搜索）
    Point2i start_left(-1, -1);
    Point2i start_right(-1, -1);

    const int search_y = 40;
    for (int y = search_y; y > proc.TRACK_STOP_ROW; y--) {
        if (start_left.x < 0 && proc.get_pixel(bin_img, cross.far_x_left, y) == proc.WHITE) {
            if (proc.get_pixel(bin_img, cross.far_x_left, y-1) == proc.BLACK) {
                start_left = Point2i(cross.far_x_left, y); // 修复：用构造函数
            }
        }
        if (start_right.x < 0 && proc.get_pixel(bin_img, cross.far_x_right, y) == proc.WHITE) {
            if (proc.get_pixel(bin_img, cross.far_x_right, y-1) == proc.BLACK) {
                start_right = Point2i(cross.far_x_right, y); // 修复：用构造函数
            }
        }
    }

    // 追踪远线（复用你成熟逻辑）
    if (start_left.x >= 0) {
        cross.far_left = proc.process_border(bin_img, start_left, true);
        if (!cross.far_left.l_corner_indices.empty())
            cross.far_L = true;
    }
    if (start_right.x >= 0) {
        cross.far_right = proc.process_border(bin_img, start_right, false);
        if (!cross.far_right.l_corner_indices.empty())
            cross.far_R = true;
    }
}