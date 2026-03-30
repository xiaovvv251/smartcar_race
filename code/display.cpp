#include "display.hpp"
#include "debug.hpp" 
#include "zf_common_headfile.hpp"
#include "image_process.hpp"
#include "cross.hpp"
// 构造函数
Display::Display(const cv::Size& main_size, const cv::Size& bev_size) 
    : m_main_size(main_size), m_bev_size(bev_size) {}

// 绘制角点圆圈
void Display::drawCornerCircle(cv::Mat& img, int x, int y, const cv::Scalar& color) {
    cv::circle(img, cv::Point(x, y), 3, color, -1);
}

// 绘制逆透视边线
void Display::drawResampledBorder(cv::Mat& img, TrackImageProcessor& proc) {
    auto left_res_img = proc.inverse_perspective_transform(proc.left_border.resampled);
    for (auto& p : left_res_img) {
        if (p.x >= 0 && p.x < img.cols && p.y >= 0 && p.y < img.rows) {
            img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(COLOR_CYAN[0], COLOR_CYAN[1], COLOR_CYAN[2]);
        }
    }
    auto right_res_img = proc.inverse_perspective_transform(proc.right_border.resampled);
    for (auto& p : right_res_img) {
        if (p.x >= 0 && p.x < img.cols && p.y >= 0 && p.y < img.rows) {
            img.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(COLOR_MAGENTA[0], COLOR_MAGENTA[1], COLOR_MAGENTA[2]);
        }
    }
}

// 在灰度图上绘制
void Display::drawOnGray(uint8_t gray[UVC_HEIGHT][UVC_WIDTH], cv::Mat& out, TrackImageProcessor& proc) {
    cv::Mat gray_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, gray);
    cv::cvtColor(gray_mat, out, cv::COLOR_GRAY2BGR);

    for (auto& p : proc.left_border.points) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 255, 0);
        }
    }
    for (auto& p : proc.right_border.points) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 0, 255);
        }
    }

    drawResampledBorder(out, proc);

    for (auto& p : proc.left_border.l_corners_img) {
        drawCornerCircle(out, p.x, p.y, COLOR_YELLOW);
    }
    for (auto& p : proc.right_border.l_corners_img) {
        drawCornerCircle(out, p.x, p.y, COLOR_YELLOW);
    }
}

// 在二值图上绘制
void Display::drawOnBin(uint8_t bin[UVC_HEIGHT][UVC_WIDTH], cv::Mat& out, TrackImageProcessor& proc) {
    cv::Mat bin_mat(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, bin);
    cv::cvtColor(bin_mat, out, cv::COLOR_GRAY2BGR);

    for (auto& p : proc.left_border.points) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 255, 0);
        }
    }
    for (auto& p : proc.right_border.points) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(0, 0, 255);
        }
    }

    drawResampledBorder(out, proc);

    for (auto& p : proc.left_border.center_img) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
        }
    }
    for (auto& p : proc.right_border.center_img) {
        if (p.x >= 0 && p.x < out.cols && p.y >= 0 && p.y < out.rows) {
            out.at<cv::Vec3b>(p.y, p.x) = cv::Vec3b(255, 0, 0);
        }
    }

    for (auto& p : proc.left_border.l_corners_img) {
        drawCornerCircle(out, p.x, p.y, COLOR_YELLOW);
    }
    for (auto& p : proc.right_border.l_corners_img) {
        drawCornerCircle(out, p.x, p.y, COLOR_YELLOW);
    }
}

// 绘制鸟瞰图
void Display::drawBEV(TrackImageProcessor& proc, cv::Mat& out) {
    out = cv::Mat::zeros(m_bev_size, CV_8UC3);
    float scale = 2.0f;
    int offset_x = m_bev_size.width / 4;
    int offset_y = m_bev_size.height - 320;
    
    for (const auto& bev_p : proc.left_border.resampled) {
        int canvas_x = (int)(bev_p.x * scale + offset_x);
        int canvas_y = (int)(bev_p.y * scale + offset_y);
        if (canvas_x >= 0 && canvas_x < out.cols && canvas_y >= 0 && canvas_y < out.rows) {
            cv::circle(out, cv::Point(canvas_x, canvas_y), 1, COLOR_CYAN, -1);
        }
    }
    for (const auto& bev_p : proc.right_border.resampled) {
        int canvas_x = (int)(bev_p.x * scale + offset_x);
        int canvas_y = (int)(bev_p.y * scale + offset_y);
        if (canvas_x >= 0 && canvas_x < out.cols && canvas_y >= 0 && canvas_y < out.rows) {
            cv::circle(out, cv::Point(canvas_x, canvas_y), 1, COLOR_MAGENTA, -1);
        }
    }
    for (const auto& bev_p : proc.left_border.center_line) {
        int canvas_x = (int)(bev_p.x * scale + offset_x);
        int canvas_y = (int)(bev_p.y * scale + offset_y);
        if (canvas_x >= 0 && canvas_x < out.cols && canvas_y >= 0 && canvas_y < out.rows) {
            cv::circle(out, cv::Point(canvas_x, canvas_y), 1, COLOR_BLUE, -1);
        }
    }
    for (const auto& bev_p : proc.right_border.center_line) {
        int canvas_x = (int)(bev_p.x * scale + offset_x);
        int canvas_y = (int)(bev_p.y * scale + offset_y);
        if (canvas_x >= 0 && canvas_x < out.cols && canvas_y >= 0 && canvas_y < out.rows) {
            cv::circle(out, cv::Point(canvas_x, canvas_y), 1, COLOR_BLUE, -1);
        }
    }
}

// 主绘制函数
void Display::draw(uint8_t gray[UVC_HEIGHT][UVC_WIDTH], uint8_t bin[UVC_HEIGHT][UVC_WIDTH], 
                   TrackImageProcessor& proc, cv::Mat& out_main, cv::Mat& out_bev) {
    cv::Mat gray_marked, bin_marked;
    
    drawOnGray(gray, gray_marked, proc);
    drawOnBin(bin, bin_marked, proc);
    
    cv::vconcat(gray_marked, bin_marked, out_main);
    cv::resize(out_main, out_main, m_main_size);
    
    drawBEV(proc, out_bev);
}

// 显示窗口（修改为全屏黑色画布）
void Display::show(const cv::Mat& main_img, const cv::Mat& bev_img, TrackImageProcessor& proc) {
    // 创建全屏黑色画布
    cv::Size screen_size(1920, 1440); // 假设屏幕分辨率为 1280x720
    cv::Mat canvas = cv::Mat::zeros(screen_size, CV_8UC3);
    // 将 main_img 放在画布左侧
    cv::Rect main_rect(0, 0, main_img.cols, main_img.rows);
    main_img.copyTo(canvas(main_rect));
    // 将 bev_img 放在画布右侧
    cv::Rect bev_rect(main_img.cols, 0, bev_img.cols, bev_img.rows);
    bev_img.copyTo(canvas(bev_rect));
    draw_debug_info(canvas, proc, cross_info, proc.process_return_flag, proc.frame_count);
    // 显示合并后的画布
    cv::imshow("Combined View", canvas);
}

// 销毁窗口
void Display::close() {
    cv::destroyAllWindows();
}