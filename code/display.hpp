#ifndef DISPLAY_HPP
#define DISPLAY_HPP
#include <opencv2/opencv.hpp>
#include "image_process.hpp"
#include "zf_common_headfile.hpp"

// 颜色定义 (BGR)
#define COLOR_BLACK       cv::Scalar(0, 0, 0)
#define COLOR_WHITE       cv::Scalar(255, 255, 255)
#define COLOR_RED         cv::Scalar(0, 0, 255)
#define COLOR_GREEN       cv::Scalar(0, 255, 0)
#define COLOR_BLUE        cv::Scalar(255, 0, 0)
#define COLOR_YELLOW      cv::Scalar(0, 255, 255)
#define COLOR_CYAN        cv::Scalar(255, 255, 0)
#define COLOR_MAGENTA     cv::Scalar(255, 0, 255)

// ====================== 绘制参数宏（全配置，无魔法值）======================
#define CORNER_CIRCLE_SIZE    6    // 角点圆圈大小
#define CORNER_CIRCLE_THICK   2    // 角点圆圈粗细
#define START_POINT_SIZE      2    // 起点圆点大小

// 显示调试类
class Display {
public:
    Display(const cv::Size& main_size = cv::Size(320, 480), 
            const cv::Size& bev_size = cv::Size(640, 480));

    void draw(uint8_t gray[UVC_HEIGHT][UVC_WIDTH],
              uint8_t bin[UVC_HEIGHT][UVC_WIDTH],
              TrackImageProcessor& proc,
              cv::Mat& out_main_img,
              cv::Mat& out_bev_img);

    void show(const cv::Mat& main_img, const cv::Mat& bev_img);
    void show(const cv::Mat& main_img, const cv::Mat& bev_img, TrackImageProcessor& proc);
    
    void close();

private:
    cv::Size m_main_size;
    cv::Size m_bev_size;
    
    void drawOnGray(uint8_t gray[UVC_HEIGHT][UVC_WIDTH], cv::Mat& out, TrackImageProcessor& proc);
    void drawOnBin(uint8_t bin[UVC_HEIGHT][UVC_WIDTH], cv::Mat& out, TrackImageProcessor& proc);

    void drawBEV(TrackImageProcessor& proc, cv::Mat& out);
    void drawCornerCircle(cv::Mat& img, int x, int y, const cv::Scalar& color);
    void drawStartPoint(cv::Mat& img, int x, int y, const cv::Scalar& color);
    void drawResampledBorder(cv::Mat& img, TrackImageProcessor& proc);
};

#endif // DISPLAY_HPP