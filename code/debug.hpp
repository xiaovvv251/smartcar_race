#ifndef DEBUG_HPP
#define DEBUG_HPP
class TrackImageProcessor;
class CrossInfo;
#include <opencv2/opencv.hpp>

// 调试信息绘制：画在黑色画布空白处，简洁不遮挡图像
void draw_debug_info(cv::Mat& canvas, 
                    TrackImageProcessor& proc, 
                    CrossInfo& cross_info,
                    bool regrow_flag,
                    int frame_count = 0);

#endif