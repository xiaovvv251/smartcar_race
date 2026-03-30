#include <iostream>
#include <string>
#include <vector>
#include <glob.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "image_process.hpp"
#include "display.hpp"

// 获取文件夹下所有BMP图片路径
std::vector<std::string> getBmpPaths(const std::string& folder) {
    std::vector<std::string> paths;
    glob_t glob_res;
    std::string pattern = folder + "/*.BMP";
    glob(pattern.c_str(), GLOB_TILDE, nullptr, &glob_res);
    for (size_t i = 500; i < glob_res.gl_pathc; ++i) {
        paths.push_back(glob_res.gl_pathv[i]);
    }
    globfree(&glob_res);
    std::sort(paths.begin(), paths.end());
    return paths;
}

int main() {
    // 配置：请修改此处为你的图包路径
    const std::string IMG_FOLDER = "/home/wzn/LS2K0300_Library/LS2K300_Library/Seekfree_LS2K0300_Opensource_Library/project_test_in_ubuntu/图包/半圈赛道图";
    const int FPS = 30;
    const int WAIT = 1000 / FPS;

    // 初始化
    auto img_paths = getBmpPaths(IMG_FOLDER);
    if (img_paths.empty()) {
        std::cerr << "Error: No images found in " << IMG_FOLDER << std::endl;
        return -1;
    }
    std::cout << "Found " << img_paths.size() << " images" << std::endl;
    std::cout << "Control: SPACE=pause/play | A=prev | D=next | ESC=exit" << std::endl;

    TrackImageProcessor proc; // 视觉处理核心
    Display disp;             // 显示调试核心

    int idx = 0;
    bool paused = false;
    bool running = true;

    while (running) {
        // 循环索引
        if (idx < 0) idx = img_paths.size() - 1;
        if (idx >= (int)img_paths.size()) idx = 0;

        // 1. 读取图像
        cv::Mat src = cv::imread(img_paths[idx], cv::IMREAD_GRAYSCALE);
        if (src.empty()) { idx++; continue; }

        // 2. 预处理：缩放并转数组
        cv::Mat resized;
        cv::resize(src, resized, cv::Size(UVC_WIDTH, UVC_HEIGHT));
        uint8_t gray[120][160];
        uint8_t bin[120][160];
        memcpy(gray, resized.data, 160*120);

        // 3. 核心视觉处理
        int thresh;
        proc.process(gray, bin, thresh);

        // 4. 绘制与显示
        cv::Mat main_img, bev_img;
        disp.draw(gray, bin, proc, main_img, bev_img);
        
        // 叠加状态文字
        std::string state = paused ? "PAUSED" : "PLAYING";
        cv::putText(main_img, cv::format("Frame:%d/%lu %s", idx+1, img_paths.size(), state.c_str()),
                    cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, COLOR_WHITE, 1);

        disp.show(main_img, bev_img,proc);

        // 5. 按键控制
        int key = cv::waitKey(paused ? 0 : WAIT) & 0xFF;
        switch (key) {
            case 27: running = false; break;           // ESC
            case 32: paused = !paused; break;          // Space
            case 'a': case 'A': if(paused) idx--; break;
            case 'd': case 'D': if(paused) idx++; break;
            default: if(!paused) idx++; break;
        }
    }

    disp.close();
    return 0;
}