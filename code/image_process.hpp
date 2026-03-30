#ifndef IMAGE_PROCESS_HPP
#define IMAGE_PROCESS_HPP
#include <vector>
#include <cstdint>
#include <utility>
#include "zf_common_headfile.hpp"
// ====================== 数据结构 ======================
struct Point2i {
    int x;
    int y;
    Point2i() : x(0), y(0) {}
    Point2i(int x, int y) : x(x), y(y) {}
};

struct Point2f {
    float x;
    float y;
    Point2f() : x(0), y(0) {}
    Point2f(float x, float y) : x(x), y(y) {}
    Point2f(const Point2i& p) : x(p.x), y(p.y) {}
};

struct BorderLine {
    std::vector<Point2i> points;
    std::vector<Point2f> bev_points;
    std::vector<Point2f> filtered;
    std::vector<Point2f> resampled;
    std::vector<Point2f> center_line;
    std::vector<Point2i> center_img;
    std::vector<Point2f> l_corners;
    std::vector<Point2i> l_corners_img;
    std::vector<int> l_corner_indices;
    bool stopped_by_boundary;
    bool stopped_by_l_corner;
    void clear();
};


// ====================== 核心处理类 ======================
class TrackImageProcessor {
public:
    static const int USE_NUM = 360;
    static const int BORDER_MIN = 1;
    static const int BORDER_MAX = 158;
    static const int IMG_CENTER = 80;
    static const uint8_t WHITE = 255;
    static const uint8_t BLACK = 0;
    static const int TRACK_STOP_ROW = 20;


    BorderLine left_border;
    BorderLine right_border;
    int l_border_arr[UVC_HEIGHT];
    int r_border_arr[UVC_HEIGHT];
    int center_line_arr[UVC_HEIGHT];
    int hightest;
    int error;
    int threshold;

    bool process_return_flag = false;  // 存储process返回的regrow值
    int frame_count = 0;             // 帧计数

    TrackImageProcessor();
    bool process(uint8_t gray_img[UVC_HEIGHT][UVC_WIDTH], uint8_t bin_ouT[UVC_HEIGHT][UVC_WIDTH], int& threshold_out);
    std::vector<Point2f> perspective_transform(const std::vector<Point2i>& points);
    std::vector<Point2i> inverse_perspective_transform(const std::vector<Point2f>& bev_points);

    const Point2i DIR_VECTORS[4];
    const float H[3][3];
    const float L_CORNER_ANGLE_THRESH;
    const int L_CORNER_NMS_KERNEL;
    const float TRACK_HALF_WIDTH;
    const int REGROW_MIN_POINTS;
    const Point2i REGROW_LEFT_SEARCH_START;
    const Point2i REGROW_RIGHT_SEARCH_START;

    void otsu_binarization(uint8_t gray_arr[UVC_HEIGHT][UVC_WIDTH], uint8_t bin_ouT[UVC_HEIGHT][UVC_WIDTH], int& thresh_out);
    void draw_black_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]);
    std::pair<Point2i, Point2i> find_start_points(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]);
    bool check_touch_boundary(int x, int y);
    uint8_t get_pixel(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int x, int y);
    std::pair<std::vector<Point2i>, bool> right_hand_track(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y, int limit_x = -1, const char* limit_dir = "none");
    std::pair<std::vector<Point2i>, bool> left_hand_track(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y, int limit_x = -1, const char* limit_dir = "none");
    bool inverse_matrix_3x3(const float m[3][3], float inv_out[3][3]);
    std::vector<Point2f> blur_points(const std::vector<Point2f>& points_in, int kernel = 5);
    std::vector<Point2f> resample_points(const std::vector<Point2f>& points_in, float dist = 10.0f);
    std::vector<float> local_angle_points(const std::vector<Point2f>& points, int dist = 5);
    std::vector<float> nms_angle(const std::vector<float>& angles, int kernel = 5);
    std::pair<std::vector<int>, std::vector<float>> find_l_corners(const std::vector<Point2f>& points, float angle_thresh = 0.3f, int nms_kernel = 5, int dist = 5);
    std::vector<Point2f> track_line_from_border(const std::vector<Point2f>& border_points, float dist, bool is_left);
    std::vector<Point2i> compute_center_from_both_sides();
    void truncate_at_l_corners(BorderLine& border);
    bool is_edge_growth_point(const Point2i& img_point, int margin = 5);
    std::vector<Point2f> track_line_from_border_unequal(const std::vector<Point2f>& border_points, float dist, bool is_left, const std::vector<bool>& edge_flags);
    Point2i* find_edge_point_upward(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y);
    BorderLine regrow_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], const Point2i& search_start, bool is_left);
    std::vector<Point2f> merge_center_lines(const std::vector<Point2f>& original, const std::vector<Point2f>& regrown);
    void truncate_at_l_corners_reverse(BorderLine& border);
    BorderLine process_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], const Point2i& start_point, bool is_left);
    void calculate_hightest();
    void extract_borders_to_array();
    void calculate_error();
};

#endif