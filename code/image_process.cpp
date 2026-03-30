#include "image_process.hpp"
#include "zf_common_headfile.hpp"
#include <cmath>
#include <algorithm>
#include <cstring>

// ====================== BorderLine 清空函数 ======================
void BorderLine::clear() {
    points.clear();
    bev_points.clear();
    filtered.clear();
    resampled.clear();
    center_line.clear();
    center_img.clear();
    l_corners.clear();
    l_corners_img.clear();
    l_corner_indices.clear();
    stopped_by_boundary = false;
    stopped_by_l_corner = false;
}


#include "cross.hpp"


// ====================== TrackImageProcessor 构造函数 ======================
TrackImageProcessor::TrackImageProcessor() 
    : DIR_VECTORS{Point2i(1,0), Point2i(0,-1), Point2i(-1,0), Point2i(0,1)} // 右、上、左、下
    , H{
        {8.99792120e+00f, 3.42620199e+01f, -8.18231892e+02f},
        {-3.15377793e-01f, 4.92066630e+01f, -9.38423519e+02f},
        {-4.38024713e-03f, 3.57061690e-01f, 1.00000000e+00f}
    }
    , L_CORNER_ANGLE_THRESH(1.1f)
    , L_CORNER_NMS_KERNEL(7)
    , TRACK_HALF_WIDTH(-15.0f)
    , REGROW_MIN_POINTS(10)
    , REGROW_LEFT_SEARCH_START(Point2i(20, 90))
    , REGROW_RIGHT_SEARCH_START(Point2i(140, 90))
{
    hightest = 0;
    error = 0;
    threshold = 128;
    // 初始化边界数组为默认值
    for (int i = 0; i < UVC_HEIGHT; i++) {
        l_border_arr[i] = BORDER_MIN;
        r_border_arr[i] = BORDER_MAX;
        center_line_arr[i] = 0;
    }
}

// ====================== 内部辅助函数实现 ======================
void TrackImageProcessor::otsu_binarization(uint8_t gray_arr[UVC_HEIGHT][UVC_WIDTH], uint8_t bin_out[UVC_HEIGHT][UVC_WIDTH], int& thresh_out) {
    int hist[256] = {0};
    // 1. 统计灰度直方图
    for (int i = 0; i < UVC_HEIGHT; i++)
        for (int j = 0; j < UVC_WIDTH; j++)
            hist[gray_arr[i][j]]++;

    float max_var = 0.0f;
    int best_th = 128;
    int total = UVC_HEIGHT * UVC_WIDTH;
    
    // 2. 遍历所有阈值，找最大类间方差
    for (int th = 0; th < 256; th++) {
        int cnt_bg = 0, cnt_fg = 0;       // 背景/前景像素计数
        long long sum_bg = 0, sum_fg = 0; // 背景/前景灰度和
        
        // 统计背景 (小于等于阈值)
        for (int i = 0; i <= th; i++) { cnt_bg += hist[i]; sum_bg += (long long)i * hist[i]; }
        // 统计前景 (大于阈值)
        cnt_fg = total - cnt_bg;
        if (cnt_bg == 0 || cnt_fg == 0) continue; // 避免全黑或全白
        
        for (int i = th+1; i < 256; i++) sum_fg += (long long)i * hist[i];
        
        // 3. 计算类间方差公式
        float mean_bg = (float)sum_bg / cnt_bg;
        float mean_fg = (float)sum_fg / cnt_fg;
        float var = (float)((long long)cnt_bg * cnt_fg) / ((float)total * total) * (mean_bg - mean_fg) * (mean_bg - mean_fg);
        
        // 4. 更新最优阈值
        if (var > max_var) { max_var = var; best_th = th; }
    }

    // 5. 应用二值化
    thresh_out = best_th;
    this->threshold = best_th;
    for (int i = 0; i < UVC_HEIGHT; i++)
        for (int j = 0; j < UVC_WIDTH; j++)
            bin_out[i][j] = (gray_arr[i][j] > best_th) ? WHITE : BLACK;
}

void TrackImageProcessor::draw_black_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]) {
    // 给左右边缘加黑框，防止追踪跑到图像外面
    for (int i = 0; i < UVC_HEIGHT; i++) {
        bin_img[i][0] = bin_img[i][1] = BLACK;
        bin_img[i][UVC_WIDTH-1] = bin_img[i][UVC_WIDTH-2] = BLACK;
    }
    // 给顶部加黑框
    for (int j = 0; j < UVC_WIDTH; j++) {
        bin_img[0][j] = bin_img[1][j] = BLACK;
    }
}

std::pair<Point2i, Point2i> TrackImageProcessor::find_start_points(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH]) {
    int start_row = UVC_HEIGHT - 10; // 从底部往上第10行开始找
    int l_x = IMG_CENTER, r_x = IMG_CENTER;
    
    // 向左找：白->黑跳变就是左边界
    for (int x = IMG_CENTER; x > BORDER_MIN; x--) {
        if (bin_img[start_row][x] == WHITE && bin_img[start_row][x-1] == BLACK) {
            l_x = x; break;
        }
    }
    
    // 向右找：白->黑跳变就是右边界
    for (int x = IMG_CENTER; x < BORDER_MAX; x++) {
        if (bin_img[start_row][x] == WHITE && bin_img[start_row][x+1] == BLACK) {
            r_x = x; break;
        }
    }
    
    return {Point2i(l_x, start_row), Point2i(r_x, start_row)};
}

bool TrackImageProcessor::check_touch_boundary(int x, int y) {
    if (y >= UVC_HEIGHT / 2) return false; // 下半部分不检查边界
    return (x <= BORDER_MIN + 1 || x >= BORDER_MAX - 1 || y <= 0); // 碰到边缘返回true
}

uint8_t TrackImageProcessor::get_pixel(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int x, int y) {
    if (0 <= y && y < UVC_HEIGHT && 0 <= x && x < UVC_WIDTH)
        return bin_img[y][x];
    return BLACK; // 越界默认返回黑色
}

std::pair<std::vector<Point2i>, bool> TrackImageProcessor::right_hand_track(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y, int limit_x, const char* limit_dir) {
    std::vector<Point2i> points;
    int x = start_x, y = start_y;
    int dir = 1, break_flag = USE_NUM, turn = 0; // dir=1初始向上
    bool stopped_by_boundary = false;

    while (break_flag > 0 && turn < 4) {
        points.emplace_back(x, y); // 记录当前点
        
        // 检查横坐标限制 (用于重生长)
        if (limit_x != -1) {
            if (strcmp(limit_dir, "left") == 0 && x < limit_x) break;
            if (strcmp(limit_dir, "right") == 0 && x > limit_x) break;
        }
        
        // 检查是否到了顶部停止行
        if (y < TRACK_STOP_ROW) break;
        // 检查是否触达图像边缘
        if (check_touch_boundary(x, y)) { stopped_by_boundary = true; break; }

        // 计算前方像素
        Point2i front = DIR_VECTORS[dir];
        int front_x = x + front.x, front_y = y + front.y;
        uint8_t front_val = get_pixel(bin_img, front_x, front_y);

        // 计算左前方像素 (右手定则：优先摸左边)
        int left_dir = (dir + 3) % 4;
        Point2i frontleft = DIR_VECTORS[left_dir];
        int fl_x = x + frontleft.x, fl_y = y + frontleft.y;
        uint8_t fl_val = get_pixel(bin_img, fl_x, fl_y);

        // 右手定则逻辑：
        // 1. 前方没路(黑)：右转
        // 2. 前方有路，左前方没路：直走
        // 3. 都有路：左转 (贴墙走)
        if (front_val == BLACK) { dir = (dir + 1) % 4; turn++; }
        else if (fl_val == BLACK) { x = front_x; y = front_y; turn = 0; }
        else { dir = left_dir; x = fl_x; y = fl_y; turn = 0; }

        break_flag--;
        
        // 防死循环：连续3个点一样就停止
        if (points.size() >= 3) {
            if (points.back().x == points[points.size()-2].x && points.back().y == points[points.size()-2].y &&
                points.back().x == points[points.size()-3].x && points.back().y == points[points.size()-3].y)
                break;
        }
    }
    return {points, stopped_by_boundary};
}

std::pair<std::vector<Point2i>, bool> TrackImageProcessor::left_hand_track(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y, int limit_x, const char* limit_dir) {
    std::vector<Point2i> points;
    int x = start_x, y = start_y;
    int dir = 1, break_flag = USE_NUM, turn = 0;
    bool stopped_by_boundary = false;

    while (break_flag > 0 && turn < 4) {
        points.emplace_back(x, y);
        
        if (limit_x != -1) {
            if (strcmp(limit_dir, "left") == 0 && x < limit_x) break;
            if (strcmp(limit_dir, "right") == 0 && x > limit_x) break;
        }
        
        if (y < TRACK_STOP_ROW) break;
        if (check_touch_boundary(x, y)) { stopped_by_boundary = true; break; }

        Point2i front = DIR_VECTORS[dir];
        int front_x = x + front.x, front_y = y + front.y;
        uint8_t front_val = get_pixel(bin_img, front_x, front_y);

        // 计算右前方像素 (左手定则：优先摸右边)
        int right_dir = (dir + 1) % 4;
        Point2i frontright = DIR_VECTORS[right_dir];
        int fr_x = x + frontright.x, fr_y = y + frontright.y;
        uint8_t fr_val = get_pixel(bin_img, fr_x, fr_y);

        // 左手定则逻辑
        if (front_val == BLACK) { dir = (dir + 3) % 4; turn++; }
        else if (fr_val == BLACK) { x = front_x; y = front_y; turn = 0; }
        else { dir = right_dir; x = fr_x; y = fr_y; turn = 0; }

        break_flag--;
        
        if (points.size() >= 3) {
            if (points.back().x == points[points.size()-2].x && points.back().y == points[points.size()-2].y &&
                points.back().x == points[points.size()-3].x && points.back().y == points[points.size()-3].y)
                break;
        }
    }
    return {points, stopped_by_boundary};
}

bool TrackImageProcessor::inverse_matrix_3x3(const float m[3][3], float inv_out[3][3]) {
    // 计算行列式
    float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    
    if (fabs(det) < 1e-6f) return false; // 奇异矩阵，不可逆
    
    // 伴随矩阵 / 行列式
    float inv_det = 1.0f / det;
    inv_out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv_out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv_out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    inv_out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv_out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv_out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    inv_out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv_out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv_out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    
    return true;
}

std::vector<Point2f> TrackImageProcessor::perspective_transform(const std::vector<Point2i>& points) {
    std::vector<Point2f> bev;
    for (const auto& p : points) {
        float x = (float)p.x, y = (float)p.y;
        // 应用单应性矩阵 H：p' = H * p
        float w = H[2][0]*x + H[2][1]*y + H[2][2]; // 齐次坐标分量
        
        if (fabs(w) < 1e-6f) continue;
        
        float bx = (H[0][0]*x + H[0][1]*y + H[0][2]) / w;
        float by = (H[1][0]*x + H[1][1]*y + H[1][2]) / w;
        bev.emplace_back(bx, by);
    }
    return bev;
}

std::vector<Point2i> TrackImageProcessor::inverse_perspective_transform(const std::vector<Point2f>& bev_points) {
    std::vector<Point2i> img;
    float H_inv[3][3];
    
    if (!inverse_matrix_3x3(H, H_inv)) return img; // 求逆失败返回空
    
    for (const auto& p : bev_points) {
        float x = p.x, y = p.y;
        // 应用逆矩阵 H_inv：p = H_inv * p'
        float w = H_inv[2][0]*x + H_inv[2][1]*y + H_inv[2][2];
        
        if (fabs(w) < 1e-6f) continue;
        
        float ix = (H_inv[0][0]*x + H_inv[0][1]*y + H_inv[0][2]) / w;
        float iy = (H_inv[1][0]*x + H_inv[1][1]*y + H_inv[1][2]) / w;
        img.emplace_back((int)round(ix), (int)round(iy));
    }
    return img;
}

std::vector<Point2f> TrackImageProcessor::blur_points(const std::vector<Point2f>& points_in, int kernel) {
    if (points_in.size() < kernel) return points_in;
    
    std::vector<Point2f> out;
    int half = kernel / 2;
    
    for (int i = 0; i < points_in.size(); i++) {
        float x_sum = 0, y_sum = 0;
        int weight_sum = 0;
        
        // 加权平均：中心权重高，边缘权重低
        for (int j = -half; j <= half; j++) {
            int idx = std::max(0, std::min(i+j, (int)points_in.size()-1));
            int weight = half + 1 - abs(j); // 三角加权
            x_sum += points_in[idx].x * weight;
            y_sum += points_in[idx].y * weight;
            weight_sum += weight;
        }
        
        out.emplace_back(x_sum/weight_sum, y_sum/weight_sum);
    }
    return out;
}

std::vector<Point2f> TrackImageProcessor::resample_points(const std::vector<Point2f>& points_in, float dist) {
    if (points_in.size() < 2) return points_in;
    
    std::vector<Point2f> out;
    out.push_back(points_in[0]); // 起点直接放入
    float remain = dist;

    // 当两点之间距离超过这个阈值时，认为是断裂，不插值连接
    const float max_gap = 5.0f;   // 可以按需要调大/调小

    for (int i = 0; i < (int)points_in.size() - 1; i++) {
        float x0 = points_in[i].x,     y0 = points_in[i].y;
        float x1 = points_in[i+1].x,   y1 = points_in[i+1].y;
        float dx = x1 - x0,            dy = y1 - y0;
        float seg_len = std::sqrt(dx*dx + dy*dy); // 两点距离
        
        if (seg_len < 1e-6f) continue;

        // 如果两点间距离太大，认为这里是“断裂”，
        // 不要在这段之间插值连接，而是从下一个点重新开始新段。
        if (seg_len > max_gap) {
            // 把下一点作为新的起点段，重新 push，一般可以直接 out.push_back(x1,y1)
            out.push_back(points_in[i+1]);
            remain = dist;
            continue;
        }
        
        // 否则在该段内按固定步长 dist 插值
        float dir_x = dx / seg_len;
        float dir_y = dy / seg_len;

        float d = remain;
        while (d < seg_len && (int)out.size() < USE_NUM) {
            float nx = x0 + dir_x * d;
            float ny = y0 + dir_y * d;
            out.emplace_back(nx, ny);
            d += dist;
        }
        remain = d - seg_len;    // 把溢出部分留给下一段
    }
    return out;
}

std::vector<float> TrackImageProcessor::local_angle_points(const std::vector<Point2f>& points, int dist) {
    std::vector<float> angles(points.size(), 0.0f);
    
    for (int i = 0; i < points.size(); i++) {
        if (i <= 0 || i >= points.size()-1) continue; // 首尾不算
        
        // 取前后 dist 个点计算向量
        int idx1 = std::max(0, std::min(i-dist, (int)points.size()-1));
        float dx1 = points[i].x - points[idx1].x;
        float dy1 = points[i].y - points[idx1].y;
        float dn1 = sqrt(dx1*dx1 + dy1*dy1);
        if (dn1 < 1e-6f) continue;
        
        int idx2 = std::max(0, std::min(i+dist, (int)points.size()-1));
        float dx2 = points[idx2].x - points[i].x;
        float dy2 = points[idx2].y - points[i].y;
        float dn2 = sqrt(dx2*dx2 + dy2*dy2);
        if (dn2 < 1e-6f) continue;
        
        // 计算两个向量的夹角 (叉积/点积)
        float c1 = dx1/dn1, s1 = dy1/dn1;
        float c2 = dx2/dn2, s2 = dy2/dn2;
        angles[i] = atan2(c1*s2 - c2*s1, c2*c1 + s2*s1);
    }
    return angles;
}

std::vector<float> TrackImageProcessor::nms_angle(const std::vector<float>& angles, int kernel) {
    if (angles.size() < kernel) return angles;
    
    std::vector<float> out = angles;
    int half = kernel / 2;
    
    for (int i = 0; i < angles.size(); i++) {
        bool is_max = true;
        
        // 非极大值抑制：如果我不是邻域内最大的，就置0
        for (int j = -half; j <= half; j++) {
            int idx = std::max(0, std::min(i+j, (int)angles.size()-1));
            if (fabs(angles[idx]) > fabs(angles[i])) { is_max = false; break; }
        }
        
        if (!is_max) out[i] = 0.0f;
    }
    return out;
}

std::pair<std::vector<int>, std::vector<float>> TrackImageProcessor::find_l_corners(const std::vector<Point2f>& points, float angle_thresh, int nms_kernel, int dist) {
    std::vector<int> corners;
    std::vector<float> angles_nms;
    
    if (points.size() < 10) return {corners, angles_nms};
    
    // 只在前30个点里找角点 (防止找到赛道深处)
    int start_idx = 3;
    int end_idx = std::min(30, (int)points.size()-1);
    
    if (start_idx >= end_idx) return {corners, angles_nms};
    
    auto angles = local_angle_points(points, dist);
    angles_nms = nms_angle(angles, nms_kernel);
    
    // 角度超过阈值即为角点
    for (int i = start_idx; i <= end_idx; i++) {
        if (fabs(angles_nms[i]) > angle_thresh) corners.push_back(i);
    }
    
    return {corners, angles_nms};
}

std::vector<Point2f> TrackImageProcessor::track_line_from_border(const std::vector<Point2f>& border_points, float dist, bool is_left) {
    std::vector<Point2f> center;
    if (border_points.size() < 2) return center;
    
    for (int i = 0; i < border_points.size(); i++) {
        float dx, dy;
        
        // 计算切线方向
        if (i == 0) { 
            dx = border_points[1].x - border_points[0].x; 
            dy = border_points[1].y - border_points[0].y; 
        }
        else if (i == border_points.size()-1) { 
            dx = border_points[i].x - border_points[i-1].x; 
            dy = border_points[i].y - border_points[i-1].y; 
        }
        else { 
            dx = border_points[i+1].x - border_points[i-1].x; 
            dy = border_points[i+1].y - border_points[i-1].y; 
        }
        
        float dn = sqrt(dx*dx + dy*dy);
        if (dn < 1e-6f) { 
            if (!center.empty()) center.push_back(center.back()); 
            continue; 
        }
        
        // 切线 -> 法线 (旋转90度)
        float tx = dx/dn, ty = dy/dn;
        float nx, ny;
        if (is_left) { nx = ty; ny = -tx; }  // 左边界：向右偏移
        else { nx = -ty; ny = tx; }           // 右边界：向左偏移
        
        // 沿法线方向偏移 dist 距离
        float cx = border_points[i].x + nx * dist;
        float cy = border_points[i].y + ny * dist;
        center.emplace_back(cx, cy);
    }
    return center;
}

void TrackImageProcessor::truncate_at_l_corners(BorderLine& border) {
    if (border.l_corner_indices.empty()) return;
    
    // 找到第一个角点
    int first_corner = *std::min_element(border.l_corner_indices.begin(), border.l_corner_indices.end());
    
    // 在角点处截断 (保留角点之前的部分)
    if (first_corner < border.resampled.size()) border.resampled.resize(first_corner);
    if (first_corner < border.center_line.size()) border.center_line.resize(first_corner);
    
    border.stopped_by_l_corner = true;
}

bool TrackImageProcessor::is_edge_growth_point(const Point2i& img_point, int margin) {
    int x = img_point.x, y = img_point.y;
    // 判断点是否在图像边缘附近
    return (x <= BORDER_MIN + margin || x >= BORDER_MAX - margin || y <= margin || y >= UVC_HEIGHT - 1 - margin);
}

std::vector<Point2f> TrackImageProcessor::track_line_from_border_unequal(const std::vector<Point2f>& border_points, float dist, bool is_left, const std::vector<bool>& edge_flags) {
    std::vector<Point2f> center;
    if (border_points.size() < 2) return center;
    
    for (int i = 0; i < border_points.size(); i++) {
        // 如果是边缘点，直接跳过，不生成中心线
        if (i < edge_flags.size() && edge_flags[i]) continue;
        
        float dx, dy;
        
        if (i == 0) { 
            dx = border_points[1].x - border_points[0].x; 
            dy = border_points[1].y - border_points[0].y; 
        }
        else if (i == border_points.size()-1) { 
            dx = border_points[i].x - border_points[i-1].x; 
            dy = border_points[i].y - border_points[i-1].y; 
        }
        else { 
            dx = border_points[i+1].x - border_points[i-1].x; 
            dy = border_points[i+1].y - border_points[i-1].y; 
        }
        
        float dn = sqrt(dx*dx + dy*dy);
        if (dn < 1e-6f) { 
            if (!center.empty()) center.push_back(center.back()); 
            continue; 
        }
        
        float tx = dx/dn, ty = dy/dn;
        float nx, ny;
        if (is_left) { nx = ty; ny = -tx; }
        else { nx = -ty; ny = tx; }
        
        float cx = border_points[i].x + nx * dist;
        float cy = border_points[i].y + ny * dist;
        center.emplace_back(cx, cy);
    }
    return center;
}

Point2i* TrackImageProcessor::find_edge_point_upward(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], int start_x, int start_y) {
    static Point2i result;
    int x = start_x;
    
    // 从下往上找，找白到黑的跳变 (赛道上边缘)
    for (int y = start_y; y > TRACK_STOP_ROW; y--) {
        if (0 <= y && y < UVC_HEIGHT && 0 <= x && x < UVC_WIDTH) {
            uint8_t curr = get_pixel(bin_img, x, y);
            uint8_t above = (y > 0) ? get_pixel(bin_img, x, y-1) : BLACK;
            
            if (curr == WHITE && above == BLACK) {
                result = Point2i(x, y);
                return &result;
            }
        }
    }
    return nullptr;
}

BorderLine TrackImageProcessor::regrow_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], const Point2i& search_start, bool is_left) {
    BorderLine border;
    
    // 1. 先找到一个边缘点
    Point2i* edge_point = find_edge_point_upward(bin_img, search_start.x, search_start.y);
    if (!edge_point) return border;
    
    int start_x = edge_point->x, start_y = edge_point->y;

    // 2. 从该点开始追踪
    if (is_left) {
        auto res = left_hand_track(bin_img, start_x, start_y, start_x, "left");
        border.points = res.first;
        border.stopped_by_boundary = res.second;
    } else {
        auto res = right_hand_track(bin_img, start_x, start_y, start_x, "right");
        border.points = res.first;
        border.stopped_by_boundary = res.second;
    }
    
    if (border.points.empty()) return border;

    // 2.5 剔除贴边假边界点
    {
        std::vector<Point2i> cleaned;
        cleaned.reserve(border.points.size());
        for (const auto& p : border.points) {
            if (!is_edge_growth_point(p, /*margin=*/2)) {
                cleaned.push_back(p);
            }
        }
        border.points.swap(cleaned);
    }

    if (border.points.empty()) return border;

    // 3. 后续处理流程和 process_border 一样
    border.bev_points = perspective_transform(border.points);
    border.filtered = blur_points(border.bev_points, 5);
    border.resampled = resample_points(border.filtered, 2.5f);
    
    if (border.resampled.empty()) return border;

    auto corner_res = find_l_corners(border.resampled, L_CORNER_ANGLE_THRESH, L_CORNER_NMS_KERNEL, 5);
    border.l_corner_indices = corner_res.first;
    truncate_at_l_corners_reverse(border); // 反向截断：保留角点之后的部分
    
    if (border.resampled.empty()) return border;


    if (!border.l_corner_indices.empty()) {
        auto all_bev = resample_points(border.filtered, 2.5f);
        std::vector<Point2f> corners_bev;
        for (int i : border.l_corner_indices) if (i < all_bev.size()) corners_bev.push_back(all_bev[i]);
        border.l_corners = corners_bev;
        border.l_corners_img = inverse_perspective_transform(corners_bev);
    }
    
    return border;
}

std::vector<Point2f> TrackImageProcessor::merge_center_lines(const std::vector<Point2f>& original, const std::vector<Point2f>& regrown) {
    std::vector<Point2f> merged = original;
    // 把重生长的线拼在原来的线后面
    merged.insert(merged.end(), regrown.begin(), regrown.end());
    return merged;
}

void TrackImageProcessor::truncate_at_l_corners_reverse(BorderLine& border) {
    if (border.l_corner_indices.empty()) return;
    
    int first_corner = *std::min_element(border.l_corner_indices.begin(), border.l_corner_indices.end());
    
    // 反向截断：删掉角点之前的部分，保留之后的部分
    if (first_corner < border.resampled.size()) {
        border.resampled.erase(border.resampled.begin(), border.resampled.begin() + first_corner + 1);
    }
    if (first_corner < border.center_line.size()) {
        border.center_line.erase(border.center_line.begin(), border.center_line.begin() + first_corner + 1);
    }
}

BorderLine TrackImageProcessor::process_border(uint8_t bin_img[UVC_HEIGHT][UVC_WIDTH], const Point2i& start_point, bool is_left) {
    BorderLine border;
    
    // 1. 边界追踪
    if (is_left) {
        auto res = left_hand_track(bin_img, start_point.x, start_point.y);
        border.points = res.first;
        border.stopped_by_boundary = res.second;
    } else {
        auto res = right_hand_track(bin_img, start_point.x, start_point.y);
        border.points = res.first;
        border.stopped_by_boundary = res.second;
    }
    
    if (border.points.empty()) return border;

    // 1.5 剔除贴着图像边界的假边界点
    {
        std::vector<Point2i> cleaned;
        cleaned.reserve(border.points.size());
        for (const auto& p : border.points) {
            // margin 可以调大/调小：控制离边缘多少像素内认为是“假边界”
            if (!is_edge_growth_point(p, /*margin=*/2)) {
                cleaned.push_back(p);
            }
        }
        border.points.swap(cleaned);
    }

    if (border.points.empty()) return border;   // 如果全被判为假边界，就直接返回空

    // 2. 逆透视变换
    border.bev_points = perspective_transform(border.points);
    // 3. 滤波平滑
    border.filtered = blur_points(border.bev_points, 5);
    // 4. 等距重采样
    border.resampled = resample_points(border.filtered, 2.5f);
    
    if (border.resampled.empty()) return border;

    // 5. 检测L角点
    auto corner_res = find_l_corners(border.resampled, L_CORNER_ANGLE_THRESH, L_CORNER_NMS_KERNEL, 5);
    border.l_corner_indices = corner_res.first;
    // 6. 在角点处截断
    truncate_at_l_corners(border);
    
    if (border.resampled.empty()) return border;


    // 10. 角点转回图像坐标 (用于画图)
    if (!border.l_corner_indices.empty()) {
        auto all_bev = resample_points(border.filtered, 2.5f);
        std::vector<Point2f> corners_bev;
        for (int i : border.l_corner_indices) if (i < all_bev.size()) corners_bev.push_back(all_bev[i]);
        border.l_corners = corners_bev;
        border.l_corners_img = inverse_perspective_transform(corners_bev);
    }
    
    return border;
}

void TrackImageProcessor::calculate_hightest() {
    auto& left = left_border.points;
    auto& right = right_border.points;
    
    if (!left.empty() && !right.empty()) {
        int min_y = UVC_HEIGHT;
        // 找左右边界交汇点最低的那个y
        for (auto& lp : left) {
            for (auto& rp : right) {
                if (abs(lp.x - rp.x) <= 5 && abs(lp.y - rp.y) <= 5) {
                    if (std::max(lp.y, rp.y) < min_y) min_y = std::max(lp.y, rp.y);
                }
            }
        }
        hightest = (min_y < UVC_HEIGHT) ? min_y : std::min(left.back().y, right.back().y);
    } else if (!left.empty()) {
        hightest = left.back().y;
    } else if (!right.empty()) {
        hightest = right.back().y;
    } else {
        hightest = UVC_HEIGHT - 1;
    }
}

void TrackImageProcessor::extract_borders_to_array() {
    // 重置数组
    for (int i = 0; i < UVC_HEIGHT; i++) {
        l_border_arr[i] = BORDER_MIN;
        r_border_arr[i] = BORDER_MAX;
        center_line_arr[i] = 0;
    }
    
    // 把vector点填入数组 (方便主循环快速访问)
    for (auto& p : left_border.points) {
        if (TRACK_STOP_ROW <= p.y && p.y < UVC_HEIGHT) l_border_arr[p.y] = p.x + 1;
    }
    
    for (auto& p : right_border.points) {
        if (TRACK_STOP_ROW <= p.y && p.y < UVC_HEIGHT) r_border_arr[p.y] = p.x - 1;
    }
}

void TrackImageProcessor::calculate_error() {
    long long sum_error = 0;
    int count = 0;
    
    // 计算从停止行到底部的平均偏差
    for (int y = TRACK_STOP_ROW; y < UVC_HEIGHT; y++) {
        if (l_border_arr[y] != BORDER_MIN && r_border_arr[y] != BORDER_MAX) {
            center_line_arr[y] = (l_border_arr[y] + r_border_arr[y]) / 2;
            sum_error += IMG_CENTER - center_line_arr[y]; // 图像中心 - 赛道中心
            count++;
        }
    }
    
    error = (count > 0) ? (int)(sum_error / count) : 0;
}

// ====================== 核心处理函数 ======================
bool TrackImageProcessor::process(uint8_t gray_img[UVC_HEIGHT][UVC_WIDTH], uint8_t bin_out[UVC_HEIGHT][UVC_WIDTH], int& threshold_out) {
    // 0. 清空上一帧数据
    left_border.clear();
    right_border.clear();

    // 1. 二值化
    otsu_binarization(gray_img, bin_out, threshold_out);
    // 2. 画黑框防干扰
    draw_black_border(bin_out);

    // 3. 找起点
    auto starts = find_start_points(bin_out);
    // 4. 处理左右边界
    left_border = process_border(bin_out, starts.first, true);
    right_border = process_border(bin_out, starts.second, false);

    check_cross(*this);
    run_cross(*this, bin_out);

    // 5. 判断是否需要重生长
    int left_cnt = left_border.center_line.size();
    int right_cnt = right_border.center_line.size();
    // 条件1：两边线都太短
    bool cond1 = (left_cnt < REGROW_MIN_POINTS && right_cnt < REGROW_MIN_POINTS);
    // 条件2：两边都因为角点或边界停止了 (说明可能只看到了一个拐角)
    bool cond2 = ((left_border.stopped_by_l_corner && right_border.stopped_by_l_corner) ||
                  (left_border.stopped_by_boundary && right_border.stopped_by_boundary));
    bool regrow = cond1 && cond2;

    if (regrow) {
        // 6. 执行重生长
        auto regrown_left = regrow_border(bin_out, REGROW_LEFT_SEARCH_START, true);
        auto regrown_right = regrow_border(bin_out, REGROW_RIGHT_SEARCH_START, false);
        

        // 9. 合并角点数据
        left_border.l_corners.insert(left_border.l_corners.end(), regrown_left.l_corners.begin(), regrown_left.l_corners.end());
        left_border.l_corners_img.insert(left_border.l_corners_img.end(), regrown_left.l_corners_img.begin(), regrown_left.l_corners_img.end());
        right_border.l_corners.insert(right_border.l_corners.end(), regrown_right.l_corners.begin(), regrown_right.l_corners.end());
        right_border.l_corners_img.insert(right_border.l_corners_img.end(), regrown_right.l_corners_img.begin(), regrown_right.l_corners_img.end());
    }

    // 10. 计算最终输出数据
    calculate_hightest();
    extract_borders_to_array();
    calculate_error();
    
    process_return_flag = regrow;  // 新增
    frame_count++;                // 新增
    return regrow;
}