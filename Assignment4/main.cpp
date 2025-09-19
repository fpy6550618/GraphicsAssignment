#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // 递归基线条件：当控制点序列只剩一个点时，返回该点
    if (control_points.size() == 1)
        return control_points[0];

    // 创建新的控制点序列，用于存储本轮递归生成的中间点
    std::vector<cv::Point2f> next_control_points;
    
    // 遍历当前控制点序列，按 t:(1-t) 的比例生成新的中间点
    for (int i = 0; i < control_points.size() - 1; i++) {
        auto& a = control_points[i];   // 当前线段的起点
        auto& b = control_points[i + 1]; // 当前线段的终点
        // 按 t:(1-t) 的比例计算分割点：p = (1-t)*a + t*b
        auto p = (1 - t) * a + t * b;
        next_control_points.push_back(p);
    }
    
    // 递归调用，继续处理新生成的控制点序列
    return recursive_bezier(next_control_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    // 对 t 从 0 到 1 进行迭代，步长设为 0.001
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        // 调用 recursive_bezier 函数计算 t 对应的曲线上的点
        cv::Point2f point = recursive_bezier(control_points, t);
        
        // 在 OpenCV 的 Mat 对象上绘制该点（绿色通道设为255，即显示为绿色）
        // 注意：OpenCV 中 Vec3b 的通道顺序为 [B, G, R]
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; // 设置 G 通道为 255
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
