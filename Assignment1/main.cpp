#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Vector3f v(0.0f, 0.0f, 1.0f);
    // 计算 sin(theta) 和 cos(theta)
    float sin_theta = std::sin(rotation_angle * MY_PI / 180.f);
    float cos_theta = std::cos(rotation_angle * MY_PI / 180.f);
    // 计算反对称矩阵 K
    Matrix3f K;
    K << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;

    // Matrix3f R = cos_theta * Matrix3f::Identity() + (1 - cos_theta) * v * v.transpose() + sin_theta * K;

    Matrix3f R = Matrix3f::Identity() + (1 - cos_theta) * K * K + sin_theta * K;
    std::cout << R << std::endl;
    model.block<3, 3>(0, 0) = R;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    //
    float tan_halffov = std::tan(eye_fov * MY_PI / 360.f);
    // projection(0, 0) = 1 / (aspect_ratio * tan_halffov);
    // projection(1, 1) = 1 / tan_halffov;
    // projection(2, 2) = (zNear + zFar) / (zNear - zFar);
    // projection(2, 3) = -2 * zNear * zFar / (zNear - zFar);
    // projection(3, 2) = -1;
    // projection = projection*-1;

    // //vedio Infer
    Eigen::Matrix4f OR;
    OR << 1 / (zNear * tan_halffov * aspect_ratio), 0, 0, 0,
        0, 1 / (zNear * tan_halffov), 0, 0,
        0, 0, -2 / (zNear - zFar), -(zNear + zFar) / (zNear - zFar),
        0, 0, 0, 1;
    Eigen::Matrix4f P2O;
    P2O << -zNear, 0, 0, 0,
        0, -zNear, 0, 0,
        0, 0, -(zNear + zFar), -zNear * zFar,
        0, 0, 1, 0;
    projection = OR * P2O;

    // float tan_half_fov = tan(eye_fov * MY_PI / 360.0f); // eye_fov 的一半，转为弧度
    // float top = zNear * tan_half_fov;
    // float bottom = -top;
    // float right = top * aspect_ratio;
    // float left = -right;

    // //Games101
    // // 正交投影矩阵：缩放和平移
    // Eigen::Matrix4f ortho = Eigen::Matrix4f::Identity();
    // ortho(0, 0) = 2.0f / (right - left);
    // ortho(1, 1) = 2.0f / (top - bottom);
    // ortho(2, 2) = 2.0f / (zNear - zFar);
    // ortho(0, 3) = -(right + left) / (right - left);
    // ortho(1, 3) = -(top + bottom) / (top - bottom);
    // ortho(2, 3) = -(zNear + zFar) / (zNear - zFar);

    // // 透视投影矩阵 -> 正交投影矩阵的变换
    // Eigen::Matrix4f persp_to_ortho = Eigen::Matrix4f::Zero();
    // persp_to_ortho(0, 0) = -zNear;
    // persp_to_ortho(1, 1) = -zNear;
    // persp_to_ortho(2, 2) = -zNear - zFar;
    // persp_to_ortho(2, 3) = -zNear * zFar;
    // persp_to_ortho(3, 2) = 1.0f;

    // // 完整的透视投影矩阵 = 正交投影矩阵 * 透视->正交变换矩阵
    // projection = ortho * persp_to_ortho;

    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {0, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        // std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
