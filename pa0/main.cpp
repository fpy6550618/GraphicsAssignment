#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;
Matrix3f rodriguesRotation(const Vector3f &axis, float theta)
{
    // 确保旋转轴是单位向量
    Vector3f v = axis.normalized();
    // 计算 sin(theta) 和 cos(theta)
    float sin_theta = std::sin(theta);
    float cos_theta = std::cos(theta);
    // 计算反对称矩阵 K
    Matrix3f K;
    K << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    // 计算 K^2
    Matrix3f K_sq = K * K;
    // Rodrigues 公式计算旋转矩阵
    Matrix3f R = Matrix3f::Identity() + (1 - cos_theta) * K_sq + sin_theta * K;
    return R;
}
Matrix3f rodriguesRotation2(const Vector3f &axis, float theta)
{
    // 确保旋转轴是单位向量
    Vector3f v = axis.normalized();
    // 计算 sin(theta) 和 cos(theta)
    float sin_theta = std::sin(theta);
    float cos_theta = std::cos(theta);
    // 计算反对称矩阵 K
    Matrix3f K;
    K << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;

    K(0,1) = -v.z();
    K(0,2) = -v.y();
    K(1,0) = v.z();
    K(1,2) = -v.x();
    K(2,0) = -v.y();
    K(2,1) = v.x();
    Matrix3f R = cos_theta * Matrix3f::Identity() + (1 - cos_theta) * v * v.transpose() + sin_theta * K;
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (std::abs(R(i,j)) < 1e-6f) R(i,j) = 0.0f;
        }
    }
    return R;
}

int main()
{

    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a / b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0 / 180.0 * acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f, 2.0f, 3.0f);
    Eigen::Vector3f w(1.0f, 0.0f, 0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i, j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    // matrix scalar multiply i * 2.0
    // matrix multiply i * j
    // matrix multiply vector i * v
    std::cout << "Example of add \n";
    std::cout << i + j << std::endl;
    std::cout << "Example of sub \n";
    std::cout << i - j << std::endl;
    std::cout << "Example of multiply \n";
    std::cout << i * j << std::endl;

    // float angle{1.5f};
    // float tx{2} ,ty{1} ,tz{3};
    // AngleAxisf rotation(angle, Vector3f::UnitZ()); // 旋转
    // Translation<float, 2> translation(tx, ty);     // 平移
    // Affine2f transform = translation * rotation.toRotationMatrix();   // 组合变换
    // Vector2f p(1,2);
    // Vector2f p_transformed = transform * p;

    // Quaternionf q(AngleAxisf(angle, Vector3f::UnitZ())); // 四元数旋转
    // Transform<float, 3, Affine> T = q * Translation3f(tx, ty, tz);
    // Vector3f p_transformed = T * p;

    //Assignment
    Vector3f P(2.0f, 1.0f, 1.0f);
    float theta = M_PI / 4.0f;
    Matrix3f R;
    R << cos(theta), -sin(theta), 0,
        sin(theta), cos(theta), 0,
        0, 0, 1;
    Matrix3f T;
    T << 1, 0, 1,
        0, 1, 2,
        0, 0, 1;
    Matrix3f M = R * T;
    Vector3f P_trans = M * P;
    std::cout << "Transformed Point:(" << P_trans(0) << "," << P_trans(1) << ")" << std::endl;

    //Perspective Projection
    float n{-1.0f}, f{-3.0f};
    Matrix4f MP;
    MP << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    Vector4f PF(2.0f, 2.0f, -2.0f, 1.f);
    Vector4f PFP = MP * PF;
    std::cout << "PFP Point:(" << PFP(0) << "," << PFP(1) << "," << PFP(2) << ")" << std::endl;

    //Rodrigues' Rotation Formula
    // 定义旋转轴（不需要单位化，函数内部会处理）
    Vector3f axis(1.0f, 0.0f, 0.0f); // 绕 x 轴旋转

    // 旋转角度（弧度）    
    theta = M_PI / 2.0f; // 90 度
    
    std::cout << "Example of Rodrigues \n";
    std::cout << rodriguesRotation(axis, theta)  << "\n\n";
    std::cout << rodriguesRotation2(axis, theta)  << "\n\n";
    return 0;
}