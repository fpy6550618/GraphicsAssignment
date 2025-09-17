// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f p(x,y);

    Vector2f v01 = _v[0].head<2>() - _v[1].head<2>();
    Vector2f v12 = _v[1].head<2>() - _v[2].head<2>();
    Vector2f v20 = _v[2].head<2>() - _v[0].head<2>();

    Vector2f vp0 = p - _v[0].head<2>();
    Vector2f vp1 = p - _v[1].head<2>();
    Vector2f vp2 = p - _v[2].head<2>();

    float cross0p = v01.x() * vp0.y() - v01.y() * vp0.x();
    float cross1p = v12.x() * vp1.y() - v12.y() * vp1.x();
    float cross2p = v20.x() * vp2.y() - v20.y() * vp2.x();

    return cross0p > 0 && cross1p > 0 && cross2p > 0
            ||cross0p < 0 && cross1p < 0 && cross2p< 0;
}

static float ComputeTriangleArea2D(const Vector3f* v)
{
    float Ax = v[0].x();
    float Ay = v[0].y();
    float Bx = v[1].x();
    float By = v[1].y();
    float Cx = v[2].x();
    float Cy = v[2].y();
    return (Ax * (By - Cy) + Bx * (Cy - Ay) + Cx * (Ay - By)) / 2;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float S = ComputeTriangleArea2D(v);

    Vector3f v1[3] = {{x , y , 0.f}, v[1], v[2]};
    float c1 = ComputeTriangleArea2D(v1) / S;
    Vector3f v2[3] = {v[0], {x , y , 0.f}, v[2]};
    float c2 = ComputeTriangleArea2D(v2) / S;
    Vector3f v3[3] = {v[0], v[1], {x , y , 0.f}};
    float c3 = ComputeTriangleArea2D(v3) / S;

    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float MinX = std::min(v[0].x(),std::min(v[1].x(),v[2].x()));
    float MaxX = std::max(v[0].x(),std::max(v[1].x(),v[2].x()));
    float MinY = std::min(v[0].y(),std::min(v[1].y(),v[2].y()));
    float MaxY = std::max(v[0].y(),std::max(v[1].y(),v[2].y()));

    MinX = std::floor(MinX);
    MaxX = std::ceil(MaxX);
    MinY = std::floor(MinY);
    MaxY = std::ceil(MaxY);
    
    // std::vector<Vector2f> SampleOffsets =
    // {
    //     {0.5f,0.5f}
    // };
    std::vector<Vector2f> SampleOffsets =
    {
        {0.25f,0.25f},{0.25f,0.75f},{0.75f,0.25f},{0.75f,0.75f}
    };
    // std::vector<Vector2f> SampleOffsets =
    // {
    //     {0.125f,0.125f},{0.125f,0.375f},{0.125f,0.625f},{0.125f,0.875f},
    //     {0.375f,0.125f},{0.375f,0.375f},{0.375f,0.625f},{0.375f,0.875f},
    //     {0.625f,0.125f},{0.625f,0.375f},{0.625f,0.625f},{0.625f,0.875f},
    //     {0.875f,0.125f},{0.875f,0.375f},{0.875f,0.625f},{0.875f,0.875f},
    // };
    for(int x = MinX; x <= MaxX; ++x)
    {
        for(int y = MinY; y <= MaxY; ++y)
        {

            float coverage = 0.f;
            for(const auto& offset : SampleOffsets)
            {
                if(insideTriangle(x + offset.x(),y + offset.y(), t.v))
                {
                    coverage += 1.0f / SampleOffsets.size();
                }
            }
            if(coverage > 0.f)
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if(z_interpolated < depth_buf[get_index(x, y)])
                {
                    depth_buf[get_index(x, y)] = z_interpolated;

                    Vector3f point;
                    point << x, y ,z_interpolated;
                    
                    // set_pixel(point, t.getColor());
                    set_pixel(point, t.getColor() * coverage);
                    if(coverage < 1.f)
                    {
                        std::cout<< "index: " << get_index(x, y)  << ",ratio: " << coverage << '\n';
                    }
                }

            }
        }
    }

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on