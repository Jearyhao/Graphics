// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>
#define SSAA false

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
    // 向量 p0p1, p1p2, p2p0 分别表示三角形的边
    Eigen::Vector3f p0p1(_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y(), 0.0f);
    Eigen::Vector3f p1p2(_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y(), 0.0f);
    Eigen::Vector3f p2p0(_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y(), 0.0f);

    // 向量 p0p, p1p, p2p 分别表示从三角形顶点到点 (x, y) 的向量
    Eigen::Vector3f p0p(x - _v[0].x(), y - _v[0].y(), 0.0f);
    Eigen::Vector3f p1p(x - _v[1].x(), y - _v[1].y(), 0.0f);
    Eigen::Vector3f p2p(x - _v[2].x(), y - _v[2].y(), 0.0f);

    // 计算叉积的 z 分量
    float z0 = p0p1.cross(p0p).z();
    float z1 = p1p2.cross(p1p).z();
    float z2 = p2p0.cross(p2p).z();

    // 如果所有叉积的 z 分量同号，则点在三角形内
    return (z0 > 0 && z1 > 0 && z2 > 0) || (z0 < 0 && z1 < 0 && z2 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
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
    if (SSAA) {
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                Eigen::Vector3f color(0, 0, 0);
                for (int i = 0; i < 4; i++) {
                    color += frame_buf_2xSSAA[get_index(x, y) + i];
                }
                color /= 4;
                set_pixel(Eigen::Vector3f(x, y, 1.0f), color);
            }
        }
    }
}

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // 步骤1:创建三角形的边界框
    float min_x = std::min({v[0].x(), v[1].x(), v[2].x()});
    float max_x = std::max({v[0].x(), v[1].x(), v[2].x()});
    float min_y = std::min({v[0].y(), v[1].y(), v[2].y()});
    float max_y = std::max({v[0].y(), v[1].y(), v[2].y()});

    // 步骤2:迭代边界框
    for (int x = std::floor(min_x); x <= std::ceil(max_x); ++x) {
        for (int y = std::floor(min_y); y <= std::ceil(max_y); ++y) {
            if (SSAA) {
                int index = 0;
                // 2xSSAA,采用一维数组存储，每个像素分成四个小像素
                for (float i = 0.25; i < 1.0; i += 0.5) {
                    for (float j = 0.25; j < 1.0; j += 0.5) {
                        if (insideTriangle(x + i, y + j, t.v)) {
                            auto [alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                            z_interpolated *= w_reciprocal;
                            int index_ans = get_index(x, y) + index;
                            if (z_interpolated < depth_buf_2xSSAA[index_ans]) {
                                depth_buf_2xSSAA[index_ans] = z_interpolated;
                                frame_buf_2xSSAA[index_ans] = t.getColor();
                            }
                        }
                        index++;
                    }
                }
            } else{
                // 检查像素中心是否在三角形内
                if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
                    // 步骤3:计算重心坐标
                    auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
                    // 计算透视校正的深度值，通过查阅资料，w_reciprocal和z_interpolated都可以反应
                    // 三维空间的正确深度信息，分别为w-buffer和z-buffer
                    // w-buffer保存的是经过投影变换后的w坐标，w坐标通常和世界坐标系中的z坐标成正比
                    // 变换到投影空间后，其值依然是线性分布的，这样无论远近的物体，都有相同深度分辨率，
                    //w-buffer的缺点正是z-buffer的优点，即不能用较高的深度分辨率来表现近处的物体，
                    // 作业中要求使用z-buffer，此处我们选用z-buffer
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    // 步骤四:判断深度
                    if (z_interpolated < depth_buf[get_index(x, y)]) {
                        // 更新深度缓冲
                        depth_buf[get_index(x, y)] = z_interpolated;
                        // 填入像素
                        set_pixel(Eigen::Vector3f(x, y, 1.0f), t.getColor());
                    }
                }
            }
        }
    }
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
        std::fill(frame_buf_2xSSAA.begin(), frame_buf_2xSSAA.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_2xSSAA.begin(), depth_buf_2xSSAA.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //利用SSAA进行超采样，每个像素分成四个小像素
    frame_buf_2xSSAA.resize(w * h * 4);
    depth_buf_2xSSAA.resize(w * h * 4);
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