#include "rasterizer.h"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <math.h>
#include <stdexcept>

using namespace Eigen;
using namespace std;

rst::pos_buf_id rst::Rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::Rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}
