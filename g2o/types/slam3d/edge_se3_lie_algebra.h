#pragma once

#include "EXTERNAL/sophus/so3.hpp"
#include "EXTERNAL/sophus/se3.hpp"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/block_solver.h"

#include "vertex_se3_lie_algebra.h"

class EdgeSE3LieAlgebra : public g2o::BaseBinaryEdge<6, Sophus::SE3, VertexSE3LieAlgebra, VertexSE3LieAlgebra>
{
public:
};