#pragma once

#include "EXTERNAL/sophus/so3.hpp"
#include "EXTERNAL/sophus/se3.hpp"

#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/block_solver.h"

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, Sophus::SE3>
{
public:

};