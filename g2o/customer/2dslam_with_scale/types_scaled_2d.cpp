//
// Created by congying on 16-12-9.
//

#include "types_scaled_2d.h"

#include <g2o/core/factory.h>
#include "vertex_se2_with_scale.h"
#include "edge_se2_with_scale.h"

namespace g2o {
    G2O_REGISTER_TYPE_GROUP(scaled_2d_slam);

    G2O_REGISTER_TYPE(SCALED_VERTEX, VertexSE2WithScale);
    G2O_REGISTER_TYPE(SCALED_EDGE, EdgeSE2WithScale);
} // end namespace