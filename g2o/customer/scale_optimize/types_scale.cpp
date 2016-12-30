//
// Created by congying on 16-12-7.
//

#include "types_scale.h"
#include <g2o/core/factory.h>
#include "VertexScale.h"
#include "EdgeScale.h"


namespace g2o {
    G2O_REGISTER_TYPE_GROUP(scale);

    G2O_REGISTER_TYPE(SCALE_VERTEX, VertexScale);
    G2O_REGISTER_TYPE(SCALE_EDGE, EdgeScale);
} // end namespace