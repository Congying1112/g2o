// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef VERTEX_SE2_WITH_SCALE_H
#define VERTEX_SE2_WITH_SCALE_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"
#include "se2_with_scale.h"
//#include "g2o_types_slam2d_api.h"
/**
 * \brief 2D pose Vertex, (x,y,theta)
 */
using namespace g2o;
using namespace std;

class VertexSE2WithScale : public BaseVertex<4, SE2WithScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW VertexSE2WithScale();

    virtual void setToOriginImpl() {
        _estimate = SE2WithScale();
    }

    virtual void oplusImpl(const double* update)
    {
#if 0
        if(id() == s_min_id) {
            cout << "##########################vertex oplusImpl################################" << endl;
            cout << _estimate.translation() << ", " << _estimate.rotation().angle() << ", " << _estimate.scale() << endl;
        }
#endif
        // 写了两个方案，最终效果都差不多

        // 方案1
        double angle = _estimate.rotation().angle();
        double cos_s = cos(angle);
        double sin_s = sin(angle);
        double s = _estimate.scale()[0];
        Vector2D t = _estimate.translation();
        double xj = s * cos_s * update[0] - s * sin_s * update[1] + t[0];
        double yj = s * sin_s * update[0] + s * cos_s * update[1] + t[1];
        _estimate.setTranslation(Vector2D(xj, yj));
        double new_s = s * (1.0 + update[3]);
        if (new_s < 0.8)
            new_s = 0.8;
        if (new_s > 1.2)
            new_s = 1.2;
        _estimate.setScale(Vector1D(new_s));
        _estimate.setRotation(Eigen::Rotation2Dd(normalize_theta(angle + update[2])));

        // 方案2
        /*Vector2D t = _estimate.translation();
        t += Eigen::Map<const Vector2D>(update);
        _estimate.setTranslation(t);
        if (id() != s_min_id) {
            Vector1D scale;
            scale[0] = _estimate.scale()[0] * (1.0 + update[3]);
            _estimate.setScale(scale);
            double angle=normalize_theta(_estimate.rotation().angle() + update[2]);
            _estimate.setRotation(Eigen::Rotation2Dd(angle));
        }*/
#if 0
        if (id() == s_min_id) {
            cout << update[0] << ", " << update[1] << ", " << update[2] << ", " << update[3] << ", " << endl;
            cout << _estimate.translation() << ", " << _estimate.rotation().angle() << ", " << _estimate.scale() << endl;
        }
#endif
    }

virtual bool setEstimateDataImpl(const double* est){
    _estimate = SE2WithScale(est[0], est[1], est[2], est[3]);
    return true;
}

virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector4D> v(est);
    v = _estimate.toVector();
    return true;
}

virtual int estimateDimension() const { return 4; }

virtual bool setMinimalEstimateDataImpl(const double* est){
    return setEstimateData(est);
}

virtual bool getMinimalEstimateData(double* est) const {
    return getEstimateData(est);
}

virtual int minimalEstimateDimension() const { return 4; }

virtual bool read(std::istream& is);
virtual bool write(std::ostream& os) const;

private:
    static int s_min_id;
};
#endif
