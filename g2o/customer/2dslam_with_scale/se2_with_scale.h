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

#ifndef SE2_WITH_SCALE_H_
#define SE2_WITH_SCALE_H_

#include "g2o/stuff/misc.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "vector1d.h"


/**
 * \brief represent SE2
 */
using namespace g2o;
using namespace std;
class SE2WithScale {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW SE2WithScale():_R(0),_t(0,0),_s(1.0){}

    SE2WithScale(const Vector4D& v):_R(v[2]),_t(v[0],v[1]),_s(v[3]){}

    SE2WithScale(double x, double y, double theta, double scale):_R(theta),_t(x,y),_s(scale){}

    //! translational component
    inline const Vector2D& translation() const {return _t;}
    void setTranslation(const Vector2D& t_) {_t=t_;}

    //! rotational component
    inline const Eigen::Rotation2Dd& rotation() const {return _R;}
    void setRotation(const Eigen::Rotation2Dd& R_) {_R=R_;}

    //! scale component
    inline const Vector1D& scale() const {return _s;}
    void setScale(const Vector1D& s_) {_s=s_;}

    //! concatenate two SE2 elements (motion composition)
    inline SE2WithScale operator * (const SE2WithScale& tr2) const{
        SE2WithScale result(*this);
        result *= tr2;
        return result;
    }

    //! motion composition operator
    inline SE2WithScale& operator *= (const SE2WithScale& tr2){
        double ci = cos(_R.angle());
        double si = sin(_R.angle());
        _t[0] = _s[0] * ci * tr2._t[0] - _s[0] * si * tr2._t[1] + _t[0];
        _t[1] = _s[0] * si * tr2._t[0] + _s[0] * ci * tr2._t[1] + _t[1];
        _s[0] *= tr2._s[0];
        _R.angle() += tr2._R.angle();
        _R.angle() = normalize_theta(_R.angle());
        return *this;
    }

    inline double operator [](int i) const {
        assert (i>=0 && i<4);
        if (i<2)
            return _t(i);
        else if (i==2)
            return _R.angle();
        else
            return _s[0];
    }


    //! assign from a 4D vector (x, y, theta, scale)
    inline void fromVector (const Vector4D& v){
        *this = SE2WithScale(v);
    }

    //! convert to a 4D vector (x, y, theta, scale)
    inline Vector4D toVector() const {
        return Vector4D(_t.x(), _t.y(), _R.angle(), _s[0]);
    }

protected:
    Eigen::Rotation2Dd _R;
    Vector2D _t;
    Vector1D _s;
};
#endif
