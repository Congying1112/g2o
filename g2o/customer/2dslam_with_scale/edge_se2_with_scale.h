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

#ifndef G2O_EDGE_SE2_H
#define G2O_EDGE_SE2_H

#include "vertex_se2_with_scale.h"
#include "g2o/config.h"
#include "g2o/core/base_binary_edge.h"


/**
 * \brief 2D edge between two Vertex2
 */
using namespace g2o;
using namespace std;
class EdgeSE2WithScale : public BaseBinaryEdge<4, SE2WithScale, VertexSE2WithScale, VertexSE2WithScale>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW EdgeSE2WithScale();
    void computeError()
    {
        const VertexSE2WithScale* v1 = static_cast<const VertexSE2WithScale*>(_vertices[0]);
        const VertexSE2WithScale* v2 = static_cast<const VertexSE2WithScale*>(_vertices[1]);
        auto est_v2 = v1->estimate() * _measurement;
        for (int i = 0; i < 4; i++) {
          _error[i] = (v2->estimate().toVector()[i] - est_v2.toVector()[i]) * (v2->estimate().toVector()[i] - est_v2.toVector()[i]) * information()(i, i);
        }
        //cout << ":: error :: " << v1->id() <<"-"<<v2->id() << " : " << _error[0] + _error[1] + _error[2] + _error[3] << endl;

#if 0
        if (v1->id() == 3 && v2->id() == 0 || 1) {
          cout << "############################### " << endl;
          cout << ":::v1" << endl << v1->estimate().toVector() << endl;
          cout << ":::v2" << endl << v2->estimate().toVector() << endl;
          cout << ":::measurement" << endl << _measurement.toVector() << endl;
          cout << ":::est_v2" << endl << est_v2.toVector() << endl;
          cout << ":::error" << endl << _error << endl;
          cout << ":: error :: " << v1->id() <<"-"<<v2->id() << " : " << _error[0] + _error[1] + _error[2] + _error[3] << endl;
          cout << "############################### " << endl;
          // exit(0);
        }
        // static map<int, map<int, vector<double>>> error_history;
        // error_history[v1->id()][v2->id()].push_back(_error[0] + _error[1] + _error[2] + _error[3]);
        // for (auto e_it1 : error_history) {
        //   for(auto e_it2 : e_it1.second) {
        //     cout << e_it1.first << ", " << e_it2.first << endl;
        //     for (auto e :e_it2.second) {
        //       cout << e << endl;
        //     }
        //   }
        // }
#endif
    }
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    virtual void setMeasurement(const SE2WithScale& m){
      _measurement = m;
    }
};

#endif
