//
// Created by congying on 16-12-7.
//

#ifndef G2O_EDGESCALE_H
#define G2O_EDGESCALE_H

#include <iostream>
using namespace std;

#include "vector1d.h"
#include <g2o/core/base_binary_edge.h>
#include "VertexScale.h"
using namespace g2o;

#define DebugEdgeScale false

class EdgeScale : public BaseBinaryEdge<1, Vector1D, VertexScale, VertexScale> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW EdgeScale(){};
    void computeError(){
        const VertexScale *v1 = static_cast<const VertexScale *>(_vertices[0]);
        const VertexScale *v2 = static_cast<const VertexScale *>(_vertices[1]);
        auto e = _measurement.inverse() * (v1->estimate().inverse() * v2->estimate());
        if (v1->id() == 0 && v2->id() == 1 && DebugEdgeScale) {
            cout << "///////////////////////" << endl;
            cout << "EDGE: " << v1->id() << " to " << v2->id() << ": " << endl;
            cout << "v1: " << v1->estimate() << endl;
            cout << "v2: " << v2->estimate() << endl;
            cout << "measurement: " << _measurement << endl;
            cout << "err: " << e << endl;
            cout << "///////////////////////" << endl;
        }
        _error = e;
    };

    virtual bool read(std::istream& is){
        Vector1D p;
        is >> p[0];
        setMeasurement(p);
        is >> information()(0, 0);
        if (DebugEdgeScale) {
            cout << "edge: "<< p[0] << " - " << information()(0, 0) << endl;
        }
        return true;
    };

    virtual bool write(std::ostream&os) const{
        os << measurement()[0] << " " << information()(0, 0);
        return os.good();
    };
};

#endif //G2O_EDGESCALE_H
