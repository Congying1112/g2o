//
// Created by congying on 16-12-7.
//

#ifndef G2O_VERTEXSCALE_H
#define G2O_VERTEXSCALE_H

#include "g2o/core/base_vertex.h"
using namespace g2o;

#include "vector1d.h"

using namespace std;

#define DebugVertexScale false

class VertexScale : public BaseVertex<1, Vector1D> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW VertexScale(){};
    virtual void setToOriginImpl(){
        _estimate = Vector1D();
    };
    virtual void oplusImpl(const double *update){
        Vector1D up(update[0]);
        if (id() == 0 && DebugVertexScale) {
            cout << "#############"<< endl;
            cout << "update: " << update[0]<< endl;
            cout<< "estimate: " << _estimate;
        }
        _estimate = _estimate + up;
        if (id() == 0 && DebugVertexScale) {
            cout << " -> " << _estimate << endl;
        }
    };
    virtual bool read(std::istream& is){
        Vector1D p;
        is >> p[0];
        if (DebugVertexScale) {
            cout << p[0] << endl;
        }
        p[0] = 1.0;
        setEstimate(p);
        return true;
    };
    virtual bool write(std::ostream& os) const{
        Vector1D p = estimate();
        os << p[0];
        return os.good();
    };
};

#endif //G2O_VERTEXSCALE_H
