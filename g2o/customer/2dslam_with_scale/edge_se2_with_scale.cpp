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

#include "edge_se2_with_scale.h"

EdgeSE2WithScale::EdgeSE2WithScale() :
        BaseBinaryEdge<4, SE2WithScale, VertexSE2WithScale, VertexSE2WithScale>()
{
}

bool EdgeSE2WithScale::read(std::istream& is)
{
      Vector4D p;
      is >> p[0] >> p[1] >> p[2] >> p[3];
      setMeasurement(SE2WithScale(p));
      for (int i = 0; i < 4; ++i)
          for (int j = i; j < 4; ++j) {
              information()(j, i) = information()(i, j) = 0;
        }
      information()(0, 0) = information()(1, 1) = information()(2, 2) = information()(3, 3) = 1.0;
      information()(2, 2) = .01;
      information()(3, 3) = 100.0;
      return true;
}

bool EdgeSE2WithScale::write(std::ostream& os) const
{
  Vector4D p = measurement().toVector();
  os << p[0] << " " << p[1] << " " << p[2] << " " << p[3];
  return os.good();
}