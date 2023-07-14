/*
 * SPDX-FileCopyrightText: 2016-2019 Emanuele Vespa
 * SPDX-FileCopyrightText: 2019-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2021 Nils Funk
 * SPDX-FileCopyrightText: 2021 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_MESH_FACE_HPP
#define SE_MESH_FACE_HPP

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <vector>

namespace se {

template<size_t NumVertexes>
struct MeshFace {
    std::array<Eigen::Vector3f, NumVertexes> vertexes;
    int8_t max_vertex_scale;
    float data = 0.0f;
    static constexpr size_t num_vertexes = NumVertexes;

    MeshFace() : max_vertex_scale(0)
    {
        vertexes.fill(Eigen::Vector3f::Zero());
    }
};

template<typename FaceT>
using Mesh = std::vector<FaceT>;

typedef MeshFace<3> Triangle;
typedef Mesh<Triangle> TriangleMesh;

typedef MeshFace<4> Quad;
typedef Mesh<Quad> QuadMesh;

inline TriangleMesh quad_to_triangle_mesh(const QuadMesh& q)
{
    TriangleMesh t(2 * q.size());
    for (size_t i = 0; i < q.size(); i++) {
        const auto& f = q[i];
        auto& a = t[2 * i];
        a.vertexes[0] = f.vertexes[0];
        a.vertexes[1] = f.vertexes[1];
        a.vertexes[2] = f.vertexes[2];
        a.max_vertex_scale = f.max_vertex_scale;
        a.data = f.data;
        auto& b = t[2 * i + 1];
        b.vertexes[0] = f.vertexes[0];
        b.vertexes[1] = f.vertexes[2];
        b.vertexes[2] = f.vertexes[3];
        b.max_vertex_scale = f.max_vertex_scale;
        b.data = f.data;
    }
    return t;
}

} // namespace se

#endif // SE_MESH_FACE_HPP
