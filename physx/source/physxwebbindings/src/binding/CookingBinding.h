//
// Created by Oasis on 2023/3/21.
//

#pragma once

#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

PxConvexMesh *createConvexMeshFromBuffer(int vertices,
                                         PxU32 vertCount,
                                         int indices,
                                         PxU32 indexCount,
                                         bool isU16,
                                         PxCooking &cooking,
                                         PxPhysics &physics) {
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = vertCount;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = (PxVec3 *)vertices;
    if (isU16) {
        convexDesc.indices.stride = 3 * sizeof(PxU16);
        convexDesc.indices.data = (PxU16 *)indices;
        convexDesc.flags = PxConvexFlag::e16_BIT_INDICES;
    } else {
        convexDesc.indices.stride = 3 * sizeof(PxU32);
        convexDesc.indices.data = (PxU32 *)indices;
    }
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxConvexMesh *convexMesh = cooking.createConvexMesh(convexDesc, physics.getPhysicsInsertionCallback());

    return convexMesh;
}

PxConvexMesh *createConvexMeshFromBuffer(int vertices, PxU32 vertCount, PxCooking &cooking, PxPhysics &physics) {
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = vertCount;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = (PxVec3 *)vertices;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxConvexMesh *convexMesh = cooking.createConvexMesh(convexDesc, physics.getPhysicsInsertionCallback());

    return convexMesh;
}

PxTriangleMesh *createTriMesh(int vertices,
                              PxU32 vertCount,
                              int indices,
                              PxU32 indexCount,
                              bool isU16,
                              PxCooking &cooking,
                              PxPhysics &physics) {
    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count = vertCount;
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.points.data = (PxVec3 *)vertices;

    meshDesc.triangles.count = indexCount;
    if (isU16) {
        meshDesc.triangles.stride = 3 * sizeof(PxU16);
        meshDesc.triangles.data = (PxU16 *)indices;
        meshDesc.flags = PxMeshFlag::e16_BIT_INDICES;
    } else {
        meshDesc.triangles.stride = 3 * sizeof(PxU32);
        meshDesc.triangles.data = (PxU32 *)indices;
    }

    PxTriangleMesh *triangleMesh = cooking.createTriangleMesh(meshDesc, physics.getPhysicsInsertionCallback());
    return triangleMesh;
}

EMSCRIPTEN_BINDINGS(physx_cooking) {
    function("PxCreateCooking", &PxCreateCooking, allow_raw_pointers());

    class_<PxMeshScale>("PxMeshScale").constructor<const PxVec3 &, const PxQuat &>();

    class_<PxTriangleMesh>("PxTriangleMesh").function("release", &PxTriangleMesh::release);
    class_<PxTriangleMeshGeometry, base<PxGeometry>>("PxTriangleMeshGeometry")
            .constructor<PxTriangleMesh *, const PxMeshScale &, PxMeshGeometryFlags>();
    class_<PxMeshGeometryFlags>("PxMeshGeometryFlags").constructor<int>();
    enum_<PxMeshGeometryFlag::Enum>("PxMeshGeometryFlag")
            .value("eDOUBLE_SIDED", PxMeshGeometryFlag::Enum::eDOUBLE_SIDED);

    class_<PxConvexMesh>("PxConvexMesh").function("release", &PxConvexMesh::release);
    class_<PxConvexMeshGeometry, base<PxGeometry>>("PxConvexMeshGeometry")
            .constructor<PxConvexMesh *, const PxMeshScale &, PxConvexMeshGeometryFlags>();
    class_<PxConvexMeshGeometryFlags>("PxConvexMeshGeometryFlags").constructor<int>();
    enum_<PxConvexMeshGeometryFlag::Enum>("PxConvexMeshGeometryFlag")
            .value("eTIGHT_BOUNDS", PxConvexMeshGeometryFlag::Enum::eTIGHT_BOUNDS);

    class_<PxCooking>("PxCooking")
            .function("createConvexMeshWithIndices",
                      optional_override([](PxCooking &cooking, int vertices, PxU32 vertCount, int indices,
                                           PxU32 indexCount, bool isU16, PxPhysics &physics) {
                          return createConvexMeshFromBuffer(vertices, vertCount, indices, indexCount, isU16, cooking,
                                                            physics);
                      }),
                      allow_raw_pointers())
            .function("createConvexMesh",
                      optional_override([](PxCooking &cooking, int vertices, PxU32 vertCount, PxPhysics &physics) {
                          return createConvexMeshFromBuffer(vertices, vertCount, cooking, physics);
                      }),
                      allow_raw_pointers())
            .function("createTriMesh",
                      optional_override([](PxCooking &cooking, int vertices, PxU32 vertCount, int indices,
                                           PxU32 indexCount, bool isU16, PxPhysics &physics) {
                          return createTriMesh(vertices, vertCount, indices, indexCount, isU16, cooking, physics);
                      }),
                      allow_raw_pointers());
    class_<PxCookingParams>("PxCookingParams").constructor<PxTolerancesScale>();
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxCooking>(PxCooking *) { /* do nothing */
}

template <>
void raw_destructor<PxConvexMesh>(PxConvexMesh *) { /* do nothing */
}

template <>
void raw_destructor<PxTriangleMesh>(PxTriangleMesh *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten