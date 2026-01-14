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
                              PxU32 triangleCount,
                              bool isU16,
                              PxCooking &cooking,
                              PxPhysics &physics) {
    PxTriangleMeshDesc meshDesc;
    meshDesc.points.count = vertCount;
    meshDesc.points.stride = sizeof(PxVec3);
    meshDesc.points.data = (PxVec3 *)vertices;

    meshDesc.triangles.count = triangleCount;
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

PxHeightField *createHeightField(int samples,
                                  PxU32 nbRows,
                                  PxU32 nbColumns,
                                  PxHeightFieldFlags flags,
                                  PxCooking &cooking,
                                  PxPhysics &physics) {
    PxHeightFieldDesc hfDesc;
    hfDesc.nbRows = nbRows;
    hfDesc.nbColumns = nbColumns;
    hfDesc.format = PxHeightFieldFormat::eS16_TM;
    hfDesc.samples.data = (PxHeightFieldSample *)samples;
    hfDesc.samples.stride = sizeof(PxHeightFieldSample);
    hfDesc.flags = flags;

    PxHeightField *heightField = cooking.createHeightField(hfDesc, physics.getPhysicsInsertionCallback());
    return heightField;
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

    // HeightField 相关
    enum_<PxHeightFieldFormat::Enum>("PxHeightFieldFormat")
            .value("eS16_TM", PxHeightFieldFormat::Enum::eS16_TM);

    enum_<PxHeightFieldFlag::Enum>("PxHeightFieldFlag")
            .value("eNO_BOUNDARY_EDGES", PxHeightFieldFlag::Enum::eNO_BOUNDARY_EDGES);

    class_<PxHeightFieldFlags>("PxHeightFieldFlags").constructor<int>();

    class_<PxHeightField>("PxHeightField")
            .function("release", &PxHeightField::release)
            .function("getNbRows", &PxHeightField::getNbRows)
            .function("getNbColumns", &PxHeightField::getNbColumns)
            .function("getHeight", &PxHeightField::getHeight);

    class_<PxHeightFieldGeometry, base<PxGeometry>>("PxHeightFieldGeometry")
            .constructor<PxHeightField *, PxMeshGeometryFlags, PxReal, PxReal, PxReal>();

    class_<PxCooking>("PxCooking")
            .function("release", &PxCooking::release)
            .function("createConvexMesh",
                      optional_override([](PxCooking &cooking, int vertices, PxU32 vertCount, PxPhysics &physics) {
                          return createConvexMeshFromBuffer(vertices, vertCount, cooking, physics);
                      }),
                      allow_raw_pointers())
            .function("createTriMesh",
                      optional_override([](PxCooking &cooking, int vertices, PxU32 vertCount, int indices,
                                           PxU32 triangleCount, bool isU16, PxPhysics &physics) {
                          return createTriMesh(vertices, vertCount, indices, triangleCount, isU16, cooking, physics);
                      }),
                      allow_raw_pointers())
            .function("createHeightField",
                      optional_override([](PxCooking &cooking, int samples, PxU32 nbRows,
                                           PxU32 nbColumns, int flags, PxPhysics &physics) {
                          return createHeightField(samples, nbRows, nbColumns, PxHeightFieldFlags(flags), cooking, physics);
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

template <>
void raw_destructor<PxHeightField>(PxHeightField *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten
