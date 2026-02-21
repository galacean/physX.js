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

// Cooking result storage (single-threaded WASM, no synchronization needed)
static PxU32 g_lastConvexCookingResult = 0;
static PxU32 g_lastTriMeshCookingResult = 0;

PxU32 getLastConvexCookingResult() { return g_lastConvexCookingResult; }
PxU32 getLastTriMeshCookingResult() { return g_lastTriMeshCookingResult; }

PxConvexMesh *createConvexMeshFromBuffer(int vertices, PxU32 vertCount, PxCooking &cooking, PxPhysics &physics) {
    PxConvexMeshDesc convexDesc;
    convexDesc.points.count = vertCount;
    convexDesc.points.stride = sizeof(PxVec3);
    convexDesc.points.data = (PxVec3 *)vertices;
    convexDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX;

    PxConvexMeshCookingResult::Enum result;
    PxConvexMesh *convexMesh = cooking.createConvexMesh(convexDesc, physics.getPhysicsInsertionCallback(), &result);
    g_lastConvexCookingResult = (PxU32)result;

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

    PxTriangleMeshCookingResult::Enum result;
    PxTriangleMesh *triangleMesh = cooking.createTriangleMesh(meshDesc, physics.getPhysicsInsertionCallback(), &result);
    g_lastTriMeshCookingResult = (PxU32)result;
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

// Helper function to create TriangleMeshGeometry with scale
PxTriangleMeshGeometry *createTriMeshGeometry(PxTriangleMesh *mesh,
                                               float scaleX, float scaleY, float scaleZ,
                                               int flags) {
    PxMeshScale scale(PxVec3(scaleX, scaleY, scaleZ), PxQuat(PxIdentity));
    return new PxTriangleMeshGeometry(mesh, scale, PxMeshGeometryFlags(flags));
}

// Helper function to create ConvexMeshGeometry with scale
PxConvexMeshGeometry *createConvexMeshGeometry(PxConvexMesh *mesh,
                                                float scaleX, float scaleY, float scaleZ,
                                                int flags) {
    PxMeshScale scale(PxVec3(scaleX, scaleY, scaleZ), PxQuat(PxIdentity));
    return new PxConvexMeshGeometry(mesh, scale, PxConvexMeshGeometryFlags(flags));
}

// Helper function to create TriangleMesh shape directly
PxShape *createTriMeshShape(PxTriangleMesh *mesh,
                            float scaleX, float scaleY, float scaleZ,
                            int geoFlags, int shapeFlags,
                            PxMaterial &material,
                            PxPhysics &physics) {
    if (!mesh) {
        return nullptr;
    }

    PxMeshScale scale(PxVec3(scaleX, scaleY, scaleZ), PxQuat(PxIdentity));
    PxTriangleMeshGeometry geometry(mesh, scale, PxMeshGeometryFlags(geoFlags));
    PxShape *shape = physics.createShape(geometry, material, true, PxShapeFlags(shapeFlags));

    return shape;
}

// Helper functions for PxCookingParams properties (PxFlags needs wrapper)
PxU32 getCookingMeshPreprocessParams(PxCookingParams &params) {
    return (PxU32)params.meshPreprocessParams;
}
void setCookingMeshPreprocessParams(PxCookingParams &params, PxU32 flags) {
    params.meshPreprocessParams = PxMeshPreprocessingFlags(flags);
}

// Midphase type helper: 0 = eBVH33, 1 = eBVH34
void setCookingMidphaseType(PxCookingParams &params, PxU32 type) {
    params.midphaseDesc.setToDefault(static_cast<PxMeshMidPhase::Enum>(type));
}

// Helper function to create ConvexMesh shape directly
PxShape *createConvexMeshShape(PxConvexMesh *mesh,
                               float scaleX, float scaleY, float scaleZ,
                               int geoFlags, int shapeFlags,
                               PxMaterial &material,
                               PxPhysics &physics) {
    PxMeshScale scale(PxVec3(scaleX, scaleY, scaleZ), PxQuat(PxIdentity));
    PxConvexMeshGeometry geometry(mesh, scale, PxConvexMeshGeometryFlags(geoFlags));
    return physics.createShape(geometry, material, true, PxShapeFlags(shapeFlags));
}

EMSCRIPTEN_BINDINGS(physx_cooking) {
    function("PxCreateCooking", &PxCreateCooking, allow_raw_pointers());
    function("createTriMeshGeometry", &createTriMeshGeometry, allow_raw_pointers());
    function("createConvexMeshGeometry", &createConvexMeshGeometry, allow_raw_pointers());
    function("createTriMeshShape", &createTriMeshShape, allow_raw_pointers());
    function("createConvexMeshShape", &createConvexMeshShape, allow_raw_pointers());

    // PxVec3 and PxQuat are already bound as value_object in MathBinding.h
    // Use object literals in JS: { x: 1, y: 2, z: 3 } and { x: 0, y: 0, z: 0, w: 1 }
    class_<PxMeshScale>("PxMeshScale").constructor<const PxVec3 &, const PxQuat &>();

    class_<PxTriangleMesh>("PxTriangleMesh")
            .function("release", &PxTriangleMesh::release)
            .function("getNbVertices", &PxTriangleMesh::getNbVertices)
            .function("getNbTriangles", &PxTriangleMesh::getNbTriangles);
    class_<PxTriangleMeshGeometry, base<PxGeometry>>("PxTriangleMeshGeometry")
            .constructor<PxTriangleMesh *, const PxMeshScale &, PxMeshGeometryFlags>();
    class_<PxMeshGeometryFlags>("PxMeshGeometryFlags").constructor<int>();
    enum_<PxMeshGeometryFlag::Enum>("PxMeshGeometryFlag")
            .value("eDOUBLE_SIDED", PxMeshGeometryFlag::Enum::eDOUBLE_SIDED);

    class_<PxConvexMesh>("PxConvexMesh")
            .function("release", &PxConvexMesh::release)
            .function("getNbVertices", &PxConvexMesh::getNbVertices)
            .function("getNbPolygons", &PxConvexMesh::getNbPolygons);
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
            .function("setParams", &PxCooking::setParams)
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
            // flags: 传数值位掩码，如 0 或 PxHeightFieldFlag::eNO_BOUNDARY_EDGES
            .function("createHeightField",
                      optional_override([](PxCooking &cooking, int samples, PxU32 nbRows,
                                           PxU32 nbColumns, int flags, PxPhysics &physics) {
                          return createHeightField(samples, nbRows, nbColumns, PxHeightFieldFlags(flags), cooking, physics);
                      }),
                      allow_raw_pointers());
    class_<PxCookingParams>("PxCookingParams")
            .constructor<PxTolerancesScale>()
            .property("meshWeldTolerance", &PxCookingParams::meshWeldTolerance)
            .property("areaTestEpsilon", &PxCookingParams::areaTestEpsilon)
            .property("planeTolerance", &PxCookingParams::planeTolerance)
            .property("buildTriangleAdjacencies", &PxCookingParams::buildTriangleAdjacencies);

    // meshPreprocessParams needs function access (PxFlags type)
    function("getCookingMeshPreprocessParams", &getCookingMeshPreprocessParams, allow_raw_pointers());
    function("setCookingMeshPreprocessParams", &setCookingMeshPreprocessParams, allow_raw_pointers());
    // Cooking result getters
    function("getLastConvexCookingResult", &getLastConvexCookingResult);
    function("getLastTriMeshCookingResult", &getLastTriMeshCookingResult);
    // Midphase type setter (0 = BVH33, 1 = BVH34)
    function("setCookingMidphaseType", &setCookingMidphaseType, allow_raw_pointers());
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
