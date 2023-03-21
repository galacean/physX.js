//
// Created by Oasis on 2023/3/21.
//

#pragma once

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten.h>
#include <emscripten/bind.h>

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