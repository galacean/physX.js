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

EMSCRIPTEN_BINDINGS(physx_math) {
    // Define PxsetCMassLocalPoseec3, PxQuat and PxTransform as value objects to allow sumerian Vector3
    // and Quaternion to be used directly without the need to free the memory
    value_object<PxVec3>("PxVec3").field("x", &PxVec3::x).field("y", &PxVec3::y).field("z", &PxVec3::z);
    register_vector<PxVec3>("PxVec3Vector");
    value_object<PxQuat>("PxQuat")
            .field("x", &PxQuat::x)
            .field("y", &PxQuat::y)
            .field("z", &PxQuat::z)
            .field("w", &PxQuat::w);
    value_object<PxTransform>("PxTransform").field("translation", &PxTransform::p).field("rotation", &PxTransform::q);
    value_object<PxExtendedVec3>("PxExtendedVec3")
            .field("x", &PxExtendedVec3::x)
            .field("y", &PxExtendedVec3::y)
            .field("z", &PxExtendedVec3::z);

    enum_<PxIDENTITY>("PxIDENTITY").value("PxIdentity", PxIDENTITY::PxIdentity);
}