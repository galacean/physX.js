//
// Created by Oasis on 2023/3/21.
//

#include <emscripten.h>
#include <emscripten/bind.h>
#include <PsSocket.h>

#include <chrono>

#include "binding/ActorBinding.h"
#include "binding/ControllerBinding.h"
#include "binding/CookingBinding.h"
#include "binding/JointBinding.h"
#include "binding/MathBinding.h"
#include "binding/PVDBinding.h"
#include "binding/SceneBinding.h"
#include "binding/ShapeBinding.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

//----------------------------------------------------------------------------------------------------------------------
EMSCRIPTEN_BINDINGS(physx) {
    constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);
    class_<PxPvd>("PxPvd").function("connect", &PxPvd::connect);

    // Global functions
    // These are generally system/scene level initialization
    function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());
    function("PxInitExtensions", &PxInitExtensions, allow_raw_pointers());
    function("PxCloseExtensions", &PxCloseExtensions, allow_raw_pointers());
    function("PxDefaultCpuDispatcherCreate", &PxDefaultCpuDispatcherCreate, allow_raw_pointers());
    function("PxCreatePhysics", &PxCreateBasePhysics, allow_raw_pointers());

    class_<PxAllocatorCallback>("PxAllocatorCallback");
    class_<PxDefaultAllocator, base<PxAllocatorCallback>>("PxDefaultAllocator").constructor<>();
    class_<PxTolerancesScale>("PxTolerancesScale")
            .constructor<>()
            .property("speed", &PxTolerancesScale::speed)
            .property("length", &PxTolerancesScale::length);

    class_<PxFoundation>("PxFoundation").function("release", &PxFoundation::release);

    /** PhysXPhysics ✅ */
    class_<PxPhysics>("PxPhysics")
            .function("release", &PxPhysics::release)
            .function("getTolerancesScale", &PxPhysics::getTolerancesScale)
            .function("createScene", &PxPhysics::createScene, allow_raw_pointers())
            .function("createShape",
                      select_overload<PxShape *(const PxGeometry &, const PxMaterial &, bool, PxShapeFlags)>(
                              &PxPhysics::createShape),
                      allow_raw_pointers())
            .function("createMaterial", &PxPhysics::createMaterial, allow_raw_pointers())
            .function("createRigidDynamic", &PxPhysics::createRigidDynamic, allow_raw_pointers())
            .function("createRigidStatic", &PxPhysics::createRigidStatic, allow_raw_pointers())
            .function("createFixedJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxFixedJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                    actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createRevoluteJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxRevoluteJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                       actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createSphericalJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxSphericalJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                        actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createDistanceJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxDistanceJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                       actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createPrismaticJoint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxPrismaticJointCreate(physics, actor0, PxTransform(localPosition0, localRotation0),
                                                        actor1, PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers())  // ✅
            .function("createD6Joint",
                      optional_override([](PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &localPosition0,
                                           const PxQuat &localRotation0, PxRigidActor *actor1,
                                           const PxVec3 &localPosition1, const PxQuat &localRotation1) {
                          return PxD6JointCreate(physics, actor0, PxTransform(localPosition0, localRotation0), actor1,
                                                 PxTransform(localPosition1, localRotation1));
                      }),
                      allow_raw_pointers());  // ✅

    class_<PxErrorCallback>("PxErrorCallback");
    class_<PxDefaultErrorCallback, base<PxErrorCallback>>("PxDefaultErrorCallback").constructor<>();

    class_<PxCpuDispatcher>("PxCpuDispatcher");
    class_<PxBVHStructure>("PxBVHStructure");
    class_<PxBaseTask>("PxBaseTask");
    class_<PxDefaultCpuDispatcher, base<PxCpuDispatcher>>("PxDefaultCpuDispatcher");

    class_<PxFilterData>("PxFilterData");
    class_<PxPairFlags>("PxPairFlags");
    class_<PxFilterFlags>("PxFilterFlags");
    enum_<PxPairFlag::Enum>("PxPairFlag");
    enum_<PxFilterFlag::Enum>("PxFilterFlag");
}

namespace emscripten {
namespace internal {
// Physx uses private destructors all over the place for its own reference counting
// embind doesn't deal with this well, so we have to override the destructors to keep them private
// in the bindings
// See: https://github.com/emscripten-core/emscripten/issues/5587
template <>
void raw_destructor<PxFoundation>(PxFoundation *) { /* do nothing */
}

template <>
void raw_destructor<PxBVHStructure>(PxBVHStructure *) { /* do nothing */
}

template <>
void raw_destructor<PxPvd>(PxPvd *) { /* do nothing */
}

}  // namespace internal
}  // namespace emscripten
