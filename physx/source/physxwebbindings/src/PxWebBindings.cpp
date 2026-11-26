
#include <emscripten.h>
#include <emscripten/bind.h>
#include <PsSocket.h>

#include <chrono>

#include "BindingHelper.h"
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

template <auto Create>
static auto createJoint(PxPhysics &physics, PxRigidActor *actor0, const PxVec3 &position0, const PxQuat &rotation0,
                        PxRigidActor *actor1, const PxVec3 &position1, const PxQuat &rotation1)
    -> decltype(Create(physics, actor0, PxTransform(position0, rotation0), actor1, PxTransform(position1, rotation1))) {
    if (!areValidJointActors(actor0, actor1) || !position0.isFinite() || !rotation0.isSane() ||
        !position1.isFinite() || !rotation1.isSane()) {
        return nullptr;
    }
    return Create(physics, actor0, PxTransform(position0, rotation0), actor1, PxTransform(position1, rotation1));
}

//----------------------------------------------------------------------------------------------------------------------
EMSCRIPTEN_BINDINGS(physx) {
    constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);

    class_<PxPvd>("PxPvd")
#if PX_SUPPORT_PVD
            .function("connect", &PxPvd::connect)
#endif
            ;

    // Global functions
    // These are generally system/scene level initialization
    function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());
    function("PxInitExtensions", &PxInitExtensions, allow_raw_pointers());
    function("PxCloseExtensions", &PxCloseExtensions, allow_raw_pointers());
    function("PxDefaultCpuDispatcherCreate", &PxDefaultCpuDispatcherCreate, allow_raw_pointers());
    function("PxCreatePhysics", optional_override([](PxU32 version, PxFoundation &foundation,
                                                    const PxTolerancesScale &scale, bool trackAllocations, PxPvd *pvd) {
        PxPhysics *physics = PxCreateBasePhysics(version, foundation, scale, trackAllocations, pvd);
        if (physics && physics->getNbScenes() == 0) PxRegisterHeightFields(*physics);
        return physics;
    }), allow_raw_pointers());

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
            .function("createScene", optional_override([](PxPhysics &physics, const PxSceneDesc &desc) {
                          return createSceneWithOwnedDispatcher(physics, desc);
                      }), allow_raw_pointers())
            .function("createShape",
                      select_overload<PxShape *(const PxGeometry &, const PxMaterial &, bool, PxShapeFlags)>(
                              &PxPhysics::createShape),
                      allow_raw_pointers())
            .function("createMaterial", &PxPhysics::createMaterial, allow_raw_pointers())
            .function("createRigidDynamic", &PxPhysics::createRigidDynamic, allow_raw_pointers())
            .function("createRigidStatic", &PxPhysics::createRigidStatic, allow_raw_pointers())
            .function("createFixedJoint", &createJoint<PxFixedJointCreate>, allow_raw_pointers())
            .function("createRevoluteJoint", &createJoint<PxRevoluteJointCreate>, allow_raw_pointers())
            .function("createDistanceJoint", &createJoint<PxDistanceJointCreate>, allow_raw_pointers())
            .function("createPrismaticJoint", &createJoint<PxPrismaticJointCreate>, allow_raw_pointers())
            .function("createD6Joint", &createJoint<PxD6JointCreate>, allow_raw_pointers());

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
