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

EMSCRIPTEN_BINDINGS(physx_actor) {
    enum_<PxForceMode::Enum>("PxForceMode")
            .value("eFORCE", PxForceMode::Enum::eFORCE)
            .value("eIMPULSE", PxForceMode::Enum::eIMPULSE)
            .value("eVELOCITY_CHANGE", PxForceMode::Enum::eVELOCITY_CHANGE)
            .value("eACCELERATION", PxForceMode::Enum::eACCELERATION);

    enum_<PxActorFlag::Enum>("PxActorFlag").value("eDISABLE_GRAVITY", PxActorFlag::Enum::eDISABLE_GRAVITY);

    /** PhysXCollider ✅ */
    class_<PxActor>("PxActor").function("setActorFlag", &PxActor::setActorFlag).function("release", &PxActor::release);
    class_<PxRigidActor, base<PxActor>>("PxRigidActor")
            .function("attachShape", &PxRigidActor::attachShape)                            // ✅
            .function("detachShape", &PxRigidActor::detachShape)                            // ✅
            .function("getGlobalPose", &PxRigidActor::getGlobalPose, allow_raw_pointers())  // ✅
            .function("setGlobalPose", &PxRigidActor::setGlobalPose, allow_raw_pointers())  // ✅
            .function("getShape", optional_override([](PxRigidActor &actor) {
                          PxShape *shape;
                          actor.getShapes(&shape, 1);
                          return shape;
                      }),
                      allow_raw_pointers());
    /** PhysXStaticCollider ✅ */
    class_<PxRigidStatic, base<PxRigidActor>>("PxRigidStatic");
    /** PhysXDynamicCollider ✅ */
    class_<PxRigidBody, base<PxRigidActor>>("PxRigidBody")
            .function("setAngularDamping", &PxRigidBody::setAngularDamping)  // ✅
            .function("getAngularDamping", &PxRigidBody::getAngularDamping)
            .function("setLinearDamping", &PxRigidBody::setLinearDamping)  // ✅
            .function("getLinearDamping", &PxRigidBody::getLinearDamping)
            .function("setAngularVelocity", &PxRigidBody::setAngularVelocity)  // ✅
            .function("getAngularVelocity", &PxRigidBody::getAngularVelocity)
            .function("setLinearVelocity", &PxRigidBody::setLinearVelocity)  // ✅
            .function("getLinearVelocity", &PxRigidBody::getLinearVelocity)
            .function("setMaxAngularVelocity", &PxRigidBody::setMaxAngularVelocity)  // ✅
            .function("getMaxAngularVelocity", &PxRigidBody::getMaxAngularVelocity)
            .function("setMaxDepenetrationVelocity", &PxRigidBody::setMaxDepenetrationVelocity)  // ✅
            .function("getMaxDepenetrationVelocity", &PxRigidBody::getMaxDepenetrationVelocity)
            .function("setMass", &PxRigidBody::setMass)  // ✅
            .function("getMass", &PxRigidBody::getMass)
            .function("setCMassLocalPose", optional_override([](PxRigidBody &body, const PxVec3 &pos) {
                          return body.setCMassLocalPose(PxTransform(pos, PxQuat(PxIDENTITY::PxIdentity)));
                      }))                                                                    // ✅
            .function("setMassSpaceInertiaTensor", &PxRigidBody::setMassSpaceInertiaTensor)  // ✅
            .function("addTorque", optional_override([](PxRigidBody &body, const PxVec3 &torque) {
                          body.addTorque(torque, PxForceMode::eFORCE, true);
                      }))  // ✅
            .function("addForce", optional_override([](PxRigidBody &body, const PxVec3 &force) {
                          body.addForce(force, PxForceMode::eFORCE, true);
                      }))  // ✅
            .function("addForceAtPos", optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          PxRigidBodyExt::addForceAtPos(body, force, pos, PxForceMode::eFORCE, true);
                      }))
            .function("addForceAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          PxRigidBodyExt::addForceAtLocalPos(body, force, pos, PxForceMode::eFORCE, true);
                      }))
            .function("addLocalForceAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          PxRigidBodyExt::addLocalForceAtLocalPos(body, force, pos, PxForceMode::eFORCE, true);
                      }))
            .function("addImpulseAtPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          PxRigidBodyExt::addForceAtPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                      }))
            .function("addImpulseAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          PxRigidBodyExt::addForceAtLocalPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                      }))
            .function("addLocalImpulseAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          PxRigidBodyExt::addLocalForceAtLocalPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                      }))
            .function("getVelocityAtPos", optional_override([](PxRigidBody &body, const PxVec3 &pos) {
                          return PxRigidBodyExt::getVelocityAtPos(body, pos);
                      }))
            .function("getLocalVelocityAtLocalPos", optional_override([](PxRigidBody &body, const PxVec3 &pos) {
                          return PxRigidBodyExt::getLocalVelocityAtLocalPos(body, pos);
                      }))
            .function("setRigidBodyFlag", &PxRigidBody::setRigidBodyFlag)
            .function("getRigidBodyFlags", optional_override([](PxRigidBody &body) {
                          return (bool)(body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC);
                      }))  // ✅
            .function("setMassAndUpdateInertia", optional_override([](PxRigidBody &body, PxReal mass) {
                          return PxRigidBodyExt::setMassAndUpdateInertia(body, mass, nullptr, false);
                      }));
    /** PhysXDynamicCollider ✅ */
    class_<PxRigidDynamic, base<PxRigidBody>>("PxRigidDynamic")
            .function("setSleepThreshold", &PxRigidDynamic::setSleepThreshold)  // ✅
            .function("getSleepThreshold", &PxRigidDynamic::getSleepThreshold)
            .function("setSolverIterationCounts", &PxRigidDynamic::setSolverIterationCounts)  // ✅
            .function("wakeUp", &PxRigidDynamic::wakeUp)                                      // ✅
            .function("setWakeCounter", &PxRigidDynamic::setWakeCounter)
            .function("isSleeping", &PxRigidDynamic::isSleeping)
            .function("putToSleep", &PxRigidDynamic::putToSleep)  // ✅
            .function("getWakeCounter", &PxRigidDynamic::getWakeCounter)
            .function("setKinematicTarget",
                      optional_override([](PxRigidDynamic &body, const PxVec3 &pos, const PxQuat &rot) {
                          return body.setKinematicTarget(PxTransform(pos, rot));
                      }))  // ✅
            .function("setRigidDynamicLockFlag", &PxRigidDynamic::setRigidDynamicLockFlag)
            .function("setRigidDynamicLockFlags", optional_override([](PxRigidDynamic &body, int flags) {
                          return body.setRigidDynamicLockFlags(PxRigidDynamicLockFlags(flags));
                      }));  // ✅
    class_<PxRigidBodyFlags>("PxRigidBodyFlags");
    enum_<PxRigidBodyFlag::Enum>("PxRigidBodyFlag")
            .value("eKINEMATIC", PxRigidBodyFlag::Enum::eKINEMATIC)
            .value("eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES",
                   PxRigidBodyFlag::Enum::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
            .value("eENABLE_CCD", PxRigidBodyFlag::Enum::eENABLE_CCD)
            .value("eENABLE_CCD_FRICTION", PxRigidBodyFlag::Enum::eENABLE_CCD_FRICTION)
            .value("eENABLE_POSE_INTEGRATION_PREVIEW", PxRigidBodyFlag::Enum::eENABLE_POSE_INTEGRATION_PREVIEW)
            .value("eENABLE_SPECULATIVE_CCD", PxRigidBodyFlag::Enum::eENABLE_SPECULATIVE_CCD)
            .value("eENABLE_CCD_MAX_CONTACT_IMPULSE", PxRigidBodyFlag::Enum::eENABLE_CCD_MAX_CONTACT_IMPULSE)
            .value("eRETAIN_ACCELERATIONS", PxRigidBodyFlag::Enum::eRETAIN_ACCELERATIONS);
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxRigidDynamic>(PxRigidDynamic *) { /* do nothing */
}

template <>
void raw_destructor<PxRigidBody>(PxRigidBody *) { /* do nothing */
}

template <>
void raw_destructor<PxRigidActor>(PxRigidActor *) { /* do nothing */
}

template <>
void raw_destructor<PxActor>(PxActor *) { /* do nothing */
}

template <>
void raw_destructor<PxRigidStatic>(PxRigidStatic *) { /* do nothing */
}


}  // namespace internal
}  // namespace emscripten