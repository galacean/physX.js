
#pragma once

#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

inline bool canApplyDynamicForce(const PxRigidBody &body) {
    return body.getScene() != nullptr && !(body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) &&
           !(body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION);
}

inline bool addExplosionForce(PxRigidBody &body, PxReal strength, const PxVec3 &origin, double radius,
                              PxReal upwardOffset, PxForceMode::Enum mode) {
    if (!canApplyDynamicForce(body) || !origin.isFinite() || !PxIsFinite(strength) ||
        !PxIsFinite(radius) || radius < 0 || !PxIsFinite(upwardOffset)) return false;

    const PxTransform pose = body.getGlobalPose();
    const PxVec3 center = pose.transform(body.getCMassLocalPose().p);
    PxVec3 point = center;
    PxReal closest = PX_MAX_F32;
    for (PxU32 i = 0, count = body.getNbShapes(); i < count; ++i) {
        PxShape *shape;
        body.getShapes(&shape, 1, i);
        const PxGeometryHolder geometry = shape->getGeometry();
        if (!isVolumeGeometry(geometry.getType())) continue;

        PxVec3 surface;
        const PxReal distance = PxGeometryQuery::pointDistance(origin, geometry.any(), pose * shape->getLocalPose(), &surface);
        if (distance == 0) {
            point = center;
            break;
        }
        if (distance > 0 && distance < closest) {
            closest = distance;
            point = surface;
        }
    }

    const double x = double(point.x) - origin.x;
    const double y = double(point.y) - origin.y;
    const double z = double(point.z) - origin.z;
    const double xzSquared = x * x + z * z;
    const double falloff = radius == 0 ? 1.0 : 1.0 - std::sqrt(xzSquared + y * y) / radius;
    if (falloff <= 0) return false;

    const double raisedY = y + upwardOffset;
    const double length = std::sqrt(xzSquared + raisedY * raisedY);
    if (length == 0) return false;

    const double scale = strength * falloff / length;
    const PxVec3 force(PxReal(x * scale), PxReal(raisedY * scale), PxReal(z * scale));
    const PxVec3 torque = (point - center).cross(force);
    if (!force.isFinite() || !torque.isFinite()) return false;
    // Pass the same mode to both accumulators so acceleration and velocity changes bypass mass and inertia.
    body.addForce(force, mode, true);
    body.addTorque(torque, mode, true);
    return true;
}

EMSCRIPTEN_BINDINGS(physx_actor) {
    enum_<PxForceMode::Enum>("PxForceMode")
            .value("eFORCE", PxForceMode::eFORCE)
            .value("eIMPULSE", PxForceMode::eIMPULSE)
            .value("eVELOCITY_CHANGE", PxForceMode::eVELOCITY_CHANGE)
            .value("eACCELERATION", PxForceMode::eACCELERATION);

    enum_<PxActorFlag::Enum>("PxActorFlag").value("eDISABLE_GRAVITY", PxActorFlag::Enum::eDISABLE_GRAVITY); // ✅

    /** PhysXCollider ✅ */
    class_<PxActor>("PxActor")
            .function("setActorFlag", &PxActor::setActorFlag) // ✅
            .function("release", &PxActor::release); // ✅
    class_<PxRigidActor, base<PxActor>>("PxRigidActor")
            .function("attachShape", &PxRigidActor::attachShape)                            // ✅
            .function("detachShape", &PxRigidActor::detachShape)                            // ✅
            .function("getGlobalPose", &PxRigidActor::getGlobalPose, allow_raw_pointers())  // ✅
            .function("setGlobalPose",
                      optional_override([](PxRigidActor &actor, const PxTransform &pose, bool autowake) {
                          if (!pose.isSane()) {
                              return;
                          }
                          actor.setGlobalPose(pose, autowake);
                      }));  // ✅
            // .function("getShape", optional_override([](PxRigidActor &actor) {
            //               PxShape *shape;
            //               actor.getShapes(&shape, 1);
            //               return shape;
            //           }),
            //           allow_raw_pointers());
    /** PhysXStaticCollider ✅ */
    class_<PxRigidStatic, base<PxRigidActor>>("PxRigidStatic");
    /** PhysXDynamicCollider ✅ */
    class_<PxRigidBody, base<PxRigidActor>>("PxRigidBody")
            .function("setAngularDamping", optional_override([](PxRigidBody &body, PxReal angularDamping) {
                          if (PxIsFinite(angularDamping) && angularDamping >= 0.0f) {
                              body.setAngularDamping(angularDamping);
                          }
                      }))  // ✅
            // .function("getAngularDamping", &PxRigidBody::getAngularDamping)
            .function("setLinearDamping", optional_override([](PxRigidBody &body, PxReal linearDamping) {
                          if (PxIsFinite(linearDamping) && linearDamping >= 0.0f) {
                              body.setLinearDamping(linearDamping);
                          }
                      }))  // ✅
            // .function("getLinearDamping", &PxRigidBody::getLinearDamping)
            .function("setAngularVelocity",
                      optional_override([](PxRigidBody &body, const PxVec3 &vel, bool autowake) {
                          if (!vel.isFinite() || (body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) ||
                              (body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)) {
                              return;
                          }
                          body.setAngularVelocity(vel, autowake);
                      }))                                                    // ✅
            .function("getAngularVelocity", &PxRigidBody::getAngularVelocity) // ✅
            .function("setLinearVelocity",
                      optional_override([](PxRigidBody &body, const PxVec3 &vel, bool autowake) {
                          if (!vel.isFinite() || (body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) ||
                              (body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)) {
                              return;
                          }
                          body.setLinearVelocity(vel, autowake);
                      }))                                                    // ✅
            .function("getLinearVelocity", &PxRigidBody::getLinearVelocity)  // ✅
            .function("setMaxLinearVelocity", &PxRigidBody::setMaxLinearVelocity)
            .function("setMaxAngularVelocity", optional_override([](PxRigidBody &body, PxReal maxAngularVelocity) {
                          if (PxIsFinite(maxAngularVelocity) && maxAngularVelocity >= 0.0f) {
                              body.setMaxAngularVelocity(maxAngularVelocity);
                          }
                      }))  // ✅
            // .function("getMaxAngularVelocity", &PxRigidBody::getMaxAngularVelocity)
            .function("setMaxDepenetrationVelocity",
                      optional_override([](PxRigidBody &body, PxReal maxDepenetrationVelocity) {
                          if (PxIsFinite(maxDepenetrationVelocity) && maxDepenetrationVelocity >= 0.0f) {
                              body.setMaxDepenetrationVelocity(maxDepenetrationVelocity);
                          }
                      }))  // ✅
            // .function("getMaxDepenetrationVelocity", &PxRigidBody::getMaxDepenetrationVelocity)
            .function("setMass", optional_override([](PxRigidBody &body, PxReal mass) {
                          if (PxIsFinite(mass) && mass >= 0.0f) {
                              body.setMass(mass);
                          }
                      }))  // ✅
            // .function("getMass", &PxRigidBody::getMass)
            .function("setCMassLocalPose", optional_override([](PxRigidBody &body, const PxVec3 &pos) {
                          if (!pos.isFinite()) {
                              return;
                          }
                          PxTransform pose = body.getCMassLocalPose();
                          pose.p = pos;
                          body.setCMassLocalPose(pose);
                      }))                                                                    // ✅
            .function("setInertiaTensorRotation", optional_override([](PxRigidBody &body, const PxQuat &rotation) {
                          if (!rotation.isUnit()) {
                              return;
                          }
                          PxTransform pose = body.getCMassLocalPose();
                          pose.q = rotation;
                          body.setCMassLocalPose(pose);
                      }))
            .function("getCMassLocalPose", &PxRigidBody::getCMassLocalPose)                  // ✅
            .function("setMassSpaceInertiaTensor", optional_override([](PxRigidBody &body, const PxVec3 &inertia) {
                          if (inertia.isFinite() && inertia.x >= 0.0f && inertia.y >= 0.0f && inertia.z >= 0.0f) {
                              body.setMassSpaceInertiaTensor(inertia);
                          }
                      }))  // ✅
            .function("getMassSpaceInertiaTensor", &PxRigidBody::getMassSpaceInertiaTensor)  // ✅
            .function("addTorque", optional_override([](PxRigidBody &body, const PxVec3 &torque, PxForceMode::Enum mode) {
                          if (torque.isFinite() && canApplyDynamicForce(body)) {
                              body.addTorque(torque, mode, true);
                          }
                      }))  // ✅
            .function("addForce", optional_override([](PxRigidBody &body, const PxVec3 &force, PxForceMode::Enum mode) {
                          if (force.isFinite() && canApplyDynamicForce(body)) {
                              body.addForce(force, mode, true);
                          }
                      }))  // ✅
            .function("addRelativeForce",
                      optional_override([](PxRigidBody &body, const PxVec3 &force, PxForceMode::Enum mode) {
                          if (!canApplyDynamicForce(body)) return;
                          const PxVec3 worldForce = body.getGlobalPose().rotate(force);
                          if (worldForce.isFinite()) body.addForce(worldForce, mode, true);
                      }))
            .function("addRelativeTorque",
                      optional_override([](PxRigidBody &body, const PxVec3 &torque, PxForceMode::Enum mode) {
                          if (!canApplyDynamicForce(body)) return;
                          const PxVec3 worldTorque = body.getGlobalPose().rotate(torque);
                          if (worldTorque.isFinite()) body.addTorque(worldTorque, mode, true);
                      }))
            .function("addExplosionForce", &addExplosionForce)
            .function("addForceAtPos", optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          if (force.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addForceAtPos(body, force, pos, PxForceMode::eFORCE, true);
                          }
                      }))
            .function("addForceAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          if (force.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addForceAtLocalPos(body, force, pos, PxForceMode::eFORCE, true);
                          }
                      }))
            .function("addLocalForceAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &force, const PxVec3 &pos) {
                          if (force.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addLocalForceAtLocalPos(body, force, pos, PxForceMode::eFORCE, true);
                          }
                      }))
            .function("addImpulseAtPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          if (impulse.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addForceAtPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                          }
                      }))
            .function("addImpulseAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          if (impulse.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addForceAtLocalPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                          }
                      }))
            .function("addLocalImpulseAtLocalPos",
                      optional_override([](PxRigidBody &body, const PxVec3 &impulse, const PxVec3 &pos) {
                          if (impulse.isFinite() && pos.isFinite() && canApplyDynamicForce(body)) {
                              PxRigidBodyExt::addLocalForceAtLocalPos(body, impulse, pos, PxForceMode::eIMPULSE, true);
                          }
                      }))
            .function("getVelocityAtPos", optional_override([](PxRigidBody &body, const PxVec3 &pos, const val &out) {
                          const PxVec3 velocity = PxRigidBodyExt::getVelocityAtPos(body, pos);
                          out.call<void>("set", velocity.x, velocity.y, velocity.z);
                      }))
            .function("getLocalVelocityAtLocalPos", optional_override([](PxRigidBody &body, const PxVec3 &pos, const val &out) {
                          const PxVec3 velocity = PxRigidBodyExt::getLocalVelocityAtLocalPos(body, pos);
                          out.call<void>("set", velocity.x, velocity.y, velocity.z);
                      }))
            .function("setRigidBodyFlag", &PxRigidBody::setRigidBodyFlag) // ✅
            .function("getRigidBodyFlags", optional_override([](PxRigidBody &body, PxRigidBodyFlag::Enum flag) {
                          return (bool)(body.getRigidBodyFlags() & flag);
                      }))  // ✅
            .function("setMassAndUpdateInertia", optional_override([](PxRigidBody &body, PxReal mass) {
                          if (!PxIsFinite(mass) || mass <= 0.0f) {
                              return false;
                          }
                          return PxRigidBodyExt::setMassAndUpdateInertia(body, mass, nullptr, false);
                      })); // ✅
    /** PhysXDynamicCollider ✅ */
    class_<PxRigidDynamic, base<PxRigidBody>>("PxRigidDynamic")
            .function("setDensity", optional_override([](PxRigidDynamic &body, PxReal density,
                                                          bool automaticCenterOfMass, bool automaticInertiaTensor) {
                          const PxScene *scene = body.getScene();
                          if (scene && !PxIsSceneSimulationIdle(*scene)) return 0.0f;
                          return PxRigidBodyExt::setDensity(body, density, automaticCenterOfMass, automaticInertiaTensor);
                      }))
            .function("setSimulationEnabled", optional_override([](PxRigidDynamic &body, bool enabled) {
                          const auto flag = PxActorFlag::eDISABLE_SIMULATION;
                          if (enabled == !(body.getActorFlags() & flag)) return false;
                          body.setActorFlag(flag, !enabled);
                          if (enabled && !(body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
                              const PxScene *scene = body.getScene();
                              body.setWakeCounter(scene ? scene->getWakeCounterResetValue()
                                                        : PxSceneDesc(PxTolerancesScale()).wakeCounterResetValue);
                          }
                          return true;
                      }))
            .function("setSleepEventId", optional_override([](PxRigidDynamic &body, PxU32 id) {
                          body.userData = reinterpret_cast<void *>(static_cast<uintptr_t>(id));
                          body.setActorFlag(PxActorFlag::eSEND_SLEEP_NOTIFIES, id != 0);
                      }))
            .function("setSleepThreshold", optional_override([](PxRigidDynamic &body, PxReal threshold) {
                          if (PxIsFinite(threshold) && threshold >= 0.0f) {
                              body.setSleepThreshold(threshold);
                          }
                      }))  // ✅
            // .function("getSleepThreshold", &PxRigidDynamic::getSleepThreshold)
            .function("setSolverIterationCounts",
                      optional_override([](PxRigidDynamic &body, double minPositionIters, double minVelocityIters) {
                          PxU32 posIters = 0;
                          PxU32 velIters = 0;
                          if (tryParseIntRange(minPositionIters, 1u, 255u, posIters) &&
                              tryParseIntRange(minVelocityIters, 1u, 255u, velIters)) {
                              body.setSolverIterationCounts(posIters, velIters);
                          }
                      }))  // ✅
            .function("wakeUp", optional_override([](PxRigidDynamic &body) {
                          if (canApplyDynamicForce(body)) {
                              body.wakeUp();
                          }
                      }))  // ✅
            // .function("setWakeCounter", &PxRigidDynamic::setWakeCounter)
            .function("isSleeping", optional_override([](const PxRigidDynamic &body) {
                          return body.getScene() ? body.isSleeping() : true;
                      }))  // ✅
            .function("putToSleep", optional_override([](PxRigidDynamic &body) {
                          if (canApplyDynamicForce(body)) {
                              body.putToSleep();
                          }
                      }))  // ✅
            // .function("getWakeCounter", &PxRigidDynamic::getWakeCounter)
            .function("setKinematicTarget",
                      optional_override([](PxRigidDynamic &body, const PxVec3 &pos, const PxQuat &rot) {
                          if (!body.getScene() || !(body.getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) ||
                              (body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) || !pos.isFinite() ||
                              !rot.isSane()) {
                              return;
                          }
                          body.setKinematicTarget(PxTransform(pos, rot));
                      }))  // ✅
            .function("getKinematicTarget", optional_override([](const PxRigidDynamic &body) -> val {
                          PxTransform target;
                          if (body.getKinematicTarget(target)) {
                              return val(target);
                          }
                          return val::null();
                      }))  // ✅
            // .function("setRigidDynamicLockFlag", &PxRigidDynamic::setRigidDynamicLockFlag)
            .function("setRigidDynamicLockFlags", optional_override([](PxRigidDynamic &body, int flags) {
                          return body.setRigidDynamicLockFlags(PxRigidDynamicLockFlags(flags));
                      }));  // ✅
    // class_<PxRigidBodyFlags>("PxRigidBodyFlags");  
    enum_<PxRigidBodyFlag::Enum>("PxRigidBodyFlag") // ✅
            .value("eKINEMATIC", PxRigidBodyFlag::Enum::eKINEMATIC)
            .value("eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES",PxRigidBodyFlag::Enum::eUSE_KINEMATIC_TARGET_FOR_SCENE_QUERIES)
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
