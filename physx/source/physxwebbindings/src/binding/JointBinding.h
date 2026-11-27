
#pragma once

#include <cmath>
#include <cstdint>
#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

inline bool sanitizeNonNegativeMaxF32(PxReal &value) {
    if (std::isnan(value)) {
        return false;
    }
    if (value < 0.0f) {
        value = 0.0f;
    } else if (value > PX_MAX_F32) {
        value = PX_MAX_F32;
    }
    return true;
}

EMSCRIPTEN_BINDINGS(physx_joint) {
    /* PhysXJoint ✅ */
    class_<PxJoint>("PxJoint")
            .function("setActors",
                      optional_override([](PxJoint &joint, PxRigidActor *actor0, PxRigidActor *actor1) {
                          if (!areValidJointActors(actor0, actor1)) {
                              return;
                          }
                          joint.setActors(actor0, actor1);
                      }),
                      allow_raw_pointers())  // ✅
            .function("setLocalPose",
                      optional_override([](PxJoint &joint, double actor, const PxVec3 &position, const PxQuat &rotation) {
                          PxU32 actorIndex = 0;
                          if (!tryParseIntRange(actor, 0u, 1u, actorIndex) || !position.isFinite() || !rotation.isSane()) {
                              return;
                          }
                          joint.setLocalPose(PxJointActorIndex::Enum(actorIndex), PxTransform(position, rotation));
                      }))  // ✅
            .function("setBreakForce", optional_override([](PxJoint &joint, PxReal force, PxReal torque) {
                          if (!sanitizeNonNegativeMaxF32(force) || !sanitizeNonNegativeMaxF32(torque)) {
                              return;
                          }
                          joint.setBreakForce(force, torque);
                      }))  // ✅
            .function("setConstraintFlag", optional_override([](PxJoint &joint, int flag, bool value) {
                          const int readOnlyMask = static_cast<int>(PxConstraintFlag::eBROKEN) |
                                                   static_cast<int>(PxConstraintFlag::eGPU_COMPATIBLE);
                          if ((flag & readOnlyMask) != 0) {
                              return;
                          }
                          joint.setConstraintFlag(PxConstraintFlag::Enum(flag), value);
                      }))  // ✅
            .function("setBreakEventId", optional_override([](PxJoint &joint, PxU32 id) {
                          joint.userData = reinterpret_cast<void *>(static_cast<uintptr_t>(id));
                      }))
            .function("isBroken", optional_override([](const PxJoint &joint) {
                          return joint.getConstraintFlags().isSet(PxConstraintFlag::eBROKEN);
                      }))
            .function("getReactionForce", optional_override([](const PxJoint &joint, const val &linear, const val &angular) {
                          PxVec3 force, torque;
                          joint.getConstraint()->getForce(force, torque);
                          linear.call<void>("set", force.x, force.y, force.z);
                          angular.call<void>("set", torque.x, torque.y, torque.z);
                      }))
            .function("setInvMassScale0", optional_override([](PxJoint &joint, PxReal invMassScale) {
                          if (PxIsFinite(invMassScale) && invMassScale >= 0.0f) {
                              joint.setInvMassScale0(invMassScale);
                          }
                      }))  // ✅
            .function("setInvInertiaScale0", optional_override([](PxJoint &joint, PxReal invInertiaScale) {
                          if (PxIsFinite(invInertiaScale) && invInertiaScale >= 0.0f) {
                              joint.setInvInertiaScale0(invInertiaScale);
                          }
                      }))  // ✅
            .function("setInvMassScale1", optional_override([](PxJoint &joint, PxReal invMassScale) {
                          if (PxIsFinite(invMassScale) && invMassScale >= 0.0f) {
                              joint.setInvMassScale1(invMassScale);
                          }
                      }))  // ✅
            .function("setInvInertiaScale1", optional_override([](PxJoint &joint, PxReal invInertiaScale) {
                          if (PxIsFinite(invInertiaScale) && invInertiaScale >= 0.0f) {
                              joint.setInvInertiaScale1(invInertiaScale);
                          }
                      }))  // ✅
            .function("release", &PxJoint::release); // ✅
    /* PhysXFixedJoint ✅ */
    class_<PxFixedJoint, base<PxJoint>>("PxFixedJoint");
            // .function("setProjectionLinearTolerance", &PxFixedJoint::setProjectionLinearTolerance)     // ✅
            // .function("setProjectionAngularTolerance", &PxFixedJoint::setProjectionAngularTolerance);  // ✅
    /* PhysXHingeJoint ✅ */
    class_<PxRevoluteJoint, base<PxJoint>>("PxRevoluteJoint")
            .function("getAngle", &PxRevoluteJoint::getAngle)        // ✅
            .function("getVelocity", &PxRevoluteJoint::getVelocity)  // ✅
            .function("setHardLimit", optional_override([](PxRevoluteJoint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                           PxReal contactDist, PxReal restitution) {
                          PxJointAngularLimitPair limit(lowerLimit, upperLimit, contactDist);
                          limit.restitution = restitution;
                          if (limit.isValid() && limit.lower > -PxTwoPi && limit.upper < PxTwoPi) {
                              joint.setLimit(limit);
                          }
                      }))  // ✅
            .function("setSoftLimit", optional_override([](PxRevoluteJoint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                           PxReal stiffness, PxReal damping) {
                          const PxJointAngularLimitPair limit(lowerLimit, upperLimit, PxSpring(stiffness, damping));
                          if (limit.isValid() && limit.lower > -PxTwoPi && limit.upper < PxTwoPi) {
                              joint.setLimit(limit);
                          }
                      }))  // ✅
            .function("setDriveVelocity",
                      optional_override([](PxRevoluteJoint &joint, PxReal velocity, bool autowake) {
                          if (PxIsFinite(velocity)) {
                              joint.setDriveVelocity(velocity, autowake);
                          }
                      }))  // ✅
            .function("setDriveForceLimit", optional_override([](PxRevoluteJoint &joint, PxReal forceLimit) {
                          if (sanitizeNonNegativeMaxF32(forceLimit)) {
                              joint.setDriveForceLimit(forceLimit);
                          }
                      }))  // ✅
            .function("setDriveGearRatio", optional_override([](PxRevoluteJoint &joint, PxReal gearRatio) {
                          if (PxIsFinite(gearRatio) && gearRatio > 0.0f) {
                              joint.setDriveGearRatio(gearRatio);
                          }
                      }))  // ✅
            .function("setRevoluteJointFlag", optional_override([](PxRevoluteJoint &joint, int flag, bool value) {
                          joint.setRevoluteJointFlag(PxRevoluteJointFlag::Enum(flag), value);
                      }));  // ✅
            // .function("setProjectionLinearTolerance", &PxRevoluteJoint::setProjectionLinearTolerance)     // ✅
            // .function("setProjectionAngularTolerance", &PxRevoluteJoint::setProjectionAngularTolerance);  // ✅
    /* PhysXSpringJoint ✅ */
    class_<PxDistanceJoint, base<PxJoint>>("PxDistanceJoint")
            .function("setMinDistance", optional_override([](PxDistanceJoint &joint, PxReal distance) {
                          if (PxIsFinite(distance)) {
                              joint.setMinDistance(distance);
                          }
                      }))  // ✅
            .function("setMaxDistance", optional_override([](PxDistanceJoint &joint, PxReal distance) {
                          if (PxIsFinite(distance)) {
                              joint.setMaxDistance(distance);
                          }
                      }))  // ✅
            .function("setTolerance", optional_override([](PxDistanceJoint &joint, PxReal tolerance) {
                          if (PxIsFinite(tolerance)) {
                              joint.setTolerance(tolerance);
                          }
                      }))  // ✅
            .function("setStiffness", optional_override([](PxDistanceJoint &joint, PxReal stiffness) {
                          if (PxIsFinite(stiffness)) {
                              joint.setStiffness(stiffness);
                          }
                      }))  // ✅
            .function("setDamping", optional_override([](PxDistanceJoint &joint, PxReal damping) {
                          if (PxIsFinite(damping)) {
                              joint.setDamping(damping);
                          }
                      }))  // ✅
            .function("setDistanceJointFlag", optional_override([](PxDistanceJoint &joint, int flag, bool value) {
                          joint.setDistanceJointFlag(PxDistanceJointFlag::Enum(flag), value);
                      }));  // ✅
    /* PhysXTranslationalJoint ✅ */
    // class_<PxPrismaticJoint, base<PxJoint>>("PxPrismaticJoint")
    //         .function("setHardLimit", optional_override([](PxPrismaticJoint &joint, PxTolerancesScale &scale,
    //                                                        PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
    //                       joint.setLimit(PxJointLinearLimitPair(scale, lowerLimit, upperLimit, contactDist));
    //                   }))  // ✅
    //         .function("setSoftLimit", optional_override([](PxPrismaticJoint &joint, PxReal lowerLimit,
    //                                                        PxReal upperLimit, PxReal stiffness, PxReal damping) {
    //                       joint.setLimit(PxJointLinearLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
    //                   }));  // ✅
            // .function("setPrismaticJointFlag", optional_override([](PxPrismaticJoint &joint, PxReal flag, bool value) {
            //               joint.setPrismaticJointFlag(PxPrismaticJointFlag::Enum(flag), value);
            //           }))                                                                                  // ✅
            // .function("setProjectionLinearTolerance", &PxPrismaticJoint::setProjectionLinearTolerance)     // ✅
            // .function("setProjectionAngularTolerance", &PxPrismaticJoint::setProjectionAngularTolerance);  // ✅
    class_<PxD6Joint, base<PxJoint>>("PxD6Joint")
            .function("setMotion", optional_override([](PxD6Joint &joint, PxU32 axis, PxU32 motion) {
                          if (axis < PxD6Axis::eCOUNT && motion <= PxD6Motion::eFREE) {
                              joint.setMotion(PxD6Axis::Enum(axis), PxD6Motion::Enum(motion));
                          }
                      }))
            .function("setLinearLimit", optional_override([](PxD6Joint &joint, PxU32 axis, PxReal lower, PxReal upper,
                                                             PxReal stiffness, PxReal damping) {
                          const PxJointLinearLimitPair limit(lower, upper, PxSpring(stiffness, damping));
                          if (axis <= PxD6Axis::eZ && limit.isValid()) {
                              joint.setLinearLimit(PxD6Axis::Enum(axis), limit);
                          }
                      }))
            .function("setTwistLimit", optional_override([](PxD6Joint &joint, PxReal lower, PxReal upper,
                                                            PxReal stiffness, PxReal damping) {
                          const PxJointAngularLimitPair limit(lower, upper, PxSpring(stiffness, damping));
                          if (limit.isValid()) {
                              joint.setTwistLimit(limit);
                          }
                      }))
            .function("setSwingLimit", optional_override([](PxD6Joint &joint, PxReal y, PxReal z,
                                                            PxReal stiffness, PxReal damping) {
                          const PxJointLimitCone limit(y, z, PxSpring(stiffness, damping));
                          if (limit.isValid()) {
                              joint.setSwingLimit(limit);
                          }
                      }))
            .function("setDrive", optional_override([](PxD6Joint &joint, PxU32 index, PxReal stiffness, PxReal damping,
                                                       PxReal forceLimit, bool acceleration) {
                          if (index < PxD6Drive::eCOUNT && sanitizeNonNegativeMaxF32(forceLimit)) {
                              const PxD6JointDrive drive(stiffness, damping, forceLimit, acceleration);
                              if (drive.isValid()) {
                                  joint.setDrive(PxD6Drive::Enum(index), drive);
                              }
                          }
                      }))
            .function("setDrivePosition", optional_override([](PxD6Joint &joint, const PxVec3 &position,
                                                               const PxQuat &rotation) {
                          if (position.isFinite() && rotation.isSane()) {
                              joint.setDrivePosition(PxTransform(position, rotation), false);
                          }
                      }))
            .function("setDriveVelocity", optional_override([](PxD6Joint &joint, const PxVec3 &linear,
                                                               const PxVec3 &angular) {
                          if (linear.isFinite() && angular.isFinite()) {
                              joint.setDriveVelocity(linear, angular, false);
                          }
                      }));
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxJoint>(PxJoint *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten
