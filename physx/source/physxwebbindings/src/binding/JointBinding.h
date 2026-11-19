
#pragma once

#include <cmath>
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
                      optional_override([](PxJoint &joint, int actor, const PxVec3 &position, const PxQuat &rotation) {
                          if ((actor != 0 && actor != 1) || !position.isFinite() || !rotation.isSane()) {
                              return;
                          }
                          joint.setLocalPose(PxJointActorIndex::Enum(actor), PxTransform(position, rotation));
                      }))  // ✅
            .function("setBreakForce", optional_override([](PxJoint &joint, PxReal force, PxReal torque) {
                          if (!sanitizeNonNegativeMaxF32(force) || !sanitizeNonNegativeMaxF32(torque)) {
                              return;
                          }
                          joint.setBreakForce(force, torque);
                      }))  // ✅
            .function("setConstraintFlag", optional_override([](PxJoint &joint, int flag, bool value) {
                          const auto constraintFlag = PxConstraintFlag::Enum(flag);
                          if (constraintFlag == PxConstraintFlag::eBROKEN ||
                              constraintFlag == PxConstraintFlag::eGPU_COMPATIBLE) {
                              return;
                          }
                          joint.setConstraintFlag(constraintFlag, value);
                      }))  // ✅
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
                                                           PxReal contactDist) {
                          const PxJointAngularLimitPair limit(lowerLimit, upperLimit, contactDist);
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
    /* PhysXConfigurableJoint ✅ */
    // class_<PxD6Joint, base<PxJoint>>("PxD6Joint")
    //         .function("setMotion", optional_override([](PxD6Joint &joint, int axis, int type) {
    //                       joint.setMotion(PxD6Axis::Enum(axis), PxD6Motion::Enum(type));
    //                   }))  // ✅
    //         .function("setHardDistanceLimit", optional_override([](PxD6Joint &joint, PxTolerancesScale &scale,
    //                                                                PxReal extent, PxReal contactDist) {
    //                       joint.setDistanceLimit(PxJointLinearLimit(scale, extent, contactDist));
    //                   }))  // ✅
    //         .function("setSoftDistanceLimit",
    //                   optional_override([](PxD6Joint &joint, PxReal extent, PxReal stiffness, PxReal damping) {
    //                       joint.setDistanceLimit(PxJointLinearLimit(extent, PxSpring(stiffness, damping)));
    //                   }))  // ✅
    //         .function("setHardLinearLimit",
    //                   optional_override([](PxD6Joint &joint, int axis, PxTolerancesScale &scale, PxReal lowerLimit,
    //                                        PxReal upperLimit, PxReal contactDist) {
    //                       joint.setLinearLimit(PxD6Axis::Enum(axis),
    //                                            PxJointLinearLimitPair(scale, lowerLimit, upperLimit, contactDist));
    //                   }))  // ✅
    //         .function("setSoftLinearLimit", optional_override([](PxD6Joint &joint, int axis, PxReal lowerLimit,
    //                                                              PxReal upperLimit, PxReal stiffness, PxReal damping) {
    //                       joint.setLinearLimit(
    //                               PxD6Axis::Enum(axis),
    //                               PxJointLinearLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
    //                   }))  // ✅
    //         .function("setHardTwistLimit",
    //                   optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
    //                       joint.setTwistLimit(PxJointAngularLimitPair(lowerLimit, upperLimit, contactDist));
    //                   }))  // ✅
    //         .function("setSoftTwistLimit", optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit,
    //                                                             PxReal stiffness, PxReal damping) {
    //                       joint.setTwistLimit(
    //                               PxJointAngularLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
    //                   }))  // ✅
    //         .function("setHardSwingLimit",
    //                   optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
    //                       joint.setSwingLimit(PxJointLimitCone(lowerLimit, upperLimit, contactDist));
    //                   }))  // ✅
    //         .function("setSoftSwingLimit", optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit,
    //                                                             PxReal stiffness, PxReal damping) {
    //                       joint.setSwingLimit(PxJointLimitCone(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
    //                   }))  // ✅
    //         .function("setHardPyramidSwingLimit",
    //                   optional_override([](PxD6Joint &joint, PxReal yLimitAngleMin, PxReal yLimitAngleMax,
    //                                        PxReal zLimitAngleMin, PxReal zLimitAngleMax, PxReal contactDist) {
    //                       joint.setPyramidSwingLimit(PxJointLimitPyramid(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin,
    //                                                                      zLimitAngleMax, contactDist));
    //                   }))  // ✅
    //         .function("setSoftPyramidSwingLimit",
    //                   optional_override([](PxD6Joint &joint, PxReal yLimitAngleMin, PxReal yLimitAngleMax,
    //                                        PxReal zLimitAngleMin, PxReal zLimitAngleMax, PxReal stiffness,
    //                                        PxReal damping) {
    //                       joint.setPyramidSwingLimit(PxJointLimitPyramid(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin,
    //                                                                      zLimitAngleMax, PxSpring(stiffness, damping)));
    //                   }))  // ✅
    //         .function("setDrive", optional_override([](PxD6Joint &joint, int index, PxReal driveStiffness,
    //                                                    PxReal driveDamping, PxReal driveForceLimit) {
    //                       joint.setDrive(PxD6Drive::Enum(index),
    //                                      PxD6JointDrive(driveStiffness, driveDamping, driveForceLimit));
    //                   }))  // ✅
    //         .function("setDrivePosition", optional_override([](PxD6Joint &joint, const PxVec3 &pos, const PxQuat &rot) {
    //                       joint.setDrivePosition(PxTransform(pos, rot));
    //                   }))                                                                           // ✅
    //         .function("setDriveVelocity", &PxD6Joint::setDriveVelocity)                             // ✅
    //         .function("setProjectionLinearTolerance", &PxD6Joint::setProjectionLinearTolerance)     // ✅
    //         .function("setProjectionAngularTolerance", &PxD6Joint::setProjectionAngularTolerance);  // ✅
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxJoint>(PxJoint *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten
