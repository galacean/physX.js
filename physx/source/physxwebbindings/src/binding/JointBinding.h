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

EMSCRIPTEN_BINDINGS(physx_joint) {
    /* PhysXJoint ✅ */
    class_<PxJoint>("PxJoint")
            .function("setActors", &PxJoint::setActors, allow_raw_pointers())  // ✅
            .function("setLocalPose",
                      optional_override([](PxJoint &joint, int actor, const PxVec3 &position, const PxQuat &rotation) {
                          joint.setLocalPose(PxJointActorIndex::Enum(actor), PxTransform(position, rotation));
                      }))                                        // ✅
            .function("setBreakForce", &PxJoint::setBreakForce)  // ✅
            .function("setConstraintFlag", optional_override([](PxJoint &joint, int flag, bool value) {
                          joint.setConstraintFlag(PxConstraintFlag::Enum(flag), value);
                      }))                                                    // ✅
            .function("setInvMassScale0", &PxJoint::setInvMassScale0)        // ✅
            .function("setInvInertiaScale0", &PxJoint::setInvInertiaScale0)  // ✅
            .function("setInvMassScale1", &PxJoint::setInvMassScale1)        // ✅
            .function("setInvInertiaScale1", &PxJoint::setInvInertiaScale1)  // ✅
            .function("release", &PxJoint::release);
    /* PhysXFixedJoint ✅ */
    class_<PxFixedJoint, base<PxJoint>>("PxFixedJoint")
            .function("setProjectionLinearTolerance", &PxFixedJoint::setProjectionLinearTolerance)     // ✅
            .function("setProjectionAngularTolerance", &PxFixedJoint::setProjectionAngularTolerance);  // ✅
    /* PhysXSphericalJoint ✅ */
    class_<PxSphericalJoint, base<PxJoint>>("PxSphericalJoint")
            .function("setHardLimitCone", optional_override([](PxSphericalJoint &joint, PxReal yLimitAngle,
                                                               PxReal zLimitAngle, PxReal contactDist) {
                          joint.setLimitCone(PxJointLimitCone(yLimitAngle, zLimitAngle, contactDist));
                      }))  // ✅
            .function("setSoftLimitCone", optional_override([](PxSphericalJoint &joint, PxReal yLimitAngle,
                                                               PxReal zLimitAngle, PxReal stiffness, PxReal damping) {
                          joint.setLimitCone(PxJointLimitCone(yLimitAngle, zLimitAngle, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setSphericalJointFlag", optional_override([](PxSphericalJoint &joint, PxReal flag, bool value) {
                          joint.setSphericalJointFlag(PxSphericalJointFlag::Enum(flag), value);
                      }))                                                                                // ✅
            .function("setProjectionLinearTolerance", &PxSphericalJoint::setProjectionLinearTolerance);  // ✅
    /* PhysXHingeJoint ✅ */
    class_<PxRevoluteJoint, base<PxJoint>>("PxRevoluteJoint")
            .function("setHardLimit", optional_override([](PxRevoluteJoint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                           PxReal contactDist) {
                          joint.setLimit(PxJointAngularLimitPair(lowerLimit, upperLimit, contactDist));
                      }))  // ✅
            .function("setSoftLimit", optional_override([](PxRevoluteJoint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                           PxReal stiffness, PxReal damping) {
                          joint.setLimit(PxJointAngularLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
                      }))                                                          // ✅
            .function("setDriveVelocity", &PxRevoluteJoint::setDriveVelocity)      // ✅
            .function("setDriveForceLimit", &PxRevoluteJoint::setDriveForceLimit)  // ✅
            .function("setDriveGearRatio", &PxRevoluteJoint::setDriveGearRatio)    // ✅
            .function("setRevoluteJointFlag", optional_override([](PxRevoluteJoint &joint, PxReal flag, bool value) {
                          joint.setRevoluteJointFlag(PxRevoluteJointFlag::Enum(flag), value);
                      }))                                                                                 // ✅
            .function("setProjectionLinearTolerance", &PxRevoluteJoint::setProjectionLinearTolerance)     // ✅
            .function("setProjectionAngularTolerance", &PxRevoluteJoint::setProjectionAngularTolerance);  // ✅
    /* PhysXSpringJoint ✅ */
    class_<PxDistanceJoint, base<PxJoint>>("PxDistanceJoint")
            .function("setMinDistance", &PxDistanceJoint::setMinDistance)  // ✅
            .function("setMaxDistance", &PxDistanceJoint::setMaxDistance)  // ✅
            .function("setTolerance", &PxDistanceJoint::setTolerance)      // ✅
            .function("setStiffness", &PxDistanceJoint::setStiffness)      // ✅
            .function("setDamping", &PxDistanceJoint::setDamping)          // ✅
            .function("setDistanceJointFlag", optional_override([](PxDistanceJoint &joint, PxReal flag, bool value) {
                          joint.setDistanceJointFlag(PxDistanceJointFlag::Enum(flag), value);
                      }));  // ✅
    /* PhysXTranslationalJoint ✅ */
    class_<PxPrismaticJoint, base<PxJoint>>("PxPrismaticJoint")
            .function("setHardLimit", optional_override([](PxPrismaticJoint &joint, PxTolerancesScale &scale,
                                                           PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
                          joint.setLimit(PxJointLinearLimitPair(scale, lowerLimit, upperLimit, contactDist));
                      }))  // ✅
            .function("setSoftLimit", optional_override([](PxPrismaticJoint &joint, PxReal lowerLimit,
                                                           PxReal upperLimit, PxReal stiffness, PxReal damping) {
                          joint.setLimit(PxJointLinearLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setPrismaticJointFlag", optional_override([](PxPrismaticJoint &joint, PxReal flag, bool value) {
                          joint.setPrismaticJointFlag(PxPrismaticJointFlag::Enum(flag), value);
                      }))                                                                                  // ✅
            .function("setProjectionLinearTolerance", &PxPrismaticJoint::setProjectionLinearTolerance)     // ✅
            .function("setProjectionAngularTolerance", &PxPrismaticJoint::setProjectionAngularTolerance);  // ✅
    /* PhysXConfigurableJoint ✅ */
    class_<PxD6Joint, base<PxJoint>>("PxD6Joint")
            .function("setMotion", optional_override([](PxD6Joint &joint, int axis, int type) {
                          joint.setMotion(PxD6Axis::Enum(axis), PxD6Motion::Enum(type));
                      }))  // ✅
            .function("setHardDistanceLimit", optional_override([](PxD6Joint &joint, PxTolerancesScale &scale,
                                                                   PxReal extent, PxReal contactDist) {
                          joint.setDistanceLimit(PxJointLinearLimit(scale, extent, contactDist));
                      }))  // ✅
            .function("setSoftDistanceLimit",
                      optional_override([](PxD6Joint &joint, PxReal extent, PxReal stiffness, PxReal damping) {
                          joint.setDistanceLimit(PxJointLinearLimit(extent, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setHardLinearLimit",
                      optional_override([](PxD6Joint &joint, int axis, PxTolerancesScale &scale, PxReal lowerLimit,
                                           PxReal upperLimit, PxReal contactDist) {
                          joint.setLinearLimit(PxD6Axis::Enum(axis),
                                               PxJointLinearLimitPair(scale, lowerLimit, upperLimit, contactDist));
                      }))  // ✅
            .function("setSoftLinearLimit", optional_override([](PxD6Joint &joint, int axis, PxReal lowerLimit,
                                                                 PxReal upperLimit, PxReal stiffness, PxReal damping) {
                          joint.setLinearLimit(
                                  PxD6Axis::Enum(axis),
                                  PxJointLinearLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setHardTwistLimit",
                      optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
                          joint.setTwistLimit(PxJointAngularLimitPair(lowerLimit, upperLimit, contactDist));
                      }))  // ✅
            .function("setSoftTwistLimit", optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                                PxReal stiffness, PxReal damping) {
                          joint.setTwistLimit(
                                  PxJointAngularLimitPair(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setHardSwingLimit",
                      optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit, PxReal contactDist) {
                          joint.setSwingLimit(PxJointLimitCone(lowerLimit, upperLimit, contactDist));
                      }))  // ✅
            .function("setSoftSwingLimit", optional_override([](PxD6Joint &joint, PxReal lowerLimit, PxReal upperLimit,
                                                                PxReal stiffness, PxReal damping) {
                          joint.setSwingLimit(PxJointLimitCone(lowerLimit, upperLimit, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setHardPyramidSwingLimit",
                      optional_override([](PxD6Joint &joint, PxReal yLimitAngleMin, PxReal yLimitAngleMax,
                                           PxReal zLimitAngleMin, PxReal zLimitAngleMax, PxReal contactDist) {
                          joint.setPyramidSwingLimit(PxJointLimitPyramid(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin,
                                                                         zLimitAngleMax, contactDist));
                      }))  // ✅
            .function("setSoftPyramidSwingLimit",
                      optional_override([](PxD6Joint &joint, PxReal yLimitAngleMin, PxReal yLimitAngleMax,
                                           PxReal zLimitAngleMin, PxReal zLimitAngleMax, PxReal stiffness,
                                           PxReal damping) {
                          joint.setPyramidSwingLimit(PxJointLimitPyramid(yLimitAngleMin, yLimitAngleMax, zLimitAngleMin,
                                                                         zLimitAngleMax, PxSpring(stiffness, damping)));
                      }))  // ✅
            .function("setDrive", optional_override([](PxD6Joint &joint, int index, PxReal driveStiffness,
                                                       PxReal driveDamping, PxReal driveForceLimit) {
                          joint.setDrive(PxD6Drive::Enum(index),
                                         PxD6JointDrive(driveStiffness, driveDamping, driveForceLimit));
                      }))  // ✅
            .function("setDrivePosition", optional_override([](PxD6Joint &joint, const PxVec3 &pos, const PxQuat &rot) {
                          joint.setDrivePosition(PxTransform(pos, rot));
                      }))                                                                           // ✅
            .function("setDriveVelocity", &PxD6Joint::setDriveVelocity)                             // ✅
            .function("setProjectionLinearTolerance", &PxD6Joint::setProjectionLinearTolerance)     // ✅
            .function("setProjectionAngularTolerance", &PxD6Joint::setProjectionAngularTolerance);  // ✅
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxJoint>(PxJoint *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten