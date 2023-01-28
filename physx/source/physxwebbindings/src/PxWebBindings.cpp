#include <emscripten.h>
#include <emscripten/bind.h>
#include <PsSocket.h>

#include <chrono>

#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

#if PX_DEBUG || PX_PROFILE || PX_CHECKED

struct PxPvdTransportWrapper : public wrapper<PxPvdTransport> {
    EMSCRIPTEN_WRAPPER(PxPvdTransportWrapper)

    void unlock() override {}

    void flush() override {}

    void release() override {}

    PxPvdTransport &lock() override { return *this; }

    uint64_t getWrittenDataSize() override { return 0; }

    bool connect() override { return call<bool>("connect"); }

    void disconnect() override { call<void>("disconnect"); }

    bool isConnected() override { return call<bool>("isConnected"); }

    bool write(const uint8_t *inBytes, uint32_t inLength) override {
        return call<bool>("write", int(inBytes), int(inLength));
    }
};

#endif

//----------------------------------------------------------------------------------------------------------------------
struct PxRaycastCallbackWrapper : public wrapper<PxRaycastCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxRaycastCallbackWrapper)

    PxAgain processTouches(const PxRaycastHit *buffer, PxU32 nbHits) override {
        for (PxU32 i = 0; i < nbHits; i++) {
            bool again = call<PxAgain>("processTouches", buffer[i]);
            if (!again) {
                return false;
            }
        }
        return true;
    }
};

PxRaycastHit *allocateRaycastHitBuffers(PxU32 nb) {
    auto *myArray = new PxRaycastHit[nb];
    return myArray;
}

//----------------------------------------------------------------------------------------------------------------------
struct PxSweepCallbackWrapper : public wrapper<PxSweepCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxSweepCallbackWrapper)

    PxAgain processTouches(const PxSweepHit *buffer, PxU32 nbHits) override {
        for (PxU32 i = 0; i < nbHits; i++) {
            bool again = call<PxAgain>("processTouches", buffer[i]);
            if (!again) {
                return false;
            }
        }
        return true;
    }
};

PxSweepHit *allocateSweepHitBuffers(PxU32 nb) {
    auto *myArray = new PxSweepHit[nb];
    return myArray;
}

//----------------------------------------------------------------------------------------------------------------------
struct PxQueryFilterCallbackWrapper : public wrapper<PxQueryFilterCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxQueryFilterCallbackWrapper)

    PxQueryHitType::Enum postFilter(const PxFilterData &filterData, const PxQueryHit &hit) override {
        return (PxQueryHitType::Enum)call<int>("postFilter", filterData, hit);
    }

    PxQueryHitType::Enum preFilter(const PxFilterData &filterData,
                                   const PxShape *shape,
                                   const PxRigidActor *actor,
                                   PxHitFlags &) override {
        return (PxQueryHitType::Enum)call<int>("preFilter", filterData, shape, actor);
    }
};

//----------------------------------------------------------------------------------------------------------------------
struct PxSimulationEventCallbackWrapper : public wrapper<PxSimulationEventCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxSimulationEventCallbackWrapper)

    void onConstraintBreak(PxConstraintInfo *, PxU32) override {}

    void onWake(PxActor **, PxU32) override {}

    void onSleep(PxActor **, PxU32) override {}

    void onContact(const PxContactPairHeader &, const PxContactPair *pairs, PxU32 nbPairs) override {
        for (PxU32 i = 0; i < nbPairs; i++) {
            const PxContactPair &cp = pairs[i];

            if (cp.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_CCD)) {
                call<void>("onContactBegin", cp.shapes[0], cp.shapes[1]);
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                if (!cp.flags.isSet(PxContactPairFlag::Enum::eREMOVED_SHAPE_0) &&
                    !cp.flags.isSet(PxContactPairFlag::Enum::eREMOVED_SHAPE_1)) {
                    call<void>("onContactEnd", cp.shapes[0], cp.shapes[1]);
                }
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
                call<void>("onContactPersist", cp.shapes[0], cp.shapes[1]);
            }
        }
    }

    void onTrigger(PxTriggerPair *pairs, PxU32 count) override {
        for (PxU32 i = 0; i < count; i++) {
            const PxTriggerPair &tp = pairs[i];

            if (tp.status & PxPairFlag::eNOTIFY_TOUCH_FOUND) {
                call<void>("onTriggerBegin", tp.triggerShape, tp.otherShape);
            } else if (tp.status & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                if (!tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_OTHER) &&
                    !tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_TRIGGER)) {
                    call<void>("onTriggerEnd", tp.triggerShape, tp.otherShape);
                }
            }
        }
    }

    void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) override {}
};

PxFilterFlags DefaultFilterShader(PxFilterObjectAttributes attributes0,
                                  PxFilterData,
                                  PxFilterObjectAttributes attributes1,
                                  PxFilterData,
                                  PxPairFlags &pairFlags,
                                  const void *,
                                  PxU32) {
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT | PxPairFlag::eDETECT_CCD_CONTACT;
        return PxFilterFlag::eDEFAULT;
    }
    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_LOST |
                PxPairFlag::eNOTIFY_TOUCH_PERSISTS | PxPairFlag::eDETECT_CCD_CONTACT;
    return PxFilterFlag::eDEFAULT;
}

PxSceneDesc *getDefaultSceneDesc(PxTolerancesScale &scale, int numThreads, PxSimulationEventCallback *callback) {
    auto *sceneDesc = new PxSceneDesc(scale);
    sceneDesc->gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc->cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    sceneDesc->filterShader = DefaultFilterShader;
    sceneDesc->simulationEventCallback = callback;
    sceneDesc->kineKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc->staticKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc->flags |= PxSceneFlag::eENABLE_CCD;
    return sceneDesc;
}

//----------------------------------------------------------------------------------------------------------------------
EMSCRIPTEN_BINDINGS(physx) {
#if PX_DEBUG || PX_PROFILE || PX_CHECKED
    class_<PxPvdTransport>("PxPvdTransport")
            .allow_subclass<PxPvdTransportWrapper>("PxPvdTransportWrapper", constructor<>());

    function("PxCreatePvd", &PxCreatePvd, allow_raw_pointers());

    class_<PxPvdInstrumentationFlags>("PxPvdInstrumentationFlags").constructor<int>();
    enum_<PxPvdInstrumentationFlag::Enum>("PxPvdInstrumentationFlag")
            .value("eALL", PxPvdInstrumentationFlag::Enum::eALL)
            .value("eDEBUG", PxPvdInstrumentationFlag::Enum::eDEBUG)
            .value("ePROFILE", PxPvdInstrumentationFlag::Enum::ePROFILE)
            .value("eMEMORY", PxPvdInstrumentationFlag::Enum::eMEMORY);

#endif
    class_<PxPvd>("PxPvd").function("connect", &PxPvd::connect);

    constant("PX_PHYSICS_VERSION", PX_PHYSICS_VERSION);

    // Global functions
    // These are generally system/scene level initialization
    function("PxCreateFoundation", &PxCreateFoundation, allow_raw_pointers());
    function("PxInitExtensions", &PxInitExtensions, allow_raw_pointers());
    function("PxDefaultCpuDispatcherCreate", &PxDefaultCpuDispatcherCreate, allow_raw_pointers());
    function("PxCreatePhysics", &PxCreateBasePhysics, allow_raw_pointers());
    function("PxCreatePlane", &PxCreatePlane, allow_raw_pointers());
    function("getDefaultSceneDesc", &getDefaultSceneDesc, allow_raw_pointers());
    function("PxDefaultSimulationFilterShader", &PxDefaultSimulationFilterShader, allow_raw_pointers());

    class_<PxSimulationEventCallback>("PxSimulationEventCallback")
            .allow_subclass<PxSimulationEventCallbackWrapper>("PxSimulationEventCallbackWrapper");

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

    class_<PxAllocatorCallback>("PxAllocatorCallback");
    class_<PxDefaultAllocator, base<PxAllocatorCallback>>("PxDefaultAllocator").constructor<>();
    class_<PxTolerancesScale>("PxTolerancesScale")
            .constructor<>()
            .property("speed", &PxTolerancesScale::speed)
            .property("length", &PxTolerancesScale::length);

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

    enum_<PxForceMode::Enum>("PxForceMode")
            .value("eFORCE", PxForceMode::Enum::eFORCE)
            .value("eIMPULSE", PxForceMode::Enum::eIMPULSE)
            .value("eVELOCITY_CHANGE", PxForceMode::Enum::eVELOCITY_CHANGE)
            .value("eACCELERATION", PxForceMode::Enum::eACCELERATION);

    class_<PxSceneDesc>("PxSceneDesc").constructor<PxTolerancesScale>().property("gravity", &PxSceneDesc::gravity);

    class_<PxFoundation>("PxFoundation").function("release", &PxFoundation::release);

    class_<PxSceneFlags>("PxSceneFlags");
    enum_<PxSceneFlag::Enum>("PxSceneFlag")
            .value("eENABLE_ACTIVE_ACTORS ", PxSceneFlag::Enum::eENABLE_ACTIVE_ACTORS)
            .value("eENABLE_CCD", PxSceneFlag::Enum::eENABLE_CCD)
            .value("eDISABLE_CCD_RESWEEP", PxSceneFlag::Enum::eDISABLE_CCD_RESWEEP)
            .value("eADAPTIVE_FORCE", PxSceneFlag::Enum::eADAPTIVE_FORCE)
            .value("eENABLE_PCM", PxSceneFlag::Enum::eENABLE_PCM)
            .value("eDISABLE_CONTACT_REPORT_BUFFER_RESIZE", PxSceneFlag::Enum::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
            .value("eDISABLE_CONTACT_CACHE", PxSceneFlag::Enum::eDISABLE_CONTACT_CACHE)
            .value("eREQUIRE_RW_LOCK", PxSceneFlag::Enum::eREQUIRE_RW_LOCK)
            .value("eENABLE_STABILIZATION", PxSceneFlag::Enum::eENABLE_STABILIZATION)
            .value("eENABLE_AVERAGE_POINT", PxSceneFlag::Enum::eENABLE_AVERAGE_POINT)
            .value("eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS", PxSceneFlag::Enum::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
            .value("eENABLE_ENHANCED_DETERMINISM", PxSceneFlag::Enum::eENABLE_ENHANCED_DETERMINISM)
            .value("eENABLE_FRICTION_EVERY_ITERATION", PxSceneFlag::Enum::eENABLE_FRICTION_EVERY_ITERATION);

    /** PhysXPhysicsManager ✅ */
    class_<PxScene>("PxScene")
            .function("setGravity", &PxScene::setGravity)  // ✅
            .function("getGravity", &PxScene::getGravity)
            .function("addActor", &PxScene::addActor, allow_raw_pointers())        // ✅
            .function("removeActor", &PxScene::removeActor, allow_raw_pointers())  // ✅
            .function("getScenePvdClient", &PxScene::getScenePvdClient, allow_raw_pointers())
            .function("getActors", &PxScene::getActors, allow_raw_pointers())
            .function("setVisualizationCullingBox", &PxScene::setVisualizationCullingBox)
            .function("collide",
                      optional_override([](PxScene &scene, PxReal elapsedTime) { scene.collide(elapsedTime); }))
            .function("fetchCollision", &PxScene::fetchCollision)
            .function("advance", optional_override([](PxScene &scene) { scene.advance(); }))
            .function("fetchResults", optional_override([](PxScene &scene, bool block) {
                          // fetchResults uses an out pointer
                          // which embind can't represent
                          // so let's override.
                          bool fetched = scene.fetchResults(block);
                          return fetched;
                      }))  // ✅
            .function("simulate", optional_override([](PxScene &scene, PxReal elapsedTime, bool controlSimulation) {
                          scene.simulate(elapsedTime, nullptr, nullptr, 0, controlSimulation);
                      }))  // ✅
            .function("raycast", &PxScene::raycast, allow_raw_pointers())
            .function("raycastAny", optional_override([](const PxScene &scene, const PxVec3 &origin,
                                                         const PxVec3 &unitDir, const PxReal distance) {
                          PxSceneQueryHit hit;
                          return PxSceneQueryExt::raycastAny(scene, origin, unitDir, distance, hit);
                      }))
            .function("raycastSingle",
                      optional_override([](const PxScene &scene, const PxVec3 &origin, const PxVec3 &unitDir,
                                           const PxReal distance, PxRaycastHit &hit,
                                           const PxSceneQueryFilterData &filterData,
                                           PxQueryFilterCallbackWrapper *callback) {
                          return PxSceneQueryExt::raycastSingle(scene, origin, unitDir, distance,
                                                                PxHitFlags(PxHitFlag::eDEFAULT), hit, filterData,
                                                                callback);
                      }),
                      allow_raw_pointers())  // ✅
            .function("sweep", &PxScene::sweep, allow_raw_pointers())
            .function("sweepAny",
                      optional_override([](const PxScene &scene, const PxGeometry &geometry, const PxTransform &pose,
                                           const PxVec3 &unitDir, const PxReal distance, PxSceneQueryFlags queryFlags) {
                          PxSweepHit hit;
                          return PxSceneQueryExt::sweepAny(scene, geometry, pose, unitDir, distance, queryFlags, hit);
                      }))
            .function("sweepSingle",
                      optional_override([](const PxScene &scene, const PxGeometry &geometry, const PxTransform &pose,
                                           const PxVec3 &unitDir, const PxReal distance, PxSceneQueryFlags outputFlags,
                                           PxSweepHit &hit) {
                          return PxSceneQueryExt::sweepSingle(scene, geometry, pose, unitDir, distance, outputFlags,
                                                              hit);
                      }))
            .function("addController", optional_override([](PxScene &scene, PxController &controller) {
                          auto actor = controller.getActor();
                          scene.addActor(*actor);
                      }))
            .function("removeController", optional_override([](PxScene &scene, PxController &controller) {
                          auto actor = controller.getActor();
                          scene.removeActor(*actor);
                      }))
            .function("createControllerManager",
                      optional_override([](PxScene &scene) { return PxCreateControllerManager(scene); }),
                      allow_raw_pointers());  // ✅

    class_<PxLocationHit>("PxLocationHit")
            .property("position", &PxLocationHit::position)
            .property("normal", &PxLocationHit::normal)
            .property("distance", &PxLocationHit::distance);
    class_<PxRaycastHit, base<PxLocationHit>>("PxRaycastHit")
            .constructor<>()
            .function("getShape", optional_override([](PxRaycastHit &block) { return block.shape; }),
                      allow_raw_pointers());
    class_<PxRaycastCallback>("PxRaycastCallback")
            .property("block", &PxRaycastCallback::block)
            .property("hasBlock", &PxRaycastCallback::hasBlock)
            .allow_subclass<PxRaycastCallbackWrapper>("PxRaycastCallbackWrapper", constructor<PxRaycastHit *, PxU32>());
    class_<PxRaycastBuffer, base<PxRaycastCallback>>("PxRaycastBuffer").constructor<>();

    function("allocateRaycastHitBuffers", &allocateRaycastHitBuffers, allow_raw_pointers());

    class_<PxSweepHit, base<PxLocationHit>>("PxSweepHit")
            .constructor<>()
            .function("getShape", optional_override([](PxSweepHit &block) { return block.shape; }),
                      allow_raw_pointers())
            .function("getActor", optional_override([](PxSweepHit &block) { return block.actor; }),
                      allow_raw_pointers());
    class_<PxSweepCallback>("PxSweepCallback")
            .property("block", &PxSweepCallback::block)
            .property("hasBlock", &PxSweepCallback::hasBlock)
            .allow_subclass<PxSweepCallbackWrapper>("PxSweepCallbackWrapper", constructor<PxSweepHit *, PxU32>());
    class_<PxSweepBuffer, base<PxSweepCallback>>("PxSweepBuffer").constructor<>();

    function("allocateSweepHitBuffers", &allocateSweepHitBuffers, allow_raw_pointers());

    class_<PxHitFlags>("PxHitFlags").constructor<int>();
    enum_<PxHitFlag::Enum>("PxHitFlag")
            .value("eDEFAULT", PxHitFlag::Enum::eDEFAULT)
            .value("eMESH_BOTH_SIDES", PxHitFlag::Enum::eMESH_BOTH_SIDES)
            .value("eMESH_MULTIPLE", PxHitFlag::Enum::eMESH_MULTIPLE);

    class_<PxQueryFilterData>("PxQueryFilterData").constructor<>().property("flags", &PxQueryFilterData::flags);
    class_<PxQueryFlags>("PxQueryFlags").constructor<int>();
    enum_<PxQueryFlag::Enum>("PxQueryFlag")
            .value("eANY_HIT", PxQueryFlag::Enum::eANY_HIT)
            .value("eDYNAMIC", PxQueryFlag::Enum::eDYNAMIC)
            .value("eSTATIC", PxQueryFlag::Enum::eSTATIC)
            .value("eNO_BLOCK", PxQueryFlag::Enum::eNO_BLOCK);
    enum_<PxQueryHitType::Enum>("PxQueryHitType")
            .value("eNONE", PxQueryHitType::Enum::eNONE)
            .value("eBLOCK", PxQueryHitType::Enum::eBLOCK)
            .value("eTOUCH", PxQueryHitType::Enum::eTOUCH);

    class_<PxQueryFilterCallback>("PxQueryFilterCallback")
            .allow_subclass<PxQueryFilterCallbackWrapper>("PxQueryFilterCallbackWrapper", constructor<>());
    class_<PxQueryCache>("PxQueryCache");

    enum_<PxCombineMode::Enum>("PxCombineMode")
            .value("eAVERAGE", PxCombineMode::Enum::eAVERAGE)
            .value("eMIN", PxCombineMode::Enum::eMIN)
            .value("eMAX", PxCombineMode::Enum::eMAX)
            .value("eMULTIPLY", PxCombineMode::Enum::eMULTIPLY);
    class_<PxMaterial>("PxMaterial")
            .function("release", &PxMaterial::release)
            .function("setDynamicFriction", &PxMaterial::setDynamicFriction)
            .function("setStaticFriction", &PxMaterial::setStaticFriction)
            .function("setRestitution", &PxMaterial::setRestitution)
            .function("setFrictionCombineMode", &PxMaterial::setFrictionCombineMode)
            .function("setRestitutionCombineMode", &PxMaterial::setRestitutionCombineMode);

    register_vector<PxMaterial *>("VectorPxMaterial");
    // setMaterials has 'PxMaterial**' as an input, which is not representable with embind
    // This is overrided to use std::vector<PxMaterial*>
    class_<PxShape>("PxShape")
            .function("release", &PxShape::release)
            .function("setContactOffset", &PxShape::setContactOffset)
            .function("getContactOffset", &PxShape::getContactOffset)
            .function("getFlags", &PxShape::getFlags)
            .function("setFlag", &PxShape::setFlag)
            .function("setFlags", &PxShape::setFlags)
            .function("setLocalPose", &PxShape::setLocalPose)
            .function("setGeometry", &PxShape::setGeometry)
            .function("getBoxGeometry", &PxShape::getBoxGeometry, allow_raw_pointers())
            .function("getSphereGeometry", &PxShape::getSphereGeometry, allow_raw_pointers())
            .function("getPlaneGeometry", &PxShape::getPlaneGeometry, allow_raw_pointers())
            .function("getCapsuleGeometry", &PxShape::getCapsuleGeometry, allow_raw_pointers())
            .function("setSimulationFilterData", &PxShape::setSimulationFilterData, allow_raw_pointers())
            .function("setQueryFilterData", &PxShape::setQueryFilterData)
            .function("getQueryFilterData", &PxShape::getQueryFilterData, allow_raw_pointers())
            .function("setMaterials", optional_override([](PxShape &shape, std::vector<PxMaterial *> materials) {
                          return shape.setMaterials(materials.data(), materials.size());
                      }));

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

    class_<PxShapeFlags>("PxShapeFlags").constructor<int>().function("isSet", &PxShapeFlags::isSet);
    enum_<PxShapeFlag::Enum>("PxShapeFlag")
            .value("eSIMULATION_SHAPE", PxShapeFlag::Enum::eSIMULATION_SHAPE)
            .value("eSCENE_QUERY_SHAPE", PxShapeFlag::Enum::eSCENE_QUERY_SHAPE)
            .value("eTRIGGER_SHAPE", PxShapeFlag::Enum::eTRIGGER_SHAPE)
            .value("eVISUALIZATION", PxShapeFlag::Enum::eVISUALIZATION);

    enum_<PxActorFlag::Enum>("PxActorFlag").value("eDISABLE_GRAVITY", PxActorFlag::Enum::eDISABLE_GRAVITY);

    class_<PxErrorCallback>("PxErrorCallback");
    class_<PxDefaultErrorCallback, base<PxErrorCallback>>("PxDefaultErrorCallback").constructor<>();

    class_<PxBVHStructure>("PxBVHStructure");

    class_<PxFilterData>("PxFilterData")
            .constructor<PxU32, PxU32, PxU32, PxU32>()
            .property("word0", &PxFilterData::word0)
            .property("word1", &PxFilterData::word1)
            .property("word2", &PxFilterData::word2)
            .property("word3", &PxFilterData::word3);
    class_<PxPairFlags>("PxPairFlags");
    class_<PxFilterFlags>("PxFilterFlags");
    enum_<PxPairFlag::Enum>("PxPairFlag");
    enum_<PxFilterFlag::Enum>("PxFilterFlag");

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
                      allow_raw_pointers())
            .function("setQueryFilterData", optional_override([](PxRigidActor &actor, PxFilterData &data) {
                          PxShape *shape;
                          actor.getShapes(&shape, 1);
                          shape->setQueryFilterData(data);
                      }))
            .function("getQueryFilterData", optional_override([](PxRigidActor &actor, PxFilterData &data) {
                          PxShape *shape;
                          actor.getShapes(&shape, 1);
                          return shape->getQueryFilterData();
                      }));
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

    /** PhysXColliderShape ✅ */
    class_<PxGeometry>("PxGeometry");
    /** PhysXBoxColliderShape ✅ */
    class_<PxBoxGeometry, base<PxGeometry>>("PxBoxGeometry")
            .constructor<>()
            .constructor<float, float, float>()
            .function("isValid", &PxBoxGeometry::isValid)
            .property("halfExtents", &PxBoxGeometry::halfExtents);  // ✅
    /** PhysXSphereColliderShape ✅ */
    class_<PxSphereGeometry, base<PxGeometry>>("PxSphereGeometry")
            .constructor<>()
            .constructor<float>()
            .property("radius", &PxSphereGeometry::radius)  // ✅
            .function("isValid", &PxSphereGeometry::isValid);
    /** PhysXCapsuleColliderShape ✅ */
    class_<PxCapsuleGeometry, base<PxGeometry>>("PxCapsuleGeometry")
            .constructor<float, float>()
            .property("radius", &PxCapsuleGeometry::radius)          // ✅
            .property("halfHeight", &PxCapsuleGeometry::halfHeight)  // ✅
            .function("isValid", &PxCapsuleGeometry::isValid);
    /** PhysXCapsuleColliderShape ✅ */
    class_<PxPlaneGeometry, base<PxGeometry>>("PxPlaneGeometry").constructor<>();

    class_<PxPlane>("PxPlane").constructor<float, float, float, float>();

    enum_<PxCapsuleClimbingMode::Enum>("PxCapsuleClimbingMode")
            .value("eCONSTRAINED", PxCapsuleClimbingMode::Enum::eCONSTRAINED)
            .value("eLAST", PxCapsuleClimbingMode::Enum::eLAST)
            .value("eEASY", PxCapsuleClimbingMode::Enum::eEASY);
    enum_<PxControllerShapeType::Enum>("PxControllerShapeType")
            .value("eBOX", PxControllerShapeType::Enum::eBOX)
            .value("eCAPSULE", PxControllerShapeType::Enum::eCAPSULE);
    enum_<PxControllerNonWalkableMode::Enum>("PxControllerNonWalkableMode")
            .value("ePREVENT_CLIMBING", PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING)
            .value("ePREVENT_CLIMBING_AND_FORCE_SLIDING",
                   PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING_AND_FORCE_SLIDING);
    enum_<PxControllerCollisionFlag::Enum>("PxControllerCollisionFlag")
            .value("eCOLLISION_SIDES", PxControllerCollisionFlag::Enum::eCOLLISION_SIDES)
            .value("eCOLLISION_UP", PxControllerCollisionFlag::Enum::eCOLLISION_UP)
            .value("eCOLLISION_DOWN", PxControllerCollisionFlag::Enum::eCOLLISION_DOWN);
    class_<PxControllerCollisionFlags>("PxControllerCollisionFlags")
            .constructor<int>()
            .function("isSet", &PxControllerCollisionFlags::isSet);
    /** PhysXCharacterControllerManager ✅ */
    class_<PxControllerManager>("PxControllerManager")
            .function("purgeControllers", &PxControllerManager::purgeControllers)                              // ✅
            .function("createController", &PxControllerManager::createController, allow_raw_pointers())        // ✅
            .function("computeInteractions", &PxControllerManager::computeInteractions, allow_raw_pointers())  // ✅
            .function("setTessellation", &PxControllerManager::setTessellation)                                // ✅
            .function("setOverlapRecoveryModule", &PxControllerManager::setOverlapRecoveryModule)              // ✅
            .function("setPreciseSweeps", &PxControllerManager::setPreciseSweeps)                              // ✅
            .function("setPreventVerticalSlidingAgainstCeiling",
                      &PxControllerManager::setPreventVerticalSlidingAgainstCeiling)  // ✅
            .function("shiftOrigin", &PxControllerManager::shiftOrigin);              // ✅
    /** PhysXCharacterControllerDesc ✅ */
    class_<PxControllerDesc>("PxControllerDesc")
            .function("isValid", &PxControllerDesc::isValid)
            .function("getType", &PxControllerDesc::getType)                          // ✅
            .property("position", &PxControllerDesc::position)                        // ✅
            .property("upDirection", &PxControllerDesc::upDirection)                  // ✅
            .property("slopeLimit", &PxControllerDesc::slopeLimit)                    // ✅
            .property("invisibleWallHeight", &PxControllerDesc::invisibleWallHeight)  // ✅
            .property("maxJumpHeight", &PxControllerDesc::maxJumpHeight)              // ✅
            .property("contactOffset", &PxControllerDesc::contactOffset)              // ✅
            .property("stepOffset", &PxControllerDesc::stepOffset)                    // ✅
            .property("density", &PxControllerDesc::density)                          // ✅
            .property("scaleCoeff", &PxControllerDesc::scaleCoeff)                    // ✅
            .property("volumeGrowth", &PxControllerDesc::volumeGrowth)                // ✅
            .function("setNonWalkableMode", optional_override([](PxControllerDesc &desc, int mode) {
                          return desc.nonWalkableMode = PxControllerNonWalkableMode::Enum(mode);
                      }))  // ✅
            .function("setMaterial", optional_override([](PxControllerDesc &desc, PxMaterial *material) {
                          return desc.material = material;
                      }),
                      allow_raw_pointers())                                                      // ✅
            .property("registerDeletionListener", &PxControllerDesc::registerDeletionListener);  // ✅
    /** PhysXCapsuleCharacterControllerDesc ✅ */
    class_<PxCapsuleControllerDesc, base<PxControllerDesc>>("PxCapsuleControllerDesc")
            .constructor<>()
            .function("isValid", &PxCapsuleControllerDesc::isValid)
            .function("setToDefault", &PxCapsuleControllerDesc::setToDefault)   // ✅
            .property("radius", &PxCapsuleControllerDesc::radius)               // ✅
            .property("height", &PxCapsuleControllerDesc::height)               // ✅
            .property("climbingMode", &PxCapsuleControllerDesc::climbingMode);  // ✅
    /** PhysXBoxCharacterControllerDesc ✅ */
    class_<PxBoxControllerDesc, base<PxControllerDesc>>("PxBoxControllerDesc")
            .constructor<>()
            .function("isValid", &PxBoxControllerDesc::isValid)
            .function("setToDefault", &PxBoxControllerDesc::setToDefault)            // ✅
            .property("halfForwardExtent", &PxBoxControllerDesc::halfForwardExtent)  // ✅
            .property("halfHeight", &PxBoxControllerDesc::halfHeight)                // ✅
            .property("halfSideExtent", &PxBoxControllerDesc::halfSideExtent);       // ✅

    /** PhysXCharacterController ✅ */
    class_<PxController>("PxController")
            .function("release", &PxController::release)
            .function("isSetControllerCollisionFlag",
                      optional_override([](PxController &controller, PxControllerCollisionFlags &flags, int flag) {
                          return flags.isSet(PxControllerCollisionFlag::Enum(flag));
                      }))  // ✅
            .function("move", optional_override([](PxController &controller, const PxVec3 &disp, PxF32 minDist,
                                                   PxF32 elapsedTime) -> uint32_t {
                          return controller.move(disp, minDist, elapsedTime, PxControllerFilters());
                      }))                                         // ✅
            .function("setPosition", &PxController::setPosition)  // ✅
            .function("getPosition", &PxController::getPosition)
            .function("setFootPosition", &PxController::setFootPosition)  // ✅
            .function("getFootPosition", &PxController::getFootPosition)  // ✅
            .function("setStepOffset", &PxController::setStepOffset)      // ✅
            .function("setNonWalkableMode", optional_override([](PxController &controller, int mode) {
                          return controller.setNonWalkableMode(PxControllerNonWalkableMode::Enum(mode));
                      }))                                                   // ✅
            .function("setContactOffset", &PxController::setContactOffset)  // ✅
            .function("setUpDirection", &PxController::setUpDirection)      // ✅
            .function("setSlopeLimit", &PxController::setSlopeLimit)        // ✅
            .function("invalidateCache", &PxController::invalidateCache)    // ✅
            .function("resize", &PxController::resize)                      // ✅
            .function("setQueryFilterData", optional_override([](PxController &ctrl, PxFilterData &data) {
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape;
                          actor->getShapes(&shape, 1);
                          shape->setQueryFilterData(data);
                      }))  // ✅
            .function("getQueryFilterData", optional_override([](PxController &ctrl, PxFilterData &data) {
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape;
                          actor->getShapes(&shape, 1);
                          return shape->getQueryFilterData();
                      }))
            /** PxCapsuleController ✅ */
            .function("setRadius", optional_override([](PxController &ctrl, PxF32 radius) {
                          static_cast<PxCapsuleController *>(&ctrl)->setRadius(radius);
                      }))  // ✅
            .function("setHeight", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxCapsuleController *>(&ctrl)->setHeight(height);
                      }))  // ✅
            .function("setClimbingMode", optional_override([](PxController &ctrl, int mode) {
                          return static_cast<PxCapsuleController *>(&ctrl)->setClimbingMode(
                                  PxCapsuleClimbingMode::Enum(mode));
                      }))  // ✅
                           /** PxBoxController ✅ */
            .function("setHalfHeight", optional_override([](PxController &ctrl, PxF32 radius) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfHeight(radius);
                      }))  // ✅
            .function("setHalfSideExtent", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfSideExtent(height);
                      }))  // ✅
            .function("setHalfForwardExtent", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfForwardExtent(height);
                      }));  // ✅
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxPvd>(PxPvd *) { /* do nothing */
}

#if PX_DEBUG || PX_PROFILE || PX_CHECKED
template <>
void raw_destructor<PxPvdTransport>(PxPvdTransport *) { /* do nothing */
}

template <>
void raw_destructor<PxPvdSceneClient>(PxPvdSceneClient *) { /* do nothing */
}

#endif

// Physx uses private destructors all over the place for its own reference counting
// embind doesn't deal with this well, so we have to override the destructors to keep them private
// in the bindings
// See: https://github.com/emscripten-core/emscripten/issues/5587
template <>
void raw_destructor<PxFoundation>(PxFoundation *) { /* do nothing */
}

template <>
void raw_destructor<PxMaterial>(PxMaterial *) { /* do nothing */
}

template <>
void raw_destructor<PxScene>(PxScene *) { /* do nothing */
}

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
void raw_destructor<PxShape>(PxShape *) { /* do nothing */
}

template <>
void raw_destructor<PxBVHStructure>(PxBVHStructure *) { /* do nothing */
}

template <>
void raw_destructor<PxRigidStatic>(PxRigidStatic *) { /* do nothing */
}

template <>
void raw_destructor<PxJoint>(PxJoint *) { /* do nothing */
}

template <>
void raw_destructor<PxController>(PxController *) { /* do nothing */
}

template <>
void raw_destructor<PxCapsuleController>(PxCapsuleController *) { /* do nothing */
}

template <>
void raw_destructor<PxControllerDesc>(PxControllerDesc *) { /* do nothing */
}

template <>
void raw_destructor<PxControllerManager>(PxControllerManager *) { /* do nothing */
}
}  // namespace internal
}  // namespace emscripten
