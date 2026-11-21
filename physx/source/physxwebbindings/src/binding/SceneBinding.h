
#pragma once

#include <emscripten.h>
#include <emscripten/bind.h>

#include "../SimulationFilterShader.h"
#include "PxPhysicsAPI.h"
#include "QueryBinding.h"

using namespace physx;
using namespace emscripten;

struct PxSimulationEventCallbackWrapper : public wrapper<PxSimulationEventCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxSimulationEventCallbackWrapper)

    void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) override {
        for (PxU32 i = 0; i < count; ++i) {
            const PxConstraintInfo &info = constraints[i];
            if (info.type == PxConstraintExtIDs::eJOINT) {
                const auto *joint = static_cast<const PxJoint *>(info.externalReference);
                const auto id = static_cast<PxU32>(reinterpret_cast<uintptr_t>(joint->userData));
                if (id) call<void>("onJointBreak", id);
            }
        }
    }

    void onWake(PxActor **, PxU32) override {}

    void onSleep(PxActor **, PxU32) override {}

    void onContact(const PxContactPairHeader &header, const PxContactPair *pairs, PxU32 nbPairs) override {
        if (header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_0) ||
            header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_1)) {
            return;
        }

        // One extra-data item set covers the shape pairs starting at its reported index.
        PxContactPairExtraDataIterator extra(header.extraDataStream, header.extraDataStreamSize);
        bool hasNext = extra.nextItemSet();
        const PxContactPairVelocity *preSolver = nullptr;
        for (PxU32 i = 0; i < nbPairs; i++) {
            while (hasNext && extra.contactPairIndex == i) {
                preSolver = extra.preSolverVelocity;
                hasNext = extra.nextItemSet();
            }
            const PxContactPair &cp = pairs[i];
            if (cp.flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_0) ||
                cp.flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_1) || !cp.shapes[0] || !cp.shapes[1]) {
                continue;
            }
            contactPoints.resize(cp.contactCount);
            const PxU32 count = cp.extractContacts(contactPoints.data(), cp.contactCount);
            const PxU32 velocityOffset = count * 10;
            // PhysX 4.1 clears the swept solver's velocity snapshot before it builds the report.
            const bool hasVelocity = preSolver && !cp.events.isSet(PxPairFlag::eNOTIFY_TOUCH_CCD);
            contactValues.resize(velocityOffset + (hasVelocity ? 3 : 0));
            for (PxU32 j = 0; j < count; ++j) {
                const PxContactPairPoint &point = contactPoints[j];
                float *out = contactValues.data() + j * 10;
                out[0] = point.position.x; out[1] = point.position.y; out[2] = point.position.z;
                out[3] = point.normal.x; out[4] = point.normal.y; out[5] = point.normal.z;
                out[6] = point.impulse.x; out[7] = point.impulse.y; out[8] = point.impulse.z;
                out[9] = point.separation;
            }
            if (hasVelocity) {
                const PxVec3 velocity = preSolver->linearVelocity[1] - preSolver->linearVelocity[0];
                contactValues[velocityOffset] = velocity.x;
                contactValues[velocityOffset + 1] = velocity.y;
                contactValues[velocityOffset + 2] = velocity.z;
            }
            const PxU32 shape0ID = getUUID(cp.shapes[0]);
            const PxU32 shape1ID = getUUID(cp.shapes[1]);
            const val contacts(typed_memory_view(contactValues.size(), contactValues.data()));
            const bool isFound = cp.events.isSet(PxPairFlag::eNOTIFY_TOUCH_FOUND);
            const bool isPersists = cp.events.isSet(PxPairFlag::eNOTIFY_TOUCH_PERSISTS);
            const bool isLost = cp.events.isSet(PxPairFlag::eNOTIFY_TOUCH_LOST);
            const bool isPureCCDRetouch =
                !isFound && !isPersists && !isLost && cp.events.isSet(PxPairFlag::eNOTIFY_TOUCH_CCD);
            if (isFound || isPureCCDRetouch) {
                call<void>("onContactBegin", shape0ID, shape1ID, count, contacts);
            }
            if (isPersists) {
                call<void>("onContactPersist", shape0ID, shape1ID, count, contacts);
            }
            if (isLost) {
                call<void>("onContactEnd", shape0ID, shape1ID, count, contacts);
            }
        }
    }

    void onTrigger(PxTriggerPair *pairs, PxU32 count) override {
        for (PxU32 i = 0; i < count; i++) {
            const PxTriggerPair &tp = pairs[i];
            if (tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_OTHER) ||
                tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_TRIGGER) || !tp.triggerShape || !tp.otherShape) {
                continue;
            }

            if (tp.status & PxPairFlag::eNOTIFY_TOUCH_FOUND) {
                call<void>("onTriggerBegin", getUUID(tp.triggerShape), getUUID(tp.otherShape));
            } else if (tp.status & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                call<void>("onTriggerEnd", getUUID(tp.triggerShape), getUUID(tp.otherShape));
            }
        }
    }

    void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) override {}

private:
    std::vector<PxContactPairPoint> contactPoints;
    std::vector<float> contactValues;
};

inline void releaseDefaultCpuDispatcher(PxCpuDispatcher *dispatcher) {
    static_cast<PxDefaultCpuDispatcher *>(dispatcher)->release();
}

inline PxScene *createSceneWithOwnedDispatcher(PxPhysics &physics, const PxSceneDesc &desc) {
    const PxTolerancesScale &physicsScale = physics.getTolerancesScale();
    const PxTolerancesScale &descScale = desc.getTolerancesScale();
    if (!desc.filterShader || descScale.length != physicsScale.length || descScale.speed != physicsScale.speed) {
        return nullptr;
    }

    PxSceneDesc descCopy = desc;
    PxDefaultCpuDispatcher *ownedDispatcher = nullptr;
    if (!descCopy.cpuDispatcher) {
        const PxU32 numThreads = static_cast<PxU32>(reinterpret_cast<uintptr_t>(descCopy.userData));
        descCopy.userData = nullptr;
        ownedDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
        descCopy.cpuDispatcher = ownedDispatcher;
    }

    PxScene *scene = descCopy.isValid() ? physics.createScene(descCopy) : nullptr;
    if (!scene) {
        if (ownedDispatcher) {
            ownedDispatcher->release();
        }
        return nullptr;
    }
    if (ownedDispatcher) {
        PxSetSceneOwnedCpuDispatcher(*scene, ownedDispatcher, &releaseDefaultCpuDispatcher);
    }
    return scene;
}

PxSceneDesc *getDefaultSceneDesc(PxTolerancesScale &scale, int numThreads, PxSimulationEventCallback *callback) {
    auto *sceneDesc = new PxSceneDesc(scale);
    sceneDesc->gravity = PxVec3(0.0f, -9.81f, 0.0f);
    // Defer dispatcher allocation to createSceneWithOwnedDispatcher so each scene owns its own dispatcher even when
    // a single PxSceneDesc is reused across multiple createScene calls or discarded unused.
    sceneDesc->cpuDispatcher = nullptr;
    sceneDesc->userData = reinterpret_cast<void *>(static_cast<uintptr_t>(numThreads > 0 ? numThreads : 0));
    sceneDesc->filterShader = physx::simulationFilterShader;
    sceneDesc->simulationEventCallback = callback;
    sceneDesc->flags |= PxSceneFlag::eENABLE_CCD;
    return sceneDesc;
}

inline bool getGroupCollisionFlagBinding(const PxScene &scene, double group1, double group2) {
    PxU32 g1 = 0;
    PxU32 g2 = 0;
    if (!tryParseCollisionGroup(group1, g1) || !tryParseCollisionGroup(group2, g2)) {
        return false;
    }
    return physx::getGroupCollisionFlag(scene, g1, g2);
}

inline void setGroupCollisionFlagBinding(PxScene &scene, double group1, double group2, bool enable) {
    PxU32 g1 = 0;
    PxU32 g2 = 0;
    if (tryParseCollisionGroup(group1, g1) && tryParseCollisionGroup(group2, g2)) {
        physx::setGroupCollisionFlag(scene, g1, g2, enable);
    }
}

inline PxU16 getGroupBinding(const PxActor *actor) {
    return actor ? physx::getGroup(*actor) : 0;
}

inline void setGroupBinding(PxActor *actor, double collisionGroup) {
    PxU32 group = 0;
    if (!actor || !tryParseCollisionGroup(collisionGroup, group)) {
        return;
    }
    physx::setGroup(*actor, group);
}

EMSCRIPTEN_BINDINGS(physx_scene) {
    function("getDefaultSceneDesc", &getDefaultSceneDesc, allow_raw_pointers()); // ✅
    // function("PxDefaultSimulationFilterShader", &PxDefaultSimulationFilterShader, allow_raw_pointers());

    function("getGroup", &getGroupBinding, allow_raw_pointers());
    function("setGroup", &setGroupBinding, allow_raw_pointers());

    class_<PxSimulationEventCallback>("PxSimulationEventCallback")
            .allow_subclass<PxSimulationEventCallbackWrapper>("PxSimulationEventCallbackWrapper"); // ✅

    class_<PxSceneDesc>("PxSceneDesc")
            .constructor<PxTolerancesScale>()
            .property("gravity", &PxSceneDesc::gravity)
            .property("bounceThresholdVelocity", &PxSceneDesc::bounceThresholdVelocity)
            .function("setSolverType", optional_override([](PxSceneDesc &desc, bool useTgs) {
                          desc.solverType = useTgs ? PxSolverType::eTGS : PxSolverType::ePGS;
                      }))
            .function("setKinematicPairs", optional_override([](PxSceneDesc &desc, bool kineKine, bool staticKine) {
                          desc.kineKineFilteringMode = kineKine ? PxPairFilteringMode::eKEEP : PxPairFilteringMode::eDEFAULT;
                          desc.staticKineFilteringMode = staticKine ? PxPairFilteringMode::eKEEP : PxPairFilteringMode::eDEFAULT;
                      }));

    // class_<PxSceneFlags>("PxSceneFlags");
    // enum_<PxSceneFlag::Enum>("PxSceneFlag")
    //         .value("eENABLE_ACTIVE_ACTORS ", PxSceneFlag::Enum::eENABLE_ACTIVE_ACTORS)
    //         .value("eENABLE_CCD", PxSceneFlag::Enum::eENABLE_CCD)
    //         .value("eDISABLE_CCD_RESWEEP", PxSceneFlag::Enum::eDISABLE_CCD_RESWEEP)
    //         .value("eADAPTIVE_FORCE", PxSceneFlag::Enum::eADAPTIVE_FORCE)
    //         .value("eENABLE_PCM", PxSceneFlag::Enum::eENABLE_PCM)
    //         .value("eDISABLE_CONTACT_REPORT_BUFFER_RESIZE", PxSceneFlag::Enum::eDISABLE_CONTACT_REPORT_BUFFER_RESIZE)
    //         .value("eDISABLE_CONTACT_CACHE", PxSceneFlag::Enum::eDISABLE_CONTACT_CACHE)
    //         .value("eREQUIRE_RW_LOCK", PxSceneFlag::Enum::eREQUIRE_RW_LOCK)
    //         .value("eENABLE_STABILIZATION", PxSceneFlag::Enum::eENABLE_STABILIZATION)
    //         .value("eENABLE_AVERAGE_POINT", PxSceneFlag::Enum::eENABLE_AVERAGE_POINT)
    //         .value("eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS", PxSceneFlag::Enum::eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS)
    //         .value("eENABLE_ENHANCED_DETERMINISM", PxSceneFlag::Enum::eENABLE_ENHANCED_DETERMINISM)
    //         .value("eENABLE_FRICTION_EVERY_ITERATION", PxSceneFlag::Enum::eENABLE_FRICTION_EVERY_ITERATION);

    /** PhysXPhysicsManager ✅ */
    class_<PxScene>("PxScene")
            .function("setContactEventEnabled", &PxSetSceneContactEventsEnabled)
            .function("getGroupCollisionFlag", &getGroupCollisionFlagBinding)
            .function("setGroupCollisionFlag", &setGroupCollisionFlagBinding)
            .function("setPairIgnored", &physx::setPairIgnored)
            .function("removeIgnoredPairs", &physx::removeIgnoredPairs)
            .function("setGravity", &PxScene::setGravity)  // ✅
            // .function("getGravity", &PxScene::getGravity)
            .function("addActor", &PxScene::addActor, allow_raw_pointers())        // ✅
            .function("removeActor", &PxScene::removeActor, allow_raw_pointers())  // ✅
            // .function("getScenePvdClient", &PxScene::getScenePvdClient, allow_raw_pointers())
            // .function("getActors", &PxScene::getActors, allow_raw_pointers())
            // .function("setVisualizationCullingBox", &PxScene::setVisualizationCullingBox)
            // .function("collide",
            //           optional_override([](PxScene &scene, PxReal elapsedTime) { scene.collide(elapsedTime); }))
            // .function("fetchCollision", &PxScene::fetchCollision)
            // .function("advance", optional_override([](PxScene &scene) { scene.advance(); }))
            .function("fetchResults", optional_override([](PxScene &scene, bool block) {
                          // fetchResults uses an out pointer
                          // which embind can't represent
                          // so let's override.
                          bool fetched = scene.fetchResults(block);
                          return fetched;
                      }))  // ✅
            .function("simulate", optional_override([](PxScene &scene, PxReal elapsedTime, bool controlSimulation) {
                          if (PxIsFinite(elapsedTime) && elapsedTime > 0.0f) {
                              scene.simulate(elapsedTime, nullptr, nullptr, 0, controlSimulation);
                          }
                      }))  // ✅
            .function("createControllerManager",
                      optional_override([](PxScene &scene) { return PxCreateControllerManager(scene); }),
                      allow_raw_pointers())  // ✅
            .function("release", &PxScene::release); // ✅
    class_<PxSceneQueries>("PxSceneQueries")
            .constructor(optional_override([](PxScene *scene) { return new PxSceneQueries(scene); }), allow_raw_pointers())
            .function("raycast", &PxSceneQueries::raycast)
            .function("raycastAll", &PxSceneQueries::raycastAll)
            .function("sweep", &PxSceneQueries::sweep)
            .function("overlap", &PxSceneQueries::overlap);
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxScene>(PxScene *) { /* do nothing */
}

}  // namespace internal
}  // namespace emscripten
