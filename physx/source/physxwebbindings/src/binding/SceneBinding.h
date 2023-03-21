//
// Created by Oasis on 2023/3/21.
//

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

    void onConstraintBreak(PxConstraintInfo *, PxU32) override {}

    void onWake(PxActor **, PxU32) override {}

    void onSleep(PxActor **, PxU32) override {}

    void onContact(const PxContactPairHeader &, const PxContactPair *pairs, PxU32 nbPairs) override {
        for (PxU32 i = 0; i < nbPairs; i++) {
            const PxContactPair &cp = pairs[i];

            if (cp.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_CCD)) {
                call<void>("onContactBegin", getUUID(cp.shapes[0]), getUUID(cp.shapes[1]));
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                if (!cp.flags.isSet(PxContactPairFlag::Enum::eREMOVED_SHAPE_0) &&
                    !cp.flags.isSet(PxContactPairFlag::Enum::eREMOVED_SHAPE_1)) {
                    call<void>("onContactEnd", getUUID(cp.shapes[0]), getUUID(cp.shapes[1]));
                }
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
                call<void>("onContactPersist", getUUID(cp.shapes[0]), getUUID(cp.shapes[1]));
            }
        }
    }

    void onTrigger(PxTriggerPair *pairs, PxU32 count) override {
        for (PxU32 i = 0; i < count; i++) {
            const PxTriggerPair &tp = pairs[i];

            if (tp.status & PxPairFlag::eNOTIFY_TOUCH_FOUND) {
                call<void>("onTriggerBegin", getUUID(tp.triggerShape), getUUID(tp.otherShape));
            } else if (tp.status & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                if (!tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_OTHER) &&
                    !tp.flags.isSet(PxTriggerPairFlag::Enum::eREMOVED_SHAPE_TRIGGER)) {
                    call<void>("onTriggerEnd", getUUID(tp.triggerShape), getUUID(tp.otherShape));
                }
            }
        }
    }

    void onAdvance(const PxRigidBody *const *, const PxTransform *, const PxU32) override {}
};

PxSceneDesc *getDefaultSceneDesc(PxTolerancesScale &scale, int numThreads, PxSimulationEventCallback *callback) {
    auto *sceneDesc = new PxSceneDesc(scale);
    sceneDesc->gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc->cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    sceneDesc->filterShader = physx::simulationFilterShader;
    sceneDesc->simulationEventCallback = callback;
    return sceneDesc;
}

EMSCRIPTEN_BINDINGS(physx_scene) {
    function("getDefaultSceneDesc", &getDefaultSceneDesc, allow_raw_pointers());
    function("PxDefaultSimulationFilterShader", &PxDefaultSimulationFilterShader, allow_raw_pointers());

    class_<PxSimulationEventCallback>("PxSimulationEventCallback")
            .allow_subclass<PxSimulationEventCallbackWrapper>("PxSimulationEventCallbackWrapper");

    class_<PxSceneDesc>("PxSceneDesc").constructor<PxTolerancesScale>().property("gravity", &PxSceneDesc::gravity);

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
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxScene>(PxScene *) { /* do nothing */
}

}  // namespace internal
}  // namespace emscripten