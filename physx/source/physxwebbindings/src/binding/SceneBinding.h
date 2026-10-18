
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

    void setContactEventEnabled(bool enabled) { contactEventEnabled = enabled; }

    void onConstraintBreak(PxConstraintInfo *, PxU32) override {}

    void onWake(PxActor **, PxU32) override {}

    void onSleep(PxActor **, PxU32) override {}

    void onContact(const PxContactPairHeader &header, const PxContactPair *pairs, PxU32 nbPairs) override {
        if (!contactEventEnabled || header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_0) ||
            header.flags.isSet(PxContactPairHeaderFlag::eREMOVED_ACTOR_1)) {
            return;
        }

        for (PxU32 i = 0; i < nbPairs; i++) {
            const PxContactPair &cp = pairs[i];
            if (cp.flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_0) ||
                cp.flags.isSet(PxContactPairFlag::eREMOVED_SHAPE_1)) {
                continue;
            }
            contactPoints.resize(cp.contactCount);
            const PxU32 count = cp.extractContacts(contactPoints.data(), cp.contactCount);
            contactValues.resize(count * 10);
            for (PxU32 j = 0; j < count; ++j) {
                const PxContactPairPoint &point = contactPoints[j];
                float *out = contactValues.data() + j * 10;
                out[0] = point.position.x; out[1] = point.position.y; out[2] = point.position.z;
                out[3] = point.normal.x; out[4] = point.normal.y; out[5] = point.normal.z;
                out[6] = point.impulse.x; out[7] = point.impulse.y; out[8] = point.impulse.z;
                out[9] = point.separation;
            }
            const PxU32 shape0ID = getUUID(cp.shapes[0]);
            const PxU32 shape1ID = getUUID(cp.shapes[1]);
            const val contacts(typed_memory_view(contactValues.size(), contactValues.data()));
            if (cp.events & (PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_CCD)) {
                call<void>("onContactBegin", shape0ID, shape1ID, contacts);
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_LOST) {
                call<void>("onContactEnd", shape0ID, shape1ID, contacts);
            } else if (cp.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS) {
                call<void>("onContactPersist", shape0ID, shape1ID, contacts);
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

private:
    bool contactEventEnabled = true;
    std::vector<PxContactPairPoint> contactPoints;
    std::vector<float> contactValues;
};

void setContactEventEnabled(PxSimulationEventCallback *callback, bool enabled) {
    static_cast<PxSimulationEventCallbackWrapper *>(callback)->setContactEventEnabled(enabled);
}

PxSceneDesc *getDefaultSceneDesc(PxTolerancesScale &scale, int numThreads, PxSimulationEventCallback *callback) {
    auto *sceneDesc = new PxSceneDesc(scale);
    sceneDesc->gravity = PxVec3(0.0f, -9.81f, 0.0f);
    sceneDesc->cpuDispatcher = PxDefaultCpuDispatcherCreate(numThreads);
    sceneDesc->filterShader = physx::simulationFilterShader;
    sceneDesc->simulationEventCallback = callback;
    sceneDesc->kineKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc->staticKineFilteringMode = PxPairFilteringMode::eKEEP;
    sceneDesc->flags |= PxSceneFlag::eENABLE_CCD;
    return sceneDesc;
}

EMSCRIPTEN_BINDINGS(physx_scene) {
    function("getDefaultSceneDesc", &getDefaultSceneDesc, allow_raw_pointers()); // ✅
    function("setContactEventEnabled", &setContactEventEnabled, allow_raw_pointers());
    // function("PxDefaultSimulationFilterShader", &PxDefaultSimulationFilterShader, allow_raw_pointers());

    function("getGroupCollisionFlag", &physx::getGroupCollisionFlag);
    function("setGroupCollisionFlag", &physx::setGroupCollisionFlag);
    function("getGroup", &physx::getGroup, allow_raw_pointers());
    function("setGroup", &physx::setGroup, allow_raw_pointers());

    // GroupsMask filter
    // function("getGroupsMask", &physx::getGroupsMask, allow_raw_pointers());
    // function("setGroupsMask", &physx::setGroupsMask, allow_raw_pointers());
    // function("getFilterOps", &physx::getFilterOps);
    // function("setFilterOps", &physx::setFilterOps);
    // function("getFilterBool", &physx::getFilterBool);
    // function("setFilterBool", &physx::setFilterBool);
    // function("getFilterConstants", &physx::getFilterConstants);
    // function("setFilterConstants", &physx::setFilterConstants);

    class_<PxSimulationEventCallback>("PxSimulationEventCallback")
            .allow_subclass<PxSimulationEventCallbackWrapper>("PxSimulationEventCallbackWrapper"); // ✅

    class_<PxSceneDesc>("PxSceneDesc").constructor<PxTolerancesScale>().property("gravity", &PxSceneDesc::gravity); // ✅

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
                          scene.simulate(elapsedTime, nullptr, nullptr, 0, controlSimulation);
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
