//
// Created by Oasis on 2023/3/21.
//

#pragma once

#include "../SimulationFilterShader.h"
#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten.h>
#include <emscripten/bind.h>

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
    sceneDesc->filterShader =  physx::simulationFilterShader;
    sceneDesc->simulationEventCallback = callback;
    return sceneDesc;
}