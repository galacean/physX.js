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
        return (PxQueryHitType::Enum)call<int>("preFilter", filterData, getUUID(shape), actor);
    }
};