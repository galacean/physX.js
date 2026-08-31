
#pragma once

#include "../SimulationFilterShader.h"
#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten.h>
#include <emscripten/bind.h>

using namespace physx;
using namespace emscripten;

template <typename HitType>
struct PxQueryCallbackCollector : PxHitCallback<HitType> {
    std::vector<HitType> hits;

    PxQueryCallbackCollector(HitType *touches, PxU32 maxNbTouches)
        : PxHitCallback<HitType>(touches, maxNbTouches) {}

    PxAgain processTouches(const HitType *buffer, PxU32 nbHits) override {
        hits.insert(hits.end(), buffer, buffer + nbHits);
        return true;
    }
};

using PxRaycastCallbackCollector = PxQueryCallbackCollector<PxRaycastHit>;
using PxOverlapCallbackCollector = PxQueryCallbackCollector<PxOverlapHit>;

//----------------------------------------------------------------------------------------------------------------------
// struct PxSweepCallbackWrapper : public wrapper<PxSweepCallback> {
//     EMSCRIPTEN_WRAPPER(explicit PxSweepCallbackWrapper)

//     PxAgain processTouches(const PxSweepHit *buffer, PxU32 nbHits) override {
//         for (PxU32 i = 0; i < nbHits; i++) {
//             bool again = call<PxAgain>("processTouches", buffer[i]);
//             if (!again) {
//                 return false;
//             }
//         }
//         return true;
//     }
// };

// PxSweepHit *allocateSweepHitBuffers(PxU32 nb) {
//     auto *myArray = new PxSweepHit[nb];
//     return myArray;
// }

//----------------------------------------------------------------------------------------------------------------------
struct PxQueryFilterCallbackWrapper : public wrapper<PxQueryFilterCallback> {
    EMSCRIPTEN_WRAPPER(explicit PxQueryFilterCallbackWrapper)

    PxQueryHitType::Enum postFilter(const PxFilterData &filterData, const PxQueryHit &hit) override {
        const PxLocationHit &locationHit = static_cast<const PxLocationHit &>(hit);
        return (PxQueryHitType::Enum)call<int>("postFilter", filterData, locationHit.distance);
    }

    PxQueryHitType::Enum preFilter(const PxFilterData &filterData,
                                   const PxShape *shape,
                                   const PxRigidActor *actor,
                                   PxHitFlags &) override {
        return (PxQueryHitType::Enum)call<int>("preFilter", filterData, getUUID(shape), getUUID(actor));
    }
};
