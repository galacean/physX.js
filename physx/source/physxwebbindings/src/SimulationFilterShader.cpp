//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "SimulationFilterShader.h"

#include "../../common/src/CmPhysXCommon.h"
#include "PsAllocator.h"
#include "PsFoundation.h"
#include "PsInlineArray.h"
#include "PsIntrinsics.h"
#include "PxPhysics.h"
#include "PxRigidActor.h"
#include "PxScene.h"
#include "PxShape.h"
#include <algorithm>
#include <vector>

using namespace physx;

namespace {
constexpr PxU32 GROUP_SIZE = 32;
constexpr PxU32 LAYER_WORDS = GROUP_SIZE / 2;

const PxU64* collisionRules(const void* data, PxU32 size) {
    return size >= LAYER_WORDS * sizeof(PxU64) ? static_cast<const PxU64*>(data) : nullptr;
}

std::vector<PxU64> copyRules(const PxScene& scene) {
    const auto* data = collisionRules(scene.getFilterShaderData(), scene.getFilterShaderDataSize());
    return data ? std::vector<PxU64>(data, data + scene.getFilterShaderDataSize() / sizeof(PxU64))
                : std::vector<PxU64>(LAYER_WORDS, ~PxU64(0));
}

bool layerCollisionEnabled(const PxU64* rules, PxU32 group1, PxU32 group2) {
    return group1 < GROUP_SIZE && group2 < GROUP_SIZE &&
           (!rules || (rules[group1 / 2] & (PxU64(1) << (group2 + 32 * (group1 % 2)))) != 0);
}

PxU64 pairKey(PxU32 first, PxU32 second) {
    return (PxU64(std::min(first, second)) << 32) | std::max(first, second);
}

bool pairCollisionEnabled(const void* data, PxU32 size, const PxFilterData& first, const PxFilterData& second) {
    const PxU32 group1 = first.word0, group2 = second.word0;
    if (group1 >= GROUP_SIZE || group2 >= GROUP_SIZE) return false;
    const PxU32 bit1 = 1u << group1, bit2 = 1u << group2;
    if ((first.word3 & bit2) || (second.word3 & bit1)) return false;

    const auto* rules = collisionRules(data, size);
    if (!(first.word2 & bit2) && !(second.word2 & bit1) && !layerCollisionEnabled(rules, group1, group2)) {
        return false;
    }
    if (rules && size > LAYER_WORDS * sizeof(PxU64)) {
        const auto* pairs = rules + LAYER_WORDS;
        const auto* end = rules + size / sizeof(PxU64);
        return !std::binary_search(pairs, end, pairKey(first.word1, second.word1));
    }
    return true;
}

void writeRules(PxScene& scene, const std::vector<PxU64>& rules) {
    scene.setFilterShaderData(rules.data(), static_cast<PxU32>(rules.size() * sizeof(PxU64)));
    PxInvalidateSceneQueryStaticTimestamp(scene);
}

static bool getFilterData(const PxActor& actor, PxFilterData& fd) {
    PxActorType::Enum aType = actor.getType();
    switch (aType) {
        case PxActorType::eRIGID_DYNAMIC:
        case PxActorType::eRIGID_STATIC:
        case PxActorType::eARTICULATION_LINK: {
            const PxRigidActor& rActor = static_cast<const PxRigidActor&>(actor);

            PxShape* shape = NULL;
            if (rActor.getShapes(&shape, 1) == 0 || !shape) {
                return false;
            }

            fd = shape->getSimulationFilterData();
            return true;
        }

        case PxActorType::eACTOR_COUNT:
        case PxActorType::eACTOR_FORCE_DWORD:
            break;
    }

    return false;
}

}  // namespace

PxFilterFlags physx::simulationFilterShader(PxFilterObjectAttributes attributes0,
                                            PxFilterData filterData0,
                                            PxFilterObjectAttributes attributes1,
                                            PxFilterData filterData1,
                                            PxPairFlags& pairFlags,
                                            const void* constantBlock,
                                            PxU32 constantBlockSize) {
    if (!pairCollisionEnabled(constantBlock, constantBlockSize, filterData0, filterData1)) {
        return PxFilterFlag::eSUPPRESS;
    }

    // let triggers through
    if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1)) {
        pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
        return PxFilterFlags();
    }

    pairFlags = PxPairFlag::eCONTACT_DEFAULT | PxPairFlag::eNOTIFY_TOUCH_FOUND | PxPairFlag::eNOTIFY_TOUCH_PERSISTS |
                PxPairFlag::eNOTIFY_TOUCH_LOST | PxPairFlag::eNOTIFY_CONTACT_POINTS | PxPairFlag::ePRE_SOLVER_VELOCITY |
                PxPairFlag::eDETECT_CCD_CONTACT | PxPairFlag::eNOTIFY_TOUCH_CCD;

    return PxFilterFlag::eDEFAULT;
}

void physx::invalidateShapeSweepCache(const PxShape& shape) {
    const PxRigidActor* actor = shape.getActor();
    if (actor && actor->getScene()) {
        PxInvalidateSceneQueryStaticTimestamp(*actor->getScene());
    }
}

bool physx::getGroupCollisionFlag(const PxScene& scene, PxU32 group1, PxU32 group2) {
    return layerCollisionEnabled(collisionRules(scene.getFilterShaderData(), scene.getFilterShaderDataSize()),
                                 group1, group2);
}

void physx::setGroupCollisionFlag(PxScene& scene, PxU32 group1, PxU32 group2, bool enable) {
    if (group1 >= GROUP_SIZE || group2 >= GROUP_SIZE || getGroupCollisionFlag(scene, group1, group2) == enable) {
        return;
    }
    auto rules = copyRules(scene);
    const PxU64 bit1 = PxU64(1) << (group1 + 32 * (group2 % 2));
    const PxU64 bit2 = PxU64(1) << (group2 + 32 * (group1 % 2));
    if (enable) {
        rules[group1 / 2] |= bit2;
        rules[group2 / 2] |= bit1;
    } else {
        rules[group1 / 2] &= ~bit2;
        rules[group2 / 2] &= ~bit1;
    }
    writeRules(scene, rules);
    PxMarkSceneFilterPairDirty(scene, group1, group2, false);
}

bool physx::shouldCollide(const PxScene& scene, const PxFilterData& first, const PxFilterData& second) {
    return pairCollisionEnabled(scene.getFilterShaderData(), scene.getFilterShaderDataSize(), first, second);
}

void physx::setPairIgnored(PxScene& scene, PxU32 first, PxU32 second, bool ignored) {
    auto rules = copyRules(scene);
    const PxU64 key = pairKey(first, second);
    const auto pos = std::lower_bound(rules.begin() + LAYER_WORDS, rules.end(), key);
    const bool found = pos != rules.end() && *pos == key;
    if (found == ignored) return;
    if (ignored) rules.insert(pos, key);
    else rules.erase(pos);
    writeRules(scene, rules);
    PxMarkSceneFilterPairDirty(scene, first, second, true);
}

void physx::removeIgnoredPairs(PxScene& scene, PxU32 shapeId) {
    if (scene.getFilterShaderDataSize() <= LAYER_WORDS * sizeof(PxU64)) return;
    auto rules = copyRules(scene);
    const auto end = std::remove_if(rules.begin() + LAYER_WORDS, rules.end(), [shapeId](PxU64 key) {
        return static_cast<PxU32>(key >> 32) == shapeId || static_cast<PxU32>(key) == shapeId;
    });
    if (end == rules.end()) return;
    rules.erase(end, rules.end());
    writeRules(scene, rules);
}

PxU16 physx::getGroup(const PxActor& actor) {
    PxFilterData fd;
    if (!getFilterData(actor, fd)) {
        return 0;
    }
    return PxU16(fd.word0);
}

void physx::setGroup(PxActor& actor, const PxU32 collisionGroup) {
    if (collisionGroup >= GROUP_SIZE) {
        return;
    }

    PxActorType::Enum aType = actor.getType();
    switch (aType) {
        case PxActorType::eRIGID_DYNAMIC:
        case PxActorType::eRIGID_STATIC:
        case PxActorType::eARTICULATION_LINK: {
            const PxRigidActor& rActor = static_cast<const PxRigidActor&>(actor);

            for (PxU32 i = 0; i < rActor.getNbShapes(); i++) {
                PxShape* shape = NULL;
                if (rActor.getShapes(&shape, 1, i) == 1 && shape) {
                    setShapeGroup(*shape, collisionGroup);
                }
            }
        } break;
        case PxActorType::eACTOR_COUNT:
        case PxActorType::eACTOR_FORCE_DWORD:
            break;
    }
}

bool physx::setShapeGroup(PxShape& shape, const PxU32 collisionGroup) {
    // Checked explicitly: PX_CHECK_* compiles away in release builds, and the query filter shifts by this value.
    if (collisionGroup >= GROUP_SIZE) {
        return false;
    }
    PxFilterData fd = shape.getSimulationFilterData();
    if (fd.word0 != collisionGroup) {
        fd.word0 = collisionGroup;
        shape.setSimulationFilterData(fd);
        invalidateShapeSweepCache(shape);
    }
    return true;
}

void physx::setShapeLayerOverrides(PxShape& shape, PxU32 includeLayers, PxU32 excludeLayers) {
    PxFilterData data = shape.getSimulationFilterData();
    data.word2 = includeLayers;
    data.word3 = excludeLayers;
    shape.setSimulationFilterData(data);
    invalidateShapeSweepCache(shape);
}
