
#pragma once

#include <cstdint>

#include "PxPhysicsAPI.h"

static_assert(sizeof(void*) >= sizeof(uint32_t), "Pointer must hold a 32-bit UUID");

inline void setUUID(physx::PxShape& shape, uint32_t uuid) {
    shape.userData = reinterpret_cast<void*>(static_cast<uintptr_t>(uuid));
}

inline uint32_t getUUID(const physx::PxShape* shape) {
    return shape ? static_cast<uint32_t>(reinterpret_cast<uintptr_t>(shape->userData)) : 0u;
}

inline uint32_t getUUID(const physx::PxRigidActor* actor) {
    return actor ? static_cast<uint32_t>(reinterpret_cast<uintptr_t>(actor->userData)) : 0u;
}

inline bool isSceneQueryEnabled(const physx::PxShape& shape) {
    return shape.getQueryFilterData().word0 == 0;
}

inline void setSceneQueryEnabled(physx::PxShape& shape, bool enabled) {
    // Native query flags must stay enabled for character-controller collision queries
    physx::PxFilterData data = shape.getQueryFilterData();
    data.word0 = enabled ? 0 : 1;
    shape.setQueryFilterData(data);
}

inline bool areValidJointActors(const physx::PxRigidActor* actor0, const physx::PxRigidActor* actor1) {
    if (actor0 == actor1) {
        return false;
    }
    const bool hasDynamic =
        (actor0 && actor0->is<physx::PxRigidBody>() != nullptr) ||
        (actor1 && actor1->is<physx::PxRigidBody>() != nullptr);
    if (!hasDynamic) {
        return false;
    }
    if (actor0 && actor1) {
        physx::PxScene* scene0 = actor0->getScene();
        physx::PxScene* scene1 = actor1->getScene();
        if (scene0 && scene1 && scene0 != scene1) {
            return false;
        }
    }
    return true;
}
