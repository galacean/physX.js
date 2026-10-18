
#pragma once

#include "PxPhysicsAPI.h"

inline uint32_t getUUID(physx::PxShape* shape) {
    return *static_cast<uint32_t*>(shape->userData);
}

inline uint32_t getUUID(const physx::PxShape* shape) {
    return *static_cast<uint32_t*>(shape->userData);
}

inline uint32_t getUUID(const physx::PxRigidActor* actor) {
    return *static_cast<uint32_t*>(actor->userData);
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
