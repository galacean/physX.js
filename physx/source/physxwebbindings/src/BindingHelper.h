
#pragma once

#include <cmath>
#include <cstdint>

#include "PxPhysicsAPI.h"

inline bool tryParseIntRange(double value, physx::PxU32 minVal, physx::PxU32 maxVal, physx::PxU32& out) {
    if (!std::isfinite(value) || std::floor(value) != value || value < static_cast<double>(minVal) ||
        value > static_cast<double>(maxVal)) {
        return false;
    }
    out = static_cast<physx::PxU32>(value);
    return true;
}

inline bool tryParseCollisionGroup(double value, physx::PxU32& out) {
    return tryParseIntRange(value, 0u, 31u, out);
}

inline void setUUID(physx::PxShape& shape, uint32_t uuid) {
    auto data = shape.getSimulationFilterData();
    data.word1 = uuid;
    shape.setSimulationFilterData(data);
}

inline uint32_t getUUID(const physx::PxShape* shape) {
    return shape ? shape->getSimulationFilterData().word1 : 0u;
}

inline uint32_t getUUID(const physx::PxRigidActor* actor) {
    return actor ? static_cast<uint32_t>(reinterpret_cast<uintptr_t>(actor->userData)) : 0u;
}

inline bool isVolumeGeometry(physx::PxGeometryType::Enum type) {
    using Type = physx::PxGeometryType;
    return type == Type::eBOX || type == Type::eSPHERE || type == Type::eCAPSULE || type == Type::eCONVEXMESH;
}

inline bool isSceneQueryEnabled(const physx::PxShape& shape) {
    return shape.getQueryFilterData().word0 == 0;
}

inline void setShapeContactEventsEnabled(physx::PxShape& shape, bool enabled) {
    shape.setFlag(physx::PxShapeFlag::eDISABLE_CONTACT_EVENTS, !enabled);
}

inline void setShapeContactModificationEnabled(physx::PxShape& shape, bool enabled) {
    shape.setFlag(physx::PxShapeFlag::eMODIFY_CONTACTS, enabled);
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
