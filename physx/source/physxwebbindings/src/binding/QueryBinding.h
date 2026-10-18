#pragma once

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten/bind.h>
#include <array>
#include <vector>

using namespace physx;
using namespace emscripten;

struct SceneQueryFilter : PxQueryFilterCallback {
    PxU32 layerMask;
    PxQueryHitType::Enum hitType;

    SceneQueryFilter(PxU32 mask, bool single)
        : layerMask(mask), hitType(single ? PxQueryHitType::eBLOCK : PxQueryHitType::eTOUCH) {}

    PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape,
                                   const PxRigidActor *, PxHitFlags &) override {
        return layerMask & (1u << shape->getSimulationFilterData().word0) ? hitType : PxQueryHitType::eNONE;
    }

    PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &hit) override {
        return static_cast<const PxLocationHit &>(hit).distance > 0 ? hitType : PxQueryHitType::eNONE;
    }
};

// Double precision preserves every 32-bit shape ID alongside the hit values
struct SceneQueryHit {
    double shapeID, distance, x, y, z, nx, ny, nz;

    template <typename Hit>
    explicit SceneQueryHit(const Hit &hit)
        : shapeID(getUUID(hit.shape)), distance(hit.distance),
          x(hit.position.x), y(hit.position.y), z(hit.position.z),
          nx(hit.normal.x), ny(hit.normal.y), nz(hit.normal.z) {}
};
static_assert(sizeof(SceneQueryHit) == 8 * sizeof(double), "Scene query hit layout must be packed");

struct RaycastCollector : PxRaycastCallback {
    std::vector<SceneQueryHit> &hits;

    RaycastCollector(PxRaycastHit *touches, PxU32 capacity, std::vector<SceneQueryHit> &results)
        : PxRaycastCallback(touches, capacity), hits(results) {}

    PxAgain processTouches(const PxRaycastHit *buffer, PxU32 count) override {
        for (PxU32 i = 0; i < count; ++i) {
            hits.emplace_back(buffer[i]);
        }
        return true;
    }
};

struct OverlapCollector : PxOverlapCallback {
    std::vector<PxU32> &shapeIDs;

    OverlapCollector(PxOverlapHit *touches, PxU32 capacity, std::vector<PxU32> &results)
        : PxOverlapCallback(touches, capacity), shapeIDs(results) {}

    PxAgain processTouches(const PxOverlapHit *buffer, PxU32 count) override {
        for (PxU32 i = 0; i < count; ++i) {
            shapeIDs.push_back(getUUID(buffer[i].shape));
        }
        return true;
    }
};

class PxSceneQueries {
    static constexpr PxU32 kTouchCount = 256;
    PxScene &_scene;
    std::array<PxRaycastHit, kTouchCount> _raycastTouches;
    std::array<PxOverlapHit, kTouchCount> _overlapTouches;
    std::vector<SceneQueryHit> _hits;
    std::vector<PxU32> _shapeIDs;

    val hitValues() const {
        // Borrowed until the next query; layout is ID, distance, position.xyz, normal.xyz
        return val(typed_memory_view(_hits.size() * 8, reinterpret_cast<const double *>(_hits.data())));
    }

    static PxQueryFilterData castFilterData() {
        return PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC |
                                 PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER);
    }

public:
    explicit PxSceneQueries(PxScene *scene) : _scene(*scene) {}

    val raycast(const PxVec3 &origin, const PxVec3 &direction, PxReal distance, PxU32 layerMask) {
        _hits.clear();
        if (distance > 0) {
            SceneQueryFilter filter(layerMask, true);
            PxRaycastBuffer result;
            _scene.raycast(origin, direction, distance, result, PxHitFlag::eDEFAULT, castFilterData(), &filter);
            if (result.hasBlock) {
                _hits.emplace_back(result.block);
            }
        }
        return hitValues();
    }

    val raycastAll(const PxVec3 &origin, const PxVec3 &direction, PxReal distance, PxU32 layerMask) {
        _hits.clear();
        if (distance > 0) {
            SceneQueryFilter filter(layerMask, false);
            RaycastCollector result(_raycastTouches.data(), kTouchCount, _hits);
            _scene.raycast(origin, direction, distance, result, PxHitFlag::eDEFAULT, castFilterData(), &filter);
        }
        return hitValues();
    }

    val sweep(const PxGeometry &geometry, const PxTransform &pose, const PxVec3 &direction,
              PxReal distance, PxU32 layerMask) {
        _hits.clear();
        if (distance > 0) {
            SceneQueryFilter filter(layerMask, true);
            PxSweepBuffer result;
            _scene.sweep(geometry, pose, direction, distance, result, PxHitFlag::eDEFAULT, castFilterData(), &filter);
            if (result.hasBlock) {
                _hits.emplace_back(result.block);
            }
        }
        return hitValues();
    }

    val overlap(const PxGeometry &geometry, const PxTransform &pose, PxU32 layerMask) {
        _shapeIDs.clear();
        SceneQueryFilter filter(layerMask, false);
        OverlapCollector result(_overlapTouches.data(), kTouchCount, _shapeIDs);
        const PxQueryFilterData data(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);
        _scene.overlap(geometry, pose, result, data, &filter);
        return val(typed_memory_view(_shapeIDs.size(), _shapeIDs.data()));
    }
};
