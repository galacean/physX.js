#pragma once

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten/bind.h>
#include <algorithm>
#include <array>
#include <vector>

using namespace physx;
using namespace emscripten;

struct SceneQueryFilter : PxQueryFilterCallback {
    PxU32 layerMask;
    PxQueryHitType::Enum hitType;

    const std::vector<PxU32> &excluded;
    bool includeTriggers;

    SceneQueryFilter(PxU32 mask, bool single, const std::vector<PxU32> &excludedIds, bool triggers)
        : layerMask(mask), hitType(single ? PxQueryHitType::eBLOCK : PxQueryHitType::eTOUCH),
          excluded(excludedIds), includeTriggers(triggers) {}

    PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape,
                                   const PxRigidActor *, PxHitFlags &) override {
        if (!isSceneQueryEnabled(*shape) || (!includeTriggers && (shape->getFlags() & PxShapeFlag::eTRIGGER_SHAPE))) {
            return PxQueryHitType::eNONE;
        }
        const PxFilterData data = shape->getSimulationFilterData();
        const PxU32 group = data.word0;
        return group < 32 && (layerMask & (1u << group)) &&
               std::find(excluded.begin(), excluded.end(), data.word1) == excluded.end()
            ? hitType : PxQueryHitType::eNONE;
    }

    PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &hit) override {
        return static_cast<const PxLocationHit &>(hit).distance > 0 ? hitType : PxQueryHitType::eNONE;
    }
};

static double getTriangleIndex(const PxQueryHit &hit) {
    if (hit.faceIndex == 0xffffffffu) return -1;
    switch (hit.shape->getGeometryType()) {
        case PxGeometryType::eTRIANGLEMESH: {
            PxTriangleMeshGeometry geometry;
            hit.shape->getTriangleMeshGeometry(geometry);
            const auto *remap = geometry.triangleMesh->getTrianglesRemap();
            return remap ? remap[hit.faceIndex] : hit.faceIndex;
        }
        case PxGeometryType::eHEIGHTFIELD:
            return hit.faceIndex;
        default:
            return -1;
    }
}

// Double precision preserves every 32-bit shape ID alongside the hit values
struct SceneQueryHit {
    double shapeID, distance, x, y, z, nx, ny, nz, triangleIndex;

    template <typename Hit>
    explicit SceneQueryHit(const Hit &hit)
        : shapeID(getUUID(hit.shape)), distance(hit.distance),
          x(hit.position.x), y(hit.position.y), z(hit.position.z),
          nx(hit.normal.x), ny(hit.normal.y), nz(hit.normal.z),
          triangleIndex(getTriangleIndex(hit)) {}
};
static_assert(sizeof(SceneQueryHit) == 9 * sizeof(double), "Scene query hit layout must be packed");

template <typename Hit>
struct CastCollector : PxHitCallback<Hit> {
    std::vector<SceneQueryHit> &hits;

    CastCollector(Hit *touches, PxU32 capacity, std::vector<SceneQueryHit> &results)
        : PxHitCallback<Hit>(touches, capacity), hits(results) {}

    PxAgain processTouches(const Hit *buffer, PxU32 count) override {
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
    PxScene *_scene;
    std::array<PxRaycastHit, kTouchCount> _raycastTouches;
    std::array<PxSweepHit, kTouchCount> _sweepTouches;
    std::array<PxOverlapHit, kTouchCount> _overlapTouches;
    std::vector<SceneQueryHit> _hits;
    std::vector<PxU32> _shapeIDs;
    std::vector<PxU32> _excluded;

    SceneQueryFilter makeFilter(PxU32 mask, bool single, bool includeTriggers, const val &excluded) {
        _excluded.clear();
        if (!excluded.isUndefined() && !excluded.isNull()) {
            const auto count = excluded["length"].as<unsigned>();
            for (unsigned i = 0; i < count; ++i) {
                _excluded.push_back(excluded[i].as<PxU32>());
            }
        }
        return SceneQueryFilter(mask, single, _excluded, includeTriggers);
    }

    val hitValues() const {
        // Borrowed until the next query; layout is ID, distance, position.xyz, normal.xyz, triangle index
        return val(typed_memory_view(_hits.size() * 9, reinterpret_cast<const double *>(_hits.data())));
    }

    static PxHitFlags castHitFlags(bool hitBackfaces) {
        PxHitFlags flags = PxHitFlag::eDEFAULT;
        if (hitBackfaces) flags |= PxHitFlag::eMESH_BOTH_SIDES;
        return flags;
    }

    static PxQueryFilterData castFilterData() {
        return PxQueryFilterData(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC |
                                 PxQueryFlag::ePREFILTER | PxQueryFlag::ePOSTFILTER);
    }

public:
    explicit PxSceneQueries(PxScene *scene) : _scene(scene) {}

    val raycast(const PxVec3 &origin, const PxVec3 &direction, PxReal distance, PxU32 layerMask, bool includeTriggers, const val &excluded, bool hitBackfaces) {
        _hits.clear();
        if (_scene && distance > 0 && PxIsFinite(distance) && origin.isFinite() && direction.isFinite() &&
            direction.magnitudeSquared() > 0) {
            auto filter = makeFilter(layerMask, true, includeTriggers, excluded);
            PxRaycastBuffer result;
            _scene->raycast(origin, direction.getNormalized(), distance, result, castHitFlags(hitBackfaces),
                            castFilterData(), &filter);
            if (result.hasBlock) {
                _hits.emplace_back(result.block);
            }
        }
        return hitValues();
    }

    val raycastAll(const PxVec3 &origin, const PxVec3 &direction, PxReal distance, PxU32 layerMask, bool includeTriggers, const val &excluded, bool hitBackfaces) {
        _hits.clear();
        if (_scene && distance > 0 && PxIsFinite(distance) && origin.isFinite() && direction.isFinite() &&
            direction.magnitudeSquared() > 0) {
            auto filter = makeFilter(layerMask, false, includeTriggers, excluded);
            CastCollector<PxRaycastHit> result(_raycastTouches.data(), kTouchCount, _hits);
            _scene->raycast(origin, direction.getNormalized(), distance, result, castHitFlags(hitBackfaces),
                            castFilterData(), &filter);
        }
        return hitValues();
    }

    val sweep(const PxGeometry &geometry, const PxTransform &pose, const PxVec3 &direction,
              PxReal distance, PxU32 layerMask, bool includeTriggers, const val &excluded, bool hitBackfaces, bool all) {
        _hits.clear();
        if (_scene && distance > 0 && PxIsFinite(distance) && pose.isSane() && direction.isFinite() &&
            direction.magnitudeSquared() > 0 && PxGeometryQuery::isValid(geometry)) {
            auto filter = makeFilter(layerMask, !all, includeTriggers, excluded);
            const PxVec3 normalized = direction.getNormalized();
            const PxHitFlags flags = castHitFlags(hitBackfaces);
            const PxQueryFilterData data = castFilterData();
            if (all) {
                CastCollector<PxSweepHit> result(_sweepTouches.data(), kTouchCount, _hits);
                _scene->sweep(geometry, pose, normalized, distance, result, flags, data, &filter);
            } else {
                PxSweepBuffer result;
                _scene->sweep(geometry, pose, normalized, distance, result, flags, data, &filter);
                if (result.hasBlock) _hits.emplace_back(result.block);
            }
        }
        return hitValues();
    }

    val overlap(const PxGeometry &geometry, const PxTransform &pose, PxU32 layerMask, bool includeTriggers, const val &excluded) {
        _shapeIDs.clear();
        if (_scene && pose.isSane() && PxGeometryQuery::isValid(geometry)) {
            auto filter = makeFilter(layerMask, false, includeTriggers, excluded);
            OverlapCollector result(_overlapTouches.data(), kTouchCount, _shapeIDs);
            const PxQueryFilterData data(PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER);
            _scene->overlap(geometry, pose, result, data, &filter);
        }
        return val(typed_memory_view(_shapeIDs.size(), _shapeIDs.data()));
    }
};
