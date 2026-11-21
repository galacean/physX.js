
#pragma once

#include <algorithm>
#include <cmath>
#include <emscripten.h>
#include <emscripten/bind.h>
#include <vector>

#include "../BindingHelper.h"
#include "../SimulationFilterShader.h"
#include "PsInlineArray.h"
#include "PxPhysicsAPI.h"

namespace physx {
    void setControllerSlopeWalls(PxController &controller, PxF32 wallHeight, PxF32 jumpHeight);
}

using namespace physx;
using namespace emscripten;

inline PxShape *getControllerShape(const PxController &ctrl) {
    PxRigidDynamic *actor = ctrl.getActor();
    PxShape *shape = nullptr;
    if (actor) {
        actor->getShapes(&shape, 1);
    }
    return shape;
}

inline bool isSolidControllerShape(const PxShape *shape) {
    return shape && (shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE);
}

using ControllerExcludedIds = shdfnd::InlineArray<PxU32, 8>;

struct ControllerLayerFilter : PxQueryFilterCallback, PxControllerFilterCallback {
    const PxScene &scene;
    PxFilterData controllerData;
    PxU32 layerMask;
    const ControllerExcludedIds &excluded;

    ControllerLayerFilter(const PxScene &owner, PxFilterData data, PxU32 mask, const ControllerExcludedIds &excludedIds)
        : scene(owner), controllerData(data), layerMask(mask), excluded(excludedIds) {}

    PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape,
                                   const PxRigidActor *actor, PxHitFlags &) override {
        if (actor->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) return PxQueryHitType::eNONE;
        return accepts(shape) ? PxQueryHitType::eBLOCK : PxQueryHitType::eNONE;
    }

    PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &) override {
        return PxQueryHitType::eNONE;
    }

    bool filter(const PxController &, const PxController &target) override {
        return accepts(getControllerShape(target));
    }

private:
    bool accepts(const PxShape *shape) const {
        // Solid shapes block movement independently of scene-query participation.
        if (!isSolidControllerShape(shape)) return false;

        const PxFilterData target = shape->getSimulationFilterData();
        return target.word0 < 32 && (layerMask & (1u << target.word0)) &&
               std::find(excluded.begin(), excluded.end(), target.word1) == excluded.end() &&
               physx::shouldCollide(scene, controllerData, target);
    }
};

struct ControllerInteractionFilter : PxControllerFilterCallback {
    const PxScene &scene;

    explicit ControllerInteractionFilter(const PxScene &owner) : scene(owner) {}

    bool filter(const PxController &first, const PxController &second) override {
        const PxShape *firstShape = getControllerShape(first);
        const PxShape *secondShape = getControllerShape(second);
        return isSolidControllerShape(firstShape) && isSolidControllerShape(secondShape) &&
               physx::shouldCollide(scene, firstShape->getSimulationFilterData(), secondShape->getSimulationFilterData());
    }
};

class ControllerHitReport final : public PxUserControllerHitReport {
public:
    bool collecting = false;
    std::vector<double> values{1};

    void onShapeHit(const PxControllerShapeHit &hit) override {
        if (collecting) append(hit, hit.shape);
    }

    void onControllerHit(const PxControllersHit &hit) override {
        if (!collecting) return;
        if (const PxShape *shape = getControllerShape(*hit.other)) {
            append(hit, shape);
        }
    }

    void onObstacleHit(const PxControllerObstacleHit &) override {}

private:
    void append(const PxControllerHit &hit, const PxShape *shape) {
        const auto start = static_cast<size_t>(values[0]);
        const auto end = start + 11;
        if (values.size() < end) values.resize(end);
        values[0] = end;
        double *out = values.data() + start;
        *out++ = getUUID(shape);
        *out++ = hit.worldPos.x;
        *out++ = hit.worldPos.y;
        *out++ = hit.worldPos.z;
        *out++ = hit.worldNormal.x;
        *out++ = hit.worldNormal.y;
        *out++ = hit.worldNormal.z;
        *out++ = hit.dir.x;
        *out++ = hit.dir.y;
        *out++ = hit.dir.z;
        *out = hit.length;
    }
};

inline ControllerHitReport &getControllerHitReport() {
    static ControllerHitReport report;
    return report;
}

inline PxU32 moveController(PxController &controller, const PxVec3 &disp, PxF32 minDist, PxF32 elapsedTime,
                           bool collectHits, PxU32 layerMask = ~PxU32(0), const val *excludedIds = nullptr) {
    auto &report = getControllerHitReport();
    if (!disp.isFinite() || !PxIsFinite(minDist) || minDist < 0.0f ||
        !PxIsFinite(elapsedTime) || elapsedTime < 0.0f) {
        report.values[0] = 1;
        return 0;
    }

    ControllerExcludedIds excluded;
    if (excludedIds && !excludedIds->isUndefined() && !excludedIds->isNull()) {
        const auto count = (*excludedIds)["length"].as<PxU32>();
        excluded.resize(count);
        for (PxU32 i = 0; i < count; ++i) excluded[i] = (*excludedIds)[i].as<PxU32>();
    }

    const PxShape *shape = getControllerShape(controller);
    ControllerLayerFilter filter(*controller.getScene(), shape ? shape->getSimulationFilterData() : PxFilterData(),
                                 layerMask, excluded);
    PxControllerFilters filters(nullptr, &filter, &filter);
    filters.mReuseGeometryCache = layerMask == ~PxU32(0) && excluded.size() == 0;
    report.values[0] = 1;
    report.collecting = collectHits;
    const PxU32 flags = controller.move(disp, minDist, elapsedTime, filters);
    report.collecting = false;
    return flags;
}

class ControllerPlatformBehavior final : public PxControllerBehaviorCallback {
public:
    PxControllerBehaviorFlags getBehaviorFlags(const PxShape &, const PxActor &) override {
        return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxController &) override {
        return {};
    }

    PxControllerBehaviorFlags getBehaviorFlags(const PxObstacle &) override {
        return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT;
    }
};

inline val getControllerMoveHits() {
    const auto &values = getControllerHitReport().values;
    if (values[0] == 1) return val::null();
    static val view = val::null();
    // A WebAssembly memory grow detaches the previous view.
    if (view.isNull() || view["length"].as<size_t>() != values.size()) {
        view = val(typed_memory_view(values.size(), values.data()));
    }
    return view;
}

EMSCRIPTEN_BINDINGS(physx_controller) {
    function("getControllerMoveHits", &getControllerMoveHits);

    enum_<PxCapsuleClimbingMode::Enum>("PxCapsuleClimbingMode")
            .value("eCONSTRAINED", PxCapsuleClimbingMode::Enum::eCONSTRAINED)
            .value("eLAST", PxCapsuleClimbingMode::Enum::eLAST)
            .value("eEASY", PxCapsuleClimbingMode::Enum::eEASY);
    // enum_<PxControllerShapeType::Enum>("PxControllerShapeType")
    //         .value("eBOX", PxControllerShapeType::Enum::eBOX)
    //         .value("eCAPSULE", PxControllerShapeType::Enum::eCAPSULE);
    enum_<PxControllerNonWalkableMode::Enum>("PxControllerNonWalkableMode") // ✅
            .value("ePREVENT_CLIMBING", PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING) // ✅
            .value("ePREVENT_CLIMBING_AND_FORCE_SLIDING",
                   PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING_AND_FORCE_SLIDING); // ✅
    enum_<PxControllerCollisionFlag::Enum>("PxControllerCollisionFlag") // ✅
            .value("eCOLLISION_SIDES", PxControllerCollisionFlag::Enum::eCOLLISION_SIDES) // ✅
            .value("eCOLLISION_UP", PxControllerCollisionFlag::Enum::eCOLLISION_UP) // ✅
            .value("eCOLLISION_DOWN", PxControllerCollisionFlag::Enum::eCOLLISION_DOWN); // ✅
    // class_<PxControllerCollisionFlags>("PxControllerCollisionFlags")
    //         .constructor<int>()
    //         .function("isSet", &PxControllerCollisionFlags::isSet);
    /** PhysXCharacterControllerManager ✅ */
    class_<PxControllerManager>("PxControllerManager")
            // .function("purgeControllers", &PxControllerManager::purgeControllers)                              // ✅
            .function("createController", optional_override([](PxControllerManager &manager, PxControllerDesc &desc) {
                          static ControllerPlatformBehavior platformBehavior;
                          desc.reportCallback = &getControllerHitReport();
                          desc.behaviorCallback = &platformBehavior;
                          return manager.createController(desc);
                      }), allow_raw_pointers())  // ✅
            .function("setOverlapRecoveryModule", &PxControllerManager::setOverlapRecoveryModule)
            .function("setPreciseSweeps", &PxControllerManager::setPreciseSweeps)
            .function("setPreventVerticalSlidingAgainstCeiling", &PxControllerManager::setPreventVerticalSlidingAgainstCeiling)
            .function("computeInteractions", optional_override([](PxControllerManager &manager, PxF32 elapsedTime) {
                          if (PxIsFinite(elapsedTime) && elapsedTime >= 0.0f) {
                              ControllerInteractionFilter filter(manager.getScene());
                              manager.computeInteractions(elapsedTime, &filter);
                          }
                      }))
            .function("release", &PxControllerManager::release);  // ✅
            // .function("setTessellation", &PxControllerManager::setTessellation)                                // ✅
            // .function("shiftOrigin", &PxControllerManager::shiftOrigin);              // ✅
    /** PhysXCharacterControllerDesc ✅ */
    class_<PxControllerDesc>("PxControllerDesc")
            // .function("isValid", &PxControllerDesc::isValid)
            // .function("getType", &PxControllerDesc::getType)                          // ✅
            .property("position", &PxControllerDesc::position)                        // ✅
            .property("upDirection", &PxControllerDesc::upDirection)                  // ✅
            .property("slopeLimit", &PxControllerDesc::slopeLimit)                    // ✅
            .property("invisibleWallHeight", &PxControllerDesc::invisibleWallHeight)  // ✅
            .property("maxJumpHeight", &PxControllerDesc::maxJumpHeight)              // ✅
            .property("contactOffset", &PxControllerDesc::contactOffset)              // ✅
            .property("stepOffset", &PxControllerDesc::stepOffset)                    // ✅
            .property("density", &PxControllerDesc::density)                          // ✅
            .property("scaleCoeff", &PxControllerDesc::scaleCoeff)                    // ✅
            .property("volumeGrowth", &PxControllerDesc::volumeGrowth)                // ✅
            .function("setNonWalkableMode", optional_override([](PxControllerDesc &desc, int mode) {
                          return desc.nonWalkableMode = PxControllerNonWalkableMode::Enum(mode);
                      }))  // ✅
            .function("setMaterial", optional_override([](PxControllerDesc &desc, PxMaterial *material) {
                          return desc.material = material;
                      }),
                      allow_raw_pointers())                                                      // ✅
            .property("registerDeletionListener", &PxControllerDesc::registerDeletionListener);  // ✅
    /** PhysXCapsuleCharacterControllerDesc ✅ */
    class_<PxCapsuleControllerDesc, base<PxControllerDesc>>("PxCapsuleControllerDesc")
            .constructor<>()
            // .function("isValid", &PxCapsuleControllerDesc::isValid)
            // .function("setToDefault", &PxCapsuleControllerDesc::setToDefault)   // ✅
            .property("radius", &PxCapsuleControllerDesc::radius)               // ✅
            .property("height", &PxCapsuleControllerDesc::height)               // ✅
            .property("climbingMode", &PxCapsuleControllerDesc::climbingMode);  // ✅
    /** PhysXBoxCharacterControllerDesc ✅ */
    class_<PxBoxControllerDesc, base<PxControllerDesc>>("PxBoxControllerDesc")
            .constructor<>()
            // .function("isValid", &PxBoxControllerDesc::isValid)
            // // .function("setToDefault", &PxBoxControllerDesc::setToDefault)            // ✅
            .property("halfForwardExtent", &PxBoxControllerDesc::halfForwardExtent)  // ✅
            .property("halfHeight", &PxBoxControllerDesc::halfHeight)                // ✅
            .property("halfSideExtent", &PxBoxControllerDesc::halfSideExtent);       // ✅

    /** PhysXCharacterController ✅ */
    class_<PxController>("PxController")
            .function("release", &PxController::release)
            // .function("isSetControllerCollisionFlag",
            //           optional_override([](PxController &controller, PxControllerCollisionFlags &flags, int flag) {
            //               return flags.isSet(PxControllerCollisionFlag::Enum(flag));
            //           }))  // ✅
            .function("move", optional_override([](PxController &controller, const PxVec3 &disp, PxF32 minDist,
                                                   PxF32 elapsedTime, bool collectHits) {
                          return moveController(controller, disp, minDist, elapsedTime, collectHits);
                      }))
            .function("moveWithFilter", optional_override([](PxController &controller, const PxVec3 &disp, PxF32 minDist,
                                                             PxF32 elapsedTime, bool collectHits, PxU32 layerMask,
                                                             const val &excludedIds) {
                          return moveController(controller, disp, minDist, elapsedTime, collectHits, layerMask, &excludedIds);
                      }))
            .function("setPosition", optional_override([](PxController &controller, const PxExtendedVec3 &pos) {
                          if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
                              return false;
                          }
                          return controller.setPosition(pos);
                      }))                                         // ✅
            .function("getPosition", &PxController::getPosition)  // ✅
            .function("setFootPosition", optional_override([](PxController &controller, const PxExtendedVec3 &pos) {
                          return toVec3(pos).isFinite() && controller.setFootPosition(pos);
                      }))
            .function("getFootPosition", optional_override([](const PxController &controller, const val &out) {
                          const PxExtendedVec3 position = controller.getFootPosition();
                          out.call<void>("set", position.x, position.y, position.z);
                      }))
            .function("setStepOffset", optional_override([](PxController &controller, PxF32 offset) {
                          if (PxIsFinite(offset) && offset >= 0.0f) {
                              controller.setStepOffset(offset);
                          }
                      }))  // ✅
            .function("setNonWalkableMode", optional_override([](PxController &controller, int mode) {
                          return controller.setNonWalkableMode(PxControllerNonWalkableMode::Enum(mode));
                      }))  // ✅
            .function("setContactOffset", optional_override([](PxController &controller, PxF32 offset) {
                          if (PxIsFinite(offset) && offset > 0.0f) {
                              controller.setContactOffset(offset);
                          }
                      }))  // ✅
            .function("setMaterial", optional_override([](PxController &ctrl, PxMaterial &material) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              PxMaterial *materialPtr = &material;
                              shape->setMaterials(&materialPtr, 1);
                          }
                      }))
            .function("setUpDirection", optional_override([](PxController &controller, const PxVec3 &up) {
                          if (up.isFinite() && up.magnitudeSquared() > 0.0f) {
                              controller.setUpDirection(up.getNormalized());
                          }
                      }))  // ✅
            .function("setSlopeLimit", optional_override([](PxController &controller, PxF32 slopeLimit) {
                          if (PxIsFinite(slopeLimit) && slopeLimit >= 0.0f) {
                              controller.setSlopeLimit(slopeLimit);
                          }
                      }))  // ✅
            // .function("invalidateCache", &PxController::invalidateCache)    // ✅
            .function("resize", optional_override([](PxController &controller, PxF32 height) {
                          // PhysX expects a capsule's cylindrical height or a box's half height.
                          if (PxIsFinite(height) && height > 0.0f) {
                              controller.resize(height);
                          }
                      }))
            /** PxCapsuleController ✅ */
            .function("setRadius", optional_override([](PxController &ctrl, PxF32 radius) {
                          if (ctrl.getType() == PxControllerShapeType::eCAPSULE && PxIsFinite(radius) &&
                              radius > 0.0f) {
                              static_cast<PxCapsuleController *>(&ctrl)->setRadius(radius);
                          }
                      }))  // ✅
            .function("setHeight", optional_override([](PxController &ctrl, PxF32 height) {
                          if (ctrl.getType() == PxControllerShapeType::eCAPSULE && PxIsFinite(height) &&
                              height > 0.0f) {
                              static_cast<PxCapsuleController *>(&ctrl)->setHeight(height);
                          }
                      }))  // ✅
            .function("setClimbingMode", optional_override([](PxController &ctrl, int mode) {
                          return ctrl.getType() == PxControllerShapeType::eCAPSULE &&
                                 (mode == PxCapsuleClimbingMode::eEASY ||
                                  mode == PxCapsuleClimbingMode::eCONSTRAINED) &&
                                 static_cast<PxCapsuleController &>(ctrl).setClimbingMode(
                                         PxCapsuleClimbingMode::Enum(mode));
                      }))
            /** PxBoxController ✅ */
            .function("setHalfHeight", optional_override([](PxController &ctrl, PxF32 halfHeight) {
                          if (ctrl.getType() == PxControllerShapeType::eBOX && PxIsFinite(halfHeight) &&
                              halfHeight > 0.0f) {
                              static_cast<PxBoxController *>(&ctrl)->setHalfHeight(halfHeight);
                          }
                      }))  // ✅
            .function("setHalfSideExtent", optional_override([](PxController &ctrl, PxF32 halfSideExtent) {
                          if (ctrl.getType() == PxControllerShapeType::eBOX && PxIsFinite(halfSideExtent) &&
                              halfSideExtent > 0.0f) {
                              static_cast<PxBoxController *>(&ctrl)->setHalfSideExtent(halfSideExtent);
                          }
                      }))  // ✅
            .function("setHalfForwardExtent", optional_override([](PxController &ctrl, PxF32 halfForwardExtent) {
                          if (ctrl.getType() == PxControllerShapeType::eBOX && PxIsFinite(halfForwardExtent) &&
                              halfForwardExtent > 0.0f) {
                              static_cast<PxBoxController *>(&ctrl)->setHalfForwardExtent(halfForwardExtent);
                          }
                      }))  // ✅
            .function("setUUID", optional_override([](PxController &ctrl, uint32_t uuid) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              setUUID(*shape, uuid);
                          }
                      }))  // ✅
            .function("setLayerOverrides", optional_override([](PxController &ctrl, PxU32 include, PxU32 exclude) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              physx::setShapeLayerOverrides(*shape, include, exclude);
                          }
                      }))
            .function("setSceneQueryEnabled", optional_override([](PxController &ctrl, bool enabled) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              setSceneQueryEnabled(*shape, enabled);
                          }
                      }))
            .function("setContactEventsEnabled", optional_override([](PxController &ctrl, bool enabled) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              setShapeContactEventsEnabled(*shape, enabled);
                          }
                      }))
            .function("setContactModificationEnabled", optional_override([](PxController &ctrl, bool enabled) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              setShapeContactModificationEnabled(*shape, enabled);
                          }
                      }))
            .function("setDetectCollisions", optional_override([](PxController &ctrl, bool enabled) {
                          if (PxShape *shape = getControllerShape(ctrl)) {
                              shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, enabled);
                          }
                      }))
            .function("setSlopeWalls", &physx::setControllerSlopeWalls)
            .function("getActor", &PxController::getActor, allow_raw_pointers());  // ✅
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxController>(PxController *) { /* do nothing */
}

template <>
void raw_destructor<PxCapsuleController>(PxCapsuleController *) { /* do nothing */
}

template <>
void raw_destructor<PxControllerDesc>(PxControllerDesc *) { /* do nothing */
}

template <>
void raw_destructor<PxControllerManager>(PxControllerManager *) { /* do nothing */
}

}  // namespace internal
}  // namespace emscripten
