
#pragma once

#include <cmath>
#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "../SimulationFilterShader.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

inline PxU32 getControllerGroup(const PxController &ctrl) {
    PxRigidDynamic *actor = ctrl.getActor();
    PxShape *shape = nullptr;
    if (actor && actor->getShapes(&shape, 1) == 1 && shape) {
        return shape->getSimulationFilterData().word0;
    }
    return 0;
}

struct ControllerLayerFilter : PxQueryFilterCallback, PxControllerFilterCallback {
    PxU32 controllerGroup;

    explicit ControllerLayerFilter(PxU32 group) : controllerGroup(group) {}

    PxQueryHitType::Enum preFilter(const PxFilterData &, const PxShape *shape,
                                   const PxRigidActor *, PxHitFlags &) override {
        // Skip triggers and non-simulation shapes; do not filter by isSceneQueryEnabled so solid shapes
        // with isSceneQuery = false still block CharacterController movement.
        if (!shape || !(shape->getFlags() & PxShapeFlag::eSIMULATION_SHAPE)) {
            return PxQueryHitType::eNONE;
        }
        return physx::getGroupCollisionFlag(controllerGroup, shape->getSimulationFilterData().word0)
                   ? PxQueryHitType::eBLOCK
                   : PxQueryHitType::eNONE;
    }

    PxQueryHitType::Enum postFilter(const PxFilterData &, const PxQueryHit &) override {
        return PxQueryHitType::eNONE;
    }

    bool filter(const PxController &a, const PxController &b) override {
        return physx::getGroupCollisionFlag(getControllerGroup(a), getControllerGroup(b));
    }
};

EMSCRIPTEN_BINDINGS(physx_controller) {
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
            .function("createController", &PxControllerManager::createController, allow_raw_pointers())           // ✅
            .function("release", &PxControllerManager::release);  // ✅
            // .function("computeInteractions", &PxControllerManager::computeInteractions, allow_raw_pointers())  // ✅
            // .function("setTessellation", &PxControllerManager::setTessellation)                                // ✅
            // .function("setOverlapRecoveryModule", &PxControllerManager::setOverlapRecoveryModule)              // ✅
            // .function("setPreciseSweeps", &PxControllerManager::setPreciseSweeps)                              // ✅
            // .function("setPreventVerticalSlidingAgainstCeiling",
                      // &PxControllerManager::setPreventVerticalSlidingAgainstCeiling)  // ✅
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
                                                   PxF32 elapsedTime) -> uint32_t {
                          if (!disp.isFinite() || !PxIsFinite(minDist) || !PxIsFinite(elapsedTime)) {
                              return 0;
                          }
                          const PxU32 filterRevision = physx::getCollisionFilterRevision();
                          const uintptr_t cachedRevision = reinterpret_cast<uintptr_t>(controller.getUserData());
                          if (cachedRevision != static_cast<uintptr_t>(filterRevision)) {
                              controller.invalidateCache();
                              controller.setUserData(
                                      reinterpret_cast<void *>(static_cast<uintptr_t>(filterRevision)));
                          }
                          ControllerLayerFilter filter(getControllerGroup(controller));
                          PxControllerFilters filters(nullptr, &filter, &filter);
                          return controller.move(disp, minDist, elapsedTime, filters);
                      }))  // ✅
            .function("setPosition", optional_override([](PxController &controller, const PxExtendedVec3 &pos) {
                          if (!std::isfinite(pos.x) || !std::isfinite(pos.y) || !std::isfinite(pos.z)) {
                              return false;
                          }
                          return controller.setPosition(pos);
                      }))                                         // ✅
            .function("getPosition", &PxController::getPosition)  // ✅
            // .function("setFootPosition", &PxController::setFootPosition)  // ✅
            // .function("getFootPosition", &PxController::getFootPosition)  // ✅
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
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape = nullptr;
                          if (actor && actor->getShapes(&shape, 1) == 1 && shape) {
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
            // .function("resize", &PxController::resize)                      // ✅
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
            // .function("setClimbingMode", optional_override([](PxController &ctrl, int mode) {
            //               return static_cast<PxCapsuleController *>(&ctrl)->setClimbingMode(
            //                       PxCapsuleClimbingMode::Enum(mode));
            //           }))  // ✅
            //                /** PxBoxController ✅ */
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
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape = nullptr;
                          if (actor && actor->getShapes(&shape, 1) == 1 && shape) {
                              setUUID(*shape, uuid);
                          }
                      }))  // ✅
            .function("setSceneQueryEnabled", optional_override([](PxController &ctrl, bool enabled) {
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape = nullptr;
                          if (actor && actor->getShapes(&shape, 1) == 1 && shape) {
                              setSceneQueryEnabled(*shape, enabled);
                          }
                      }))
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
