//
// Created by Oasis on 2023/3/21.
//

#pragma once

#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace emscripten;

EMSCRIPTEN_BINDINGS(physx_controller) {
    enum_<PxCapsuleClimbingMode::Enum>("PxCapsuleClimbingMode")
            .value("eCONSTRAINED", PxCapsuleClimbingMode::Enum::eCONSTRAINED)
            .value("eLAST", PxCapsuleClimbingMode::Enum::eLAST)
            .value("eEASY", PxCapsuleClimbingMode::Enum::eEASY);
    enum_<PxControllerShapeType::Enum>("PxControllerShapeType")
            .value("eBOX", PxControllerShapeType::Enum::eBOX)
            .value("eCAPSULE", PxControllerShapeType::Enum::eCAPSULE);
    enum_<PxControllerNonWalkableMode::Enum>("PxControllerNonWalkableMode")
            .value("ePREVENT_CLIMBING", PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING)
            .value("ePREVENT_CLIMBING_AND_FORCE_SLIDING",
                   PxControllerNonWalkableMode::Enum::ePREVENT_CLIMBING_AND_FORCE_SLIDING);
    enum_<PxControllerCollisionFlag::Enum>("PxControllerCollisionFlag")
            .value("eCOLLISION_SIDES", PxControllerCollisionFlag::Enum::eCOLLISION_SIDES)
            .value("eCOLLISION_UP", PxControllerCollisionFlag::Enum::eCOLLISION_UP)
            .value("eCOLLISION_DOWN", PxControllerCollisionFlag::Enum::eCOLLISION_DOWN);
    class_<PxControllerCollisionFlags>("PxControllerCollisionFlags")
            .constructor<int>()
            .function("isSet", &PxControllerCollisionFlags::isSet);
    /** PhysXCharacterControllerManager ✅ */
    class_<PxControllerManager>("PxControllerManager")
            .function("purgeControllers", &PxControllerManager::purgeControllers)                              // ✅
            .function("createController", &PxControllerManager::createController, allow_raw_pointers())        // ✅
            .function("computeInteractions", &PxControllerManager::computeInteractions, allow_raw_pointers())  // ✅
            .function("setTessellation", &PxControllerManager::setTessellation)                                // ✅
            .function("setOverlapRecoveryModule", &PxControllerManager::setOverlapRecoveryModule)              // ✅
            .function("setPreciseSweeps", &PxControllerManager::setPreciseSweeps)                              // ✅
            .function("setPreventVerticalSlidingAgainstCeiling",
                      &PxControllerManager::setPreventVerticalSlidingAgainstCeiling)  // ✅
            .function("shiftOrigin", &PxControllerManager::shiftOrigin);              // ✅
    /** PhysXCharacterControllerDesc ✅ */
    class_<PxControllerDesc>("PxControllerDesc")
            .function("isValid", &PxControllerDesc::isValid)
            .function("getType", &PxControllerDesc::getType)                          // ✅
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
            .function("isValid", &PxCapsuleControllerDesc::isValid)
            .function("setToDefault", &PxCapsuleControllerDesc::setToDefault)   // ✅
            .property("radius", &PxCapsuleControllerDesc::radius)               // ✅
            .property("height", &PxCapsuleControllerDesc::height)               // ✅
            .property("climbingMode", &PxCapsuleControllerDesc::climbingMode);  // ✅
    /** PhysXBoxCharacterControllerDesc ✅ */
    class_<PxBoxControllerDesc, base<PxControllerDesc>>("PxBoxControllerDesc")
            .constructor<>()
            .function("isValid", &PxBoxControllerDesc::isValid)
            .function("setToDefault", &PxBoxControllerDesc::setToDefault)            // ✅
            .property("halfForwardExtent", &PxBoxControllerDesc::halfForwardExtent)  // ✅
            .property("halfHeight", &PxBoxControllerDesc::halfHeight)                // ✅
            .property("halfSideExtent", &PxBoxControllerDesc::halfSideExtent);       // ✅

    /** PhysXCharacterController ✅ */
    class_<PxController>("PxController")
            .function("release", optional_override([](PxController &ctrl) {
                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape;
                          actor->getShapes(&shape, 1);
                          free(shape->userData);
                          ctrl.release();
                      }))
            .function("isSetControllerCollisionFlag",
                      optional_override([](PxController &controller, PxControllerCollisionFlags &flags, int flag) {
                          return flags.isSet(PxControllerCollisionFlag::Enum(flag));
                      }))  // ✅
            .function("move", optional_override([](PxController &controller, const PxVec3 &disp, PxF32 minDist,
                                                   PxF32 elapsedTime) -> uint32_t {
                          return controller.move(disp, minDist, elapsedTime, PxControllerFilters());
                      }))                                         // ✅
            .function("setPosition", &PxController::setPosition)  // ✅
            .function("getPosition", &PxController::getPosition)
            .function("setFootPosition", &PxController::setFootPosition)  // ✅
            .function("getFootPosition", &PxController::getFootPosition)  // ✅
            .function("setStepOffset", &PxController::setStepOffset)      // ✅
            .function("setNonWalkableMode", optional_override([](PxController &controller, int mode) {
                          return controller.setNonWalkableMode(PxControllerNonWalkableMode::Enum(mode));
                      }))                                                   // ✅
            .function("setContactOffset", &PxController::setContactOffset)  // ✅
            .function("setUpDirection", &PxController::setUpDirection)      // ✅
            .function("setSlopeLimit", &PxController::setSlopeLimit)        // ✅
            .function("invalidateCache", &PxController::invalidateCache)    // ✅
            .function("resize", &PxController::resize)                      // ✅
            /** PxCapsuleController ✅ */
            .function("setRadius", optional_override([](PxController &ctrl, PxF32 radius) {
                          static_cast<PxCapsuleController *>(&ctrl)->setRadius(radius);
                      }))  // ✅
            .function("setHeight", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxCapsuleController *>(&ctrl)->setHeight(height);
                      }))  // ✅
            .function("setClimbingMode", optional_override([](PxController &ctrl, int mode) {
                          return static_cast<PxCapsuleController *>(&ctrl)->setClimbingMode(
                                  PxCapsuleClimbingMode::Enum(mode));
                      }))  // ✅
                           /** PxBoxController ✅ */
            .function("setHalfHeight", optional_override([](PxController &ctrl, PxF32 radius) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfHeight(radius);
                      }))  // ✅
            .function("setHalfSideExtent", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfSideExtent(height);
                      }))  // ✅
            .function("setHalfForwardExtent", optional_override([](PxController &ctrl, PxF32 height) {
                          static_cast<PxBoxController *>(&ctrl)->setHalfForwardExtent(height);
                      }))
            .function("setUUID", optional_override([](PxController &ctrl, uint32_t uuid) {
                          auto ptr = malloc(sizeof(uint32_t));
                          memcpy(ptr, &uuid, sizeof(uint32_t));

                          PxRigidDynamic *actor = ctrl.getActor();
                          PxShape *shape;
                          actor->getShapes(&shape, 1);
                          shape->userData = ptr;
                      }));  // ✅
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