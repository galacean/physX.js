#pragma once

#include "../BindingHelper.h"
#include <emscripten.h>
#include <emscripten/bind.h>

EM_JS_DEPS(contactModificationErrors, "$UTF8ToString");
EM_JS(void, throwContactModificationError, (const char *message), {
    throw new Error(UTF8ToString(message));
});

class PxContactModificationCallback final : public physx::PxContactModifyCallback,
                                            public physx::PxCCDContactModifyCallback {
public:
    explicit PxContactModificationCallback(emscripten::val callback) : mCallback(std::move(callback)) {}

    void onContactModify(physx::PxContactModifyPair *pairs, physx::PxU32 count) override {
        dispatch(pairs, count, false);
    }

    void onCCDContactModify(physx::PxContactModifyPair *pairs, physx::PxU32 count) override {
        dispatch(pairs, count, true);
    }

    physx::PxU32 getPairCount() const { return mPairCount; }
    bool isCCD() const { return mCCD; }

    physx::PxU32 getColliderId(double pairIndex, double shapeIndex) const {
        return getUUID(getPair(pairIndex).shape[getShapeIndex(shapeIndex)]);
    }

    physx::PxU32 getContactCount(double pairIndex) const {
        return getPair(pairIndex).contacts.size();
    }

    bool getPoint(double pairIndex, double contactIndex, const emscripten::val &out) const {
        physx::PxU32 index;
        const auto *contacts = findContact(pairIndex, contactIndex, index);
        if (!contacts) return false;
        const auto &point = contacts->getPoint(index);
        out.call<void>("set", point.x, point.y, point.z);
        return true;
    }

    bool getNormal(double pairIndex, double contactIndex, const emscripten::val &out) const {
        physx::PxU32 index;
        const auto *contacts = findContact(pairIndex, contactIndex, index);
        if (!contacts) return false;
        const auto &normal = contacts->getNormal(index);
        // Sweeps generate normals in the opposite direction to discrete contacts.
        const physx::PxReal sign = mCCD ? -1.0f : 1.0f;
        out.call<void>("set", sign * normal.x, sign * normal.y, sign * normal.z);
        return true;
    }

    void setPoint(double pairIndex, double contactIndex, physx::PxReal x, physx::PxReal y, physx::PxReal z) const {
        const physx::PxVec3 point(x, y, z);
        if (!point.isFinite()) fail("Contact point must be finite.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setPoint(index, point);
    }

    void setNormal(double pairIndex, double contactIndex, physx::PxReal x, physx::PxReal y, physx::PxReal z) const {
        const physx::PxVec3 normal(x, y, z);
        if (!normal.isNormalized()) fail("Contact normal must be a unit vector.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setNormal(index, mCCD ? -normal : normal);
    }

    void setMaxImpulse(double pairIndex, double contactIndex, physx::PxReal impulse) const {
        if (!physx::PxIsFinite(impulse) || impulse < 0) fail("Maximum contact impulse must be finite and nonnegative.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setMaxImpulse(index, impulse);
    }

    void ignoreContact(double pairIndex, double contactIndex) const {
        // Swept CCD reads maximum impulse only if setMaxImpulse marks the patch.
        setMaxImpulse(pairIndex, contactIndex, 0);
    }

    void setRestitution(double pairIndex, double contactIndex, physx::PxReal restitution) const {
        if (!(restitution >= 0 && restitution <= 1)) fail("Contact restitution must be between 0 and 1.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setRestitution(index, restitution);
    }

    physx::PxReal getSeparation(double pairIndex, double contactIndex) const {
        requireDiscrete();
        physx::PxU32 index;
        const auto &contacts = getContact(pairIndex, contactIndex, index);
        return contacts.getSeparation(index);
    }

    void setSeparation(double pairIndex, double contactIndex, physx::PxReal separation) const {
        requireDiscrete();
        if (!physx::PxIsFinite(separation)) fail("Contact separation must be finite.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setSeparation(index, separation);
    }

    void setTargetVelocity(double pairIndex, double contactIndex, physx::PxReal x, physx::PxReal y, physx::PxReal z) const {
        requireDiscrete();
        const physx::PxVec3 velocity(x, y, z);
        if (!velocity.isFinite()) fail("Contact target velocity must be finite.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        contacts.setTargetVelocity(index, velocity);
    }

    void setFriction(double pairIndex, double contactIndex, physx::PxReal friction,
                     void (physx::PxContactSet::*setter)(physx::PxU32, physx::PxReal)) const {
        requireDiscrete();
        if (!physx::PxIsFinite(friction) || friction < 0) fail("Contact friction must be finite and nonnegative.");
        physx::PxU32 index;
        auto &contacts = getContact(pairIndex, contactIndex, index);
        (contacts.*setter)(index, friction);
    }

    void setInverseMassScale(double pairIndex, double shapeIndex, physx::PxReal scale) const {
        requireDiscrete();
        requireScale(scale);
        auto &contacts = getPair(pairIndex).contacts;
        if (getShapeIndex(shapeIndex) == 0) contacts.setInvMassScale0(scale);
        else contacts.setInvMassScale1(scale);
    }

    void setInverseInertiaScale(double pairIndex, double shapeIndex, physx::PxReal scale) const {
        requireDiscrete();
        requireScale(scale);
        auto &contacts = getPair(pairIndex).contacts;
        if (getShapeIndex(shapeIndex) == 0) contacts.setInvInertiaScale0(scale);
        else contacts.setInvInertiaScale1(scale);
    }

    static void setSceneCallback(physx::PxScene &scene, PxContactModificationCallback *callback) {
        if (callback && scene.getCpuDispatcher()->getWorkerCount() != 0) {
            fail("Contact modification requires a scene with zero physics worker threads.");
        }
        scene.setContactModifyCallback(callback);
        if (scene.getContactModifyCallback() != callback) {
            fail("Contact modification callback cannot change during simulation.");
        }
        scene.setCCDContactModifyCallback(callback);
    }

private:
    [[noreturn]] static void fail(const char *message) {
        throwContactModificationError(message);
        __builtin_unreachable();
    }

    static void requireScale(physx::PxReal scale) {
        if (!(scale >= 0 && scale < PX_MAX_F32)) fail("Contact mass or inertia scale must be finite and nonnegative.");
    }

    void requireDiscrete() const {
        if (mCCD) fail("This operation requires a discrete contact.");
    }

    static physx::PxU32 getShapeIndex(double value) {
        physx::PxU32 index;
        if (!tryParseIntRange(value, 0, 1, index)) fail("Contact shape index must be 0 or 1.");
        return index;
    }

    physx::PxContactModifyPair *findPair(double pairIndex) const {
        physx::PxU32 index;
        return mPairCount && tryParseIntRange(pairIndex, 0, mPairCount - 1, index) ? mPairs + index : nullptr;
    }

    physx::PxContactModifyPair &getPair(double pairIndex) const {
        if (auto *pair = findPair(pairIndex)) return *pair;
        fail("Contact pair index is out of range or the callback has ended.");
    }

    physx::PxContactSet *findContact(double pairIndex, double contactIndex, physx::PxU32 &index) const {
        auto *pair = findPair(pairIndex);
        if (!pair) return nullptr;
        const auto count = pair->contacts.size();
        return count && tryParseIntRange(contactIndex, 0, count - 1, index) ? &pair->contacts : nullptr;
    }

    physx::PxContactSet &getContact(double pairIndex, double contactIndex, physx::PxU32 &index) const {
        if (auto *contacts = findContact(pairIndex, contactIndex, index)) return *contacts;
        fail("Contact index is out of range or the callback has ended.");
    }

    void dispatch(physx::PxContactModifyPair *pairs, physx::PxU32 count, bool ccd) {
        mPairs = pairs;
        mPairCount = count;
        mCCD = ccd;
        mCallback();
        mPairs = nullptr;
        mPairCount = 0;
        mCCD = false;
    }

    emscripten::val mCallback;
    physx::PxContactModifyPair *mPairs = nullptr;
    physx::PxU32 mPairCount = 0;
    bool mCCD = false;
};
