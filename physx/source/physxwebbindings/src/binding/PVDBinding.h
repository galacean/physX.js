//
// Created by Oasis on 2023/3/21.
//

#pragma once

#if PX_DEBUG || PX_PROFILE || PX_CHECKED

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"
#include <emscripten.h>
#include <emscripten/bind.h>

using namespace physx;
using namespace emscripten;

struct PxPvdTransportWrapper : public wrapper<PxPvdTransport> {
    EMSCRIPTEN_WRAPPER(PxPvdTransportWrapper)

    void unlock() override {}

    void flush() override {}

    void release() override {}

    PxPvdTransport &lock() override { return *this; }

    uint64_t getWrittenDataSize() override { return 0; }

    bool connect() override { return call<bool>("connect"); }

    void disconnect() override { call<void>("disconnect"); }

    bool isConnected() override { return call<bool>("isConnected"); }

    bool write(const uint8_t *inBytes, uint32_t inLength) override {
        return call<bool>("write", int(inBytes), int(inLength));
    }
};

#endif