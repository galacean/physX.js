//
// Created by Oasis on 2023/3/21.
//

#pragma once

#if PX_DEBUG || PX_PROFILE || PX_CHECKED

#include <emscripten.h>
#include <emscripten/bind.h>

#include "../BindingHelper.h"
#include "PxPhysicsAPI.h"

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

EMSCRIPTEN_BINDINGS(physx_pvd) {
    class_<PxPvdTransport>("PxPvdTransport")
            .allow_subclass<PxPvdTransportWrapper>("PxPvdTransportWrapper", constructor<>());

    function("PxCreatePvd", &PxCreatePvd, allow_raw_pointers());

    class_<PxPvdInstrumentationFlags>("PxPvdInstrumentationFlags").constructor<int>();
    enum_<PxPvdInstrumentationFlag::Enum>("PxPvdInstrumentationFlag")
            .value("eALL", PxPvdInstrumentationFlag::Enum::eALL)
            .value("eDEBUG", PxPvdInstrumentationFlag::Enum::eDEBUG)
            .value("ePROFILE", PxPvdInstrumentationFlag::Enum::ePROFILE)
            .value("eMEMORY", PxPvdInstrumentationFlag::Enum::eMEMORY);
}

namespace emscripten {
namespace internal {
template <>
void raw_destructor<PxPvd>(PxPvd *) { /* do nothing */
}

template <>
void raw_destructor<PxPvdTransport>(PxPvdTransport *) { /* do nothing */
}

template <>
void raw_destructor<PxPvdSceneClient>(PxPvdSceneClient *) { /* do nothing */
}

}  // namespace internal
}  // namespace emscripten

#endif