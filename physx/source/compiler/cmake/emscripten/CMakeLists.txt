##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##  * Redistributions of source code must retain the above copyright
##    notice, this list of conditions and the following disclaimer.
##  * Redistributions in binary form must reproduce the above copyright
##    notice, this list of conditions and the following disclaimer in the
##    documentation and/or other materials provided with the distribution.
##  * Neither the name of NVIDIA CORPORATION nor the names of its
##    contributors may be used to endorse or promote products derived
##    from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
## EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
## PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
## CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
## EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
## PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
## PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
## OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
## (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
## OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##
## Copyright (c) 2018-2019 NVIDIA Corporation. All rights reserved.

STRING(TOLOWER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_LOWERCASE)

SET(CMAKE_STATIC_LIBRARY_PREFIX "")

# SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-exceptions -ffunction-sections -fdata-sections -fstrict-aliasing -msimd128 ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")
SET(PHYSX_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fno-exceptions -ffunction-sections -fdata-sections -fstrict-aliasing --bind ${CLANG_WARNINGS}" CACHE INTERNAL "PhysX CXX")

SET(CMAKE_CXX_FLAGS ${PHYSX_CXX_FLAGS})

SET(PHYSX_EMSCRIPTEN_COMPILE_DEFS "${CUDA_FLAG};${PHYSX_AUTOBUILD}" CACHE INTERNAL "Base PhysX preprocessor definitions")

SET(PHYSX_EMSCRIPTEN_DEBUG_COMPILE_DEFS   "NDEBUG;PX_DEBUG=1;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1"  CACHE INTERNAL "Debug PhysX preprocessor definitions")
SET(PHYSX_EMSCRIPTEN_CHECKED_COMPILE_DEFS "NDEBUG;PX_CHECKED=1;${NVTX_FLAG};PX_SUPPORT_PVD=1" CACHE INTERNAL "Checked PhysX preprocessor definitions")
SET(PHYSX_EMSCRIPTEN_PROFILE_COMPILE_DEFS "NDEBUG;PX_PROFILE=1;${NVTX_FLAG};PX_SUPPORT_PVD=1"  CACHE INTERNAL "Profile PhysX preprocessor definitions")
SET(PHYSX_EMSCRIPTEN_RELEASE_COMPILE_DEFS "NDEBUG;PX_SUPPORT_PVD=0" CACHE INTERNAL "Release PhysX preprocessor definitions")

# TODO: Turn on SIMD 
ADD_DEFINITIONS(-DPX_SIMD_DISABLED)


INCLUDE(PhysXFoundation.cmake)
INCLUDE(LowLevel.cmake)
INCLUDE(LowLevelAABB.cmake)
INCLUDE(LowLevelDynamics.cmake)
INCLUDE(PhysX.cmake)
INCLUDE(PhysXCharacterKinematic.cmake)
INCLUDE(PhysXCommon.cmake)
INCLUDE(PhysXCooking.cmake)
INCLUDE(PhysXExtensions.cmake)
INCLUDE(PhysXVehicle.cmake)
INCLUDE(SceneQuery.cmake)
INCLUDE(SimulationController.cmake)
INCLUDE(FastXml.cmake)
INCLUDE(PhysXPvdSDK.cmake)
INCLUDE(PhysXTask.cmake)
INCLUDE(emscripten/PhysXWebBindings.cmake)

# Set folder PhysX SDK to all common SDK source projects
SET_PROPERTY(TARGET PhysX PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCharacterKinematic PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCommon PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXCooking PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXExtensions PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXVehicle PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevel PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelAABB PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET LowLevelDynamics PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SceneQuery PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET SimulationController PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET FastXml PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXPvdSDK PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXTask PROPERTY FOLDER "PhysX SDK")
SET_PROPERTY(TARGET PhysXFoundation PROPERTY FOLDER "PhysX SDK")	
SET_PROPERTY(TARGET PhysXWebBindings PROPERTY FOLDER "PhysX SDK")	

SET(PHYSXDISTRO_LIBS PhysXFoundation PhysX PhysXCharacterKinematic PhysXPvdSDK PhysXCommon PhysXCooking PhysXExtensions PhysXVehicle PhysXWebBindings)

INSTALL(
	TARGETS ${PHYSXDISTRO_LIBS}
	EXPORT PhysXSDK
	DESTINATION $<$<CONFIG:debug>:${PX_ROOT_LIB_DIR}/debug>$<$<CONFIG:release>:${PX_ROOT_LIB_DIR}/release>$<$<CONFIG:checked>:${PX_ROOT_LIB_DIR}/checked>$<$<CONFIG:profile>:${PX_ROOT_LIB_DIR}/profile>	
	)

