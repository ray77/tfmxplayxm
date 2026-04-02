cmake_minimum_required(VERSION 3.5)
project(tfmxplay VERSION 0.0.9)

# are you kidding me?! macOS 10.15 and still no C++11 by default?!
set(CMAKE_CXX_STANDARD 11)

option(STATIC_BUILD "Build fully static executable (no external dylibs)" OFF)

if (WIN32 AND MINGW)
  if (STATIC_BUILD)
    find_library(HAVE_SDL2 libSDL2.a)
  else()
    find_library(HAVE_SDL2 SDL2)
  endif()
  find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2)
elseif (WIN32)
  add_subdirectory(SDL)
  set(HAVE_SDL2 SDL2-static)
elseif (UNIX AND NOT APPLE AND STATIC_BUILD)
  set(SDL2_STATIC_LIB "" CACHE FILEPATH "Path to SDL2 static library (libSDL2.a)")
  if (NOT SDL2_INCLUDE_DIR)
    find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2)
  endif()
elseif (APPLE AND STATIC_BUILD)
  set(SDL2_STATIC_LIB "${CMAKE_BINARY_DIR}/libSDL2-universal.a" CACHE FILEPATH
      "Path to universal (arm64+x86_64) SDL2 static library")
  if (NOT SDL2_INCLUDE_DIR)
    find_path(SDL2_INCLUDE_DIR SDL.h
      HINTS /opt/homebrew/include /usr/local/include
      PATH_SUFFIXES SDL2)
  endif()
else()
  if (APPLE AND CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
    set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH OFF)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
  endif()
  find_library(HAVE_SDL2 SDL2)
  find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2 NO_CMAKE_SYSTEM_PATH)
  if (NOT SDL2_INCLUDE_DIR AND CMAKE_PREFIX_PATH)
    find_path(SDL2_INCLUDE_DIR SDL.h HINTS ${CMAKE_PREFIX_PATH} PATH_SUFFIXES include/SDL2 include)
  endif()
  if (NOT SDL2_INCLUDE_DIR)
    find_path(SDL2_INCLUDE_DIR SDL.h PATH_SUFFIXES SDL2)
  endif()
  find_library(HAVE_DRM drm)
  set(CMAKE_FIND_USE_CMAKE_SYSTEM_PATH ON)
endif()

add_executable(tfmxplay src/blip_buf.c src/sinc.cpp src/tfmx.cpp src/tfmxplay.cpp src/time.cpp src/convert2xm.cpp)
target_compile_definitions(tfmxplay PRIVATE TFMXPLAY_VERSION="${PROJECT_VERSION}")

if (HAVE_DRM)
  target_compile_definitions(tfmxplay PUBLIC _SYNC_VBLANK)
  target_link_libraries(tfmxplay ${HAVE_DRM})
endif()

if (SDL2_INCLUDE_DIR)
  get_filename_component(SDL2_INCLUDE_PARENT "${SDL2_INCLUDE_DIR}" DIRECTORY)
  target_include_directories(tfmxplay PRIVATE ${SDL2_INCLUDE_DIR} ${SDL2_INCLUDE_PARENT})
endif()

if (WIN32 AND MINGW)
  if (STATIC_BUILD)
    target_link_libraries(tfmxplay -static -static-libgcc -static-libstdc++
      mingw32 SDL2main ${HAVE_SDL2}
      dinput8 dxguid dxerr8 hid cfgmgr32 uuid
      imm32 ole32 oleaut32 version winmm gdi32 setupapi user32 advapi32 shell32)
  else()
    target_link_libraries(tfmxplay mingw32 SDL2main ${HAVE_SDL2})
  endif()
elseif (WIN32)
  target_link_libraries(tfmxplay SDL2main ${HAVE_SDL2})
elseif (UNIX AND NOT APPLE AND STATIC_BUILD)
  target_link_libraries(tfmxplay
    -static-libgcc -static-libstdc++
    -Wl,-Bstatic ${SDL2_STATIC_LIB}
    -Wl,-Bdynamic -lpthread -ldl -lm
    -lasound -lpulse -lpulse-simple
    -lX11 -lXext -lXi -lXfixes -lXrandr -lXcursor -lxkbcommon
    -lwayland-client -lwayland-cursor -lwayland-egl
    -lEGL -lGL -lgbm -ldrm
    -ldbus-1)
elseif (APPLE AND STATIC_BUILD)
  target_link_libraries(tfmxplay
    ${SDL2_STATIC_LIB}
    "-framework CoreAudio" "-framework AudioToolbox"
    "-framework CoreVideo" "-framework Cocoa"
    "-framework Carbon" "-framework IOKit"
    "-framework ForceFeedback"
    "-weak_framework CoreHaptics"
    "-weak_framework GameController"
    "-weak_framework QuartzCore"
    "-weak_framework Metal"
    -lobjc -lm -liconv)
else()
  target_link_libraries(tfmxplay ${HAVE_SDL2})
endif()
