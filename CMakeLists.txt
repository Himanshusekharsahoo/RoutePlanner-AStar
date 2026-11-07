cmake_minimum_required(VERSION 3.10)

# Define the project
project(OSM_A_star_search CXX)

# Specify C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Support vcpkg toolchain if VCPKG_ROOT environment variable is defined and toolchain not passed via CLI
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{VCPKG_ROOT} AND EXISTS "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Path to vcpkg toolchain file")
endif()

# Find pkgconf
find_package(PkgConfig QUIET)

# Use pkgconf to find GraphicsMagick
if(PKG_CONFIG_FOUND)
    pkg_check_modules(GraphicsMagick QUIET GraphicsMagick)
    if(GraphicsMagick_FOUND)
        include_directories(${GraphicsMagick_INCLUDE_DIRS})
        link_directories(${GraphicsMagick_LIBRARY_DIRS})
    endif()

    # Use pkgconf to find cairo
    pkg_check_modules(cairo QUIET cairo)
    if(cairo_FOUND)
        include_directories(${cairo_INCLUDE_DIRS})
        link_directories(${cairo_LIBRARY_DIRS})
    endif()
endif()

# Manually define the Cairo::Cairo target
if(cairo_FOUND AND NOT TARGET Cairo::Cairo)
    add_library(Cairo::Cairo INTERFACE IMPORTED)
    set_target_properties(Cairo::Cairo PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${cairo_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${cairo_LIBRARIES}"
    )
endif()

# Locate pugixml: find_package or fallback to bundled thirdparty/pugixml
find_package(pugixml QUIET)
if(NOT TARGET pugixml::pugixml AND NOT TARGET pugixml)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pugixml/CMakeLists.txt")
        add_subdirectory(thirdparty/pugixml EXCLUDE_FROM_ALL)
        if(NOT TARGET pugixml::pugixml AND TARGET pugixml)
            add_library(pugixml::pugixml ALIAS pugixml)
        endif()
    endif()
endif()

find_package(Iconv QUIET)
find_package(io2d QUIET)

# Add the main project executable (if io2d is available)
if(TARGET io2d::io2d AND TARGET Cairo::Cairo)
    add_executable(OSM_A_star_search 
        src/main.cpp
        src/model.cpp
        src/render.cpp
        src/route_model.cpp
        src/route_planner.cpp
    )

    # Link the required libraries to the executable
    target_link_libraries(OSM_A_star_search
        PRIVATE io2d::io2d
        PUBLIC pugixml::pugixml
        PUBLIC Cairo::Cairo
        PUBLIC ${GraphicsMagick_LIBRARIES}
        PRIVATE Iconv::Iconv
    )

    if(MSVC)
        target_compile_options(OSM_A_star_search PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
    endif()

    if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        target_link_libraries(OSM_A_star_search PUBLIC pthread)
    endif()
else()
    message(STATUS "io2d not found; GUI target OSM_A_star_search will not be built. Tests remain fully functional.")
endif()

# ==============================================================================
# Automated Testing Infrastructure (Phase 2)
# ==============================================================================
enable_testing()

# Integrate bundled GoogleTest framework from thirdparty/googletest
if(NOT TARGET gtest_main)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/googletest/CMakeLists.txt")
        set(BUILD_GMOCK OFF CACHE BOOL "Build GMock" FORCE)
        set(INSTALL_GTEST OFF CACHE BOOL "Install GTest" FORCE)
        set(gtest_force_shared_crt ON CACHE BOOL "Use shared CRT" FORCE)
        add_subdirectory(thirdparty/googletest EXCLUDE_FROM_ALL)
    endif()
endif()

# Add the dedicated unit test executable
add_executable(route_planner_tests
    test/utest_rp_a_star_search.cpp
    src/model.cpp
    src/route_model.cpp
    src/route_planner.cpp
)

target_include_directories(route_planner_tests PRIVATE src)

# Link only algorithm-required dependencies (no GUI / render / cairo / GraphicsMagick)
target_link_libraries(route_planner_tests
    PRIVATE gtest_main
    PUBLIC pugixml::pugixml
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(route_planner_tests PUBLIC pthread)
endif()

# Register with CTest
add_test(NAME route_planner_tests COMMAND route_planner_tests)
set_tests_properties(route_planner_tests PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
