cmake_minimum_required(VERSION 3.10)

# Define the project
project(OSM_A_star_search)

# Specify the vcpkg toolchain file for dependency management
set(CMAKE_TOOLCHAIN_FILE "C:/Users/HP/Desktop/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

# Add vcpkg's installed libraries to the search path
list(APPEND CMAKE_PREFIX_PATH "C:/Users/HP/Desktop/vcpkg/installed/x64-windows/share")

# Find pkgconf
find_package(PkgConfig REQUIRED)

# Use pkgconf to find GraphicsMagick
pkg_check_modules(GraphicsMagick REQUIRED GraphicsMagick)
include_directories(${GraphicsMagick_INCLUDE_DIRS})
link_directories(${GraphicsMagick_LIBRARY_DIRS})

# Use pkgconf to find cairo
pkg_check_modules(cairo REQUIRED cairo)
include_directories(${cairo_INCLUDE_DIRS})
link_directories(${cairo_LIBRARY_DIRS})

# Manually define the Cairo::Cairo target
if(NOT TARGET Cairo::Cairo)
    add_library(Cairo::Cairo INTERFACE IMPORTED)
    set_target_properties(Cairo::Cairo PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${cairo_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "${cairo_LIBRARIES}"
    )
endif()

# Locate required packages
find_package(pugixml REQUIRED)
find_package(Iconv REQUIRED)
find_package(io2d REQUIRED)

# Add the main project executable and specify source files
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

# Add a testing executable (optional, if testing is part of your project)
add_executable(test 
    test/utest_rp_a_star_search.cpp
    src/model.cpp 
    src/render.cpp 
    src/route_model.cpp 
    src/route_planner.cpp
)

# Link testing libraries
target_link_libraries(test 
    PRIVATE gtest_main 
    PUBLIC pugixml::pugixml
    PUBLIC Cairo::Cairo
    PUBLIC ${GraphicsMagick_LIBRARIES}
    PRIVATE Iconv::Iconv
)

# Set platform-specific options
if(MSVC)
    target_compile_options(OSM_A_star_search PUBLIC /D_SILENCE_CXX17_ALLOCATOR_VOID_DEPRECATION_WARNING /wd4459)
endif()

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    target_link_libraries(OSM_A_star_search PUBLIC pthread)
endif()