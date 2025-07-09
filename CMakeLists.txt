cmake_minimum_required (VERSION 3.10)
project(WeldTracker VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/bin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_DEBUG "/MTd")

add_library(project_interface INTERFACE)
target_include_directories(project_interface INTERFACE
	${CMAKE_CURRENT_SOURCE_DIR}/include)


# ================= 核心库模块 =================
# 1. WTrackDType (纯头文件库)
add_library(WTrackDType INTERFACE)
target_sources(WTrackDType INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/WTrackDType.h>
)
target_link_libraries(WTrackDType INTERFACE project_interface)

# 2. TxtMethd
add_library(TxtMethod STATIC)
target_sources(TxtMethod
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/TxtMethod/TxtMethod.h
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TxtMethod/TxtMethod.cpp
)
target_link_libraries(TxtMethod PUBLIC project_interface)

# 3. ImageMethod/Matrix (纯头文件库)
add_library(Matrix INTERFACE)
target_sources(Matrix INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ImageMethod/Matrix.h>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ImageMethod/Matrix.inl>
)
target_link_libraries(Matrix INTERFACE project_interface)

# 4. ImageMethod/PixelToLaserCoord
add_library(PixelToLaserCoord STATIC)
target_sources(PixelToLaserCoord
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/ImageMethod/PixelToLaserCoord.h
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ImageMethod/PixelToLaserCoord.cpp
)
target_link_libraries(PixelToLaserCoord PUBLIC project_interface)

# 5. RobotMethod/LasercoordToTcp
add_library(LaserCoordToTcp STATIC)
target_sources(LaserCoordToTcp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/RobotMethod/LaserCoordToTcp.h
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/RobotMethod/LaserCoordToTcp.cpp
)
target_link_libraries(LaserCoordToTcp PUBLIC project_interface PixelToLaserCoord)

# 6. RobotMethod/TrackAlgMethod
add_library(TrackAlgMethod STATIC)
target_sources(TrackAlgMethod
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include/RobotMethod/TrackAlgMethod.h
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/RobotMethod/TrackAlgMethod.cpp
)
target_link_libraries(TrackAlgMethod PUBLIC project_interface)

# ================= 主应用程序 =================
add_executable(WeldTracker src/WeldTracker.cpp)

target_link_libraries(WeldTracker PRIVATE 
    project_interface
)

# ================= 测试单元 ====================
option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
    
    enable_testing()
    
    message(STATUS "Testing...")

    # 获取 Google Test
    LIST(APPEND CMAKE_PREFIX_PATH "/home/HwHiAiUser/WeldTrackAppCPP/CppLib")
    find_package(GTest REQUIRED)
    message(STATUS "GTEST ${GTEST_INCLUDE_DIRS}")
    message(STATUS "GTEST ${GTEST_LIBRARY_DIRS}")
    
    # 1. 添加 WTrackDType 测试
    add_executable(test_WTrackDType tests/test_WTrackDType.cpp)
    target_link_libraries(test_WTrackDType PRIVATE
        WTrackDType
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME WTrackDTypeTests COMMAND test_WTrackDType)

    # 2. 添加 TxtMethod 测试
    add_executable(test_TxtMethod tests/test_TxtMethod.cpp)
    target_link_libraries(test_TxtMethod PRIVATE
        TxtMethod
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME TxtMethodTests COMMAND test_TxtMethod) 
    
    # 3. 添加 Matrix 测试
    add_executable(test_Matrix tests/test_Matrix.cpp)
    target_link_libraries(test_Matrix PRIVATE
        Matrix
	    GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME MatrixTests COMMAND test_Matrix)

    # 4. 添加 PixelToLaserCoord 测试
    add_executable(test_PixelToLaserCoord tests/test_PixelToLaserCoord.cpp)
    target_link_libraries(test_PixelToLaserCoord PRIVATE
        PixelToLaserCoord
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME PixelToLaserCoordTests COMMAND test_PixelToLaserCoord)

    # 5. 添加 LaserCoordToTcp 测试
    add_executable(test_LaserCoordToTcp tests/test_LaserCoordToTcp.cpp)
    target_link_libraries(test_LaserCoordToTcp PRIVATE
        LaserCoordToTcp
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME LaserCoordToTcpTests COMMAND test_LaserCoordToTcp)

    # 6. 添加 TrackAlgMethod 测试
    add_executable(test_TrackAlgMethod tests/test_TrackAlgMethod.cpp)
    target_link_libraries(test_TrackAlgMethod PRIVATE
        TrackAlgMethod
        GTest::gtest
        GTest::gtest_main
    )
    add_test(NAME TrackAlgMethodTests COMMAND test_TrackAlgMethod)
endif()
