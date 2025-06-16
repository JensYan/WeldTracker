cmake_minimum_required(VERSION 3.15)
project("WeldTracker" VERSION 1.0.0 LANGUAGES CXX)

# 设置输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/bin)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 全局包含目录（所有目标共享）
add_library(project_interface INTERFACE)
target_include_directories(project_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 启用测试
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    enable_testing()
    # 查找 GTest
    find_package(GTest REQUIRED)
endif()

# ================= 核心库模块 =================
# 1. WTrackDType 库
add_library(WTrackDType INTERFACE)
target_sources(WTrackDType INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/WTrackDType.h>
)
target_link_libraries(WTrackDType INTERFACE project_interface)

# 2. TxtMethod 库
add_library(TxtMethod STATIC)
target_sources(TxtMethod PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/TxtMethod/TxtMethod.h>
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TxtMethod/TxtMethod.cpp
)
target_link_libraries(TxtMethod PUBLIC project_interface)
# 仅当构建共享库时需要导出符号
# target_compile_definitions(TxtMethod PRIVATE TXTMETHOD_EXPORT)

# ================= 测试目标 =================
if(BUILD_TESTING)
    # WTrackDType 测试
    add_executable(test_WTrackDType tests/test_WTrackDType.cpp)
    target_link_libraries(test_WTrackDType PRIVATE
        WTrackDType
        GTest::gtest_main
    )
    add_test(NAME WTrackDTypeTests COMMAND test_WTrackDType)
    
    # TxtMethod 测试
    add_executable(test_TxtMethod tests/test_TxtMethod.cpp)
    target_link_libraries(test_TxtMethod PRIVATE 
        TxtMethod 
        GTest::gtest_main
    )
    add_test(NAME TxtMethodTests COMMAND test_TxtMethod)
endif()

# ================= 主应用程序 =================
add_executable(WeldTracker src/WeldTracker.cpp)
target_link_libraries(WeldTracker PRIVATE 
    project_interface
    WTrackDType
    TxtMethod
)

# ================= 编译器特定设置 =================
if(MSVC)
    # 为所有目标设置警告选项
    add_compile_options(/W4 /WX /permissive-)
    
    # 禁用特定警告（按需添加）
    add_compile_options(/wd4251) # 禁用dll接口警告
else()
    # GCC/Clang 设置
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ================= 安装规则 =================
install(TARGETS WeldTracker
    RUNTIME DESTINATION bin
    BUNDLE DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)