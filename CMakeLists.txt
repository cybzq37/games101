# CMakeList.txt: games101 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.10)

# 生成 compile_commands.json 供 clangd 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project ("games101")

# 配置完成后，将 compile_commands.json 复制到项目根目录（如果存在）
if(CMAKE_EXPORT_COMPILE_COMMANDS)
    set(COMPILE_COMMANDS_JSON "${CMAKE_BINARY_DIR}/compile_commands.json")
    set(COMPILE_COMMANDS_LINK "${CMAKE_SOURCE_DIR}/compile_commands.json")
    # 在配置阶段创建一个自定义目标来复制文件
    add_custom_command(
        OUTPUT ${COMPILE_COMMANDS_LINK}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${COMPILE_COMMANDS_JSON}
            ${COMPILE_COMMANDS_LINK}
        DEPENDS ${COMPILE_COMMANDS_JSON}
        COMMENT "Copying compile_commands.json to project root"
    )
    add_custom_target(copy_compile_commands ALL DEPENDS ${COMPILE_COMMANDS_LINK})
endif()

# 如果支持，请为 MSVC 编译器启用热重载（仅在 MSVC 编译器时设置）
if(MSVC)
  if(POLICY CMP0141)
    cmake_policy(SET CMP0141 NEW)
    set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
  endif()
endif()


set(FFMPEG_ROOT "D:/Devpkg/ffmpeg/ffmpeg-n4.4.5-gpl-amd64-shared")
set(FFMPEG_INCLUDE_DIR "${FFMPEG_ROOT}/include")
set(FFMPEG_LIB_DIR "${FFMPEG_ROOT}/lib")

# OpenCV 配置
set(OpenCV_ROOT "D:/Devpkg/opencv/opencv-4.7.0/build")
# OpenCV_DIR 需要指向包含 OpenCVConfig.cmake 的目录
set(OpenCV_DIR "${OpenCV_ROOT}/x64/vc16/lib")
find_package(OpenCV REQUIRED)

# 检查 OpenCV 目录结构
if(NOT EXISTS "${OpenCV_ROOT}/include")
    message(WARNING "OpenCV include directory not found: ${OpenCV_ROOT}/include")
endif()
if(NOT EXISTS "${OpenCV_ROOT}/x64/vc16/lib")
    message(WARNING "OpenCV lib directory not found: ${OpenCV_ROOT}/x64/vc16/lib")
endif()

# Eigen 配置（头文件库，无需链接）
set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen")
set(EIGEN_INCLUDE_DIR "${EIGEN_ROOT}")

# 检查 Eigen 目录结构
if(NOT EXISTS "${EIGEN_INCLUDE_DIR}/Eigen")
    message(WARNING "Eigen include directory not found: ${EIGEN_INCLUDE_DIR}/Eigen")
else()
    message(STATUS "Found Eigen at: ${EIGEN_INCLUDE_DIR}")
endif()


# 将源代码添加到此项目的可执行文件。
add_executable (games101
    "src/main.cpp"
)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET games101 PROPERTY CXX_STANDARD 20)
endif()

# 使用 target_include_directories 替代全局 include_directories
target_include_directories(games101 PRIVATE ${FFMPEG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIR})
target_link_directories(games101 PRIVATE ${FFMPEG_LIB_DIR})
target_link_libraries(games101 PRIVATE
    avcodec
    avdevice
    avfilter
    avformat
    avutil
    postproc
    swresample
    swscale
    ${OpenCV_LIBS})

# 设置运行时 DLL 路径（Windows）
if(WIN32)
    set(FFMPEG_BIN_DIR "${FFMPEG_ROOT}/bin")
    set(OpenCV_BIN_DIR "${OpenCV_ROOT}/x64/vc16/bin")

    # 检查 FFmpeg bin 目录是否存在
    if(EXISTS "${FFMPEG_BIN_DIR}")
        # 设置 Visual Studio 调试器环境变量（使用正斜杠，CMake 会自动转换）
        set_target_properties(games101 PROPERTIES
            VS_DEBUGGER_ENVIRONMENT "PATH=${FFMPEG_BIN_DIR};${OpenCV_BIN_DIR};$ENV{PATH}"
        )

        # 获取所有需要复制的 FFmpeg DLL 文件列表
        file(GLOB FFMPEG_DLLS "${FFMPEG_BIN_DIR}/*.dll")

        # 复制所有 FFmpeg DLL 到输出目录
        if(FFMPEG_DLLS)
            foreach(DLL ${FFMPEG_DLLS})
                get_filename_component(DLL_NAME ${DLL} NAME)
                add_custom_command(TARGET games101 POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DLL}"
                    $<TARGET_FILE_DIR:games101>
                    COMMENT "Copying ${DLL_NAME} to output directory"
                )
            endforeach()
        else()
            message(WARNING "未找到 FFmpeg DLL 文件在: ${FFMPEG_BIN_DIR}")
        endif()
    else()
        message(WARNING "FFmpeg bin directory not found: ${FFMPEG_BIN_DIR}")
    endif()

    # 检查 OpenCV bin 目录是否存在
    if(EXISTS "${OpenCV_BIN_DIR}")
        # 获取所有需要复制的 OpenCV DLL 文件列表
        file(GLOB OpenCV_DLLS "${OpenCV_BIN_DIR}/*.dll")

        # 复制所有 OpenCV DLL 到输出目录
        if(OpenCV_DLLS)
            foreach(DLL ${OpenCV_DLLS})
                get_filename_component(DLL_NAME ${DLL} NAME)
                add_custom_command(TARGET games101 POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DLL}"
                    $<TARGET_FILE_DIR:games101>
                    COMMENT "Copying ${DLL_NAME} to output directory"
                )
            endforeach()
        else()
            message(WARNING "未找到 OpenCV DLL 文件在: ${OpenCV_BIN_DIR}")
        endif()
    else()
        message(WARNING "OpenCV bin directory not found: ${OpenCV_BIN_DIR}")
    endif()

endif()

# 安装规则
install(TARGETS games101
    RUNTIME DESTINATION bin
    COMPONENT Runtime
)

# 安装配置文件（如果存在）
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.ini")
    install(FILES config.ini
        DESTINATION bin
        COMPONENT Runtime
    )
endif()

# 安装所有 DLL 文件（Windows）
if(WIN32)
    if(FFMPEG_DLLS)
        install(FILES ${FFMPEG_DLLS}
            DESTINATION bin
            COMPONENT Runtime
        )
    endif()
    if(OpenCV_DLLS)
        install(FILES ${OpenCV_DLLS}
            DESTINATION bin
            COMPONENT Runtime
        )
    endif()
endif()
