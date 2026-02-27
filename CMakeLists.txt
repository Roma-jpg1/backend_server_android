cmake_minimum_required(VERSION 3.14)
project(BACKEND_SERVER_ANDROID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)


# ---- SDL2 ----
find_package(SDL2 REQUIRED)

# ---- ZMQ (libzmq) ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# ---- OpenGL / GLEW ----
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# ---- ImGui ----
add_library(imgui STATIC
    ${THIRD_PARTY_DIR}/imgui/imgui.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_draw.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_demo.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_tables.cpp
    ${THIRD_PARTY_DIR}/imgui/imgui_widgets.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_sdl2.cpp
    ${THIRD_PARTY_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${THIRD_PARTY_DIR}/imgui
    ${THIRD_PARTY_DIR}/imgui/backends
    ${SDL2_INCLUDE_DIRS}
)

target_compile_options(imgui PRIVATE -fPIC)

# ---- ImPlot ----
add_library(implot STATIC
    ${THIRD_PARTY_DIR}/implot/implot.cpp
    ${THIRD_PARTY_DIR}/implot/implot_demo.cpp
    ${THIRD_PARTY_DIR}/implot/implot_items.cpp
)

target_include_directories(implot PUBLIC
    ${THIRD_PARTY_DIR}/implot
)

target_link_libraries(implot PRIVATE imgui)
target_compile_options(implot PRIVATE -fPIC)

add_subdirectory(${THIRD_PARTY_DIR}/json)


# ---- Main executable ----
add_executable(main ${SRC_DIR}/main.cpp)

target_include_directories(main PRIVATE
    ${INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIRS}
)

target_link_libraries(main PRIVATE
    imgui
    implot
    nlohmann_json::nlohmann_json
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${ZMQ_LIBRARIES}
    pthread
)

target_compile_options(main PRIVATE ${ZMQ_CFLAGS_OTHER})
