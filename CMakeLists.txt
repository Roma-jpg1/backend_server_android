cmake_minimum_required(VERSION 3.14)
project(BACKEND_SERVER_ANDROID)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)


# ---- SDL2 ----
find_package(SDL2 REQUIRED HINTS /opt/homebrew /usr/local)

# ---- ZMQ (libzmq) ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
pkg_check_modules(PNG REQUIRED libpng)
pkg_check_modules(GLEW REQUIRED glew)

find_path(CPPZMQ_INCLUDE_DIR zmq.hpp
    PATHS /opt/homebrew/include /usr/local/include
          /opt/homebrew/opt/cppzmq/include
)
if(NOT CPPZMQ_INCLUDE_DIR)
    message(FATAL_ERROR "cppzmq header zmq.hpp not found. Install cppzmq or add its include path.")
endif()

# ---- PostgreSQL client (libpq) ----
find_path(LIBPQ_INCLUDE_DIR libpq-fe.h
    PATHS /usr/include/postgresql /usr/local/include/postgresql
          /opt/homebrew/include /opt/homebrew/include/postgresql@18
          /opt/homebrew/Cellar/libpq/*/include
)
find_library(LIBPQ_LIBRARY pq
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
          /opt/homebrew/lib /opt/homebrew/opt/libpq/lib
          /opt/homebrew/Cellar/libpq/*/lib
)

# ---- OpenGL / GLEW ----
find_package(OpenGL REQUIRED)

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

if(EXISTS "${THIRD_PARTY_DIR}/json/CMakeLists.txt")
    add_subdirectory(${THIRD_PARTY_DIR}/json)
else()
    find_package(nlohmann_json REQUIRED)
endif()

# ---- Main executable ----
add_executable(main ${SRC_DIR}/main.cpp)

target_include_directories(main PRIVATE
    ${INCLUDE_DIR}
    ${ZMQ_INCLUDE_DIRS}
    ${CPPZMQ_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}
    ${LIBPQ_INCLUDE_DIR}
    ${PNG_INCLUDE_DIRS}
)

target_link_directories(main PRIVATE
    ${PNG_LIBRARY_DIRS}
    ${ZMQ_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
)

target_link_libraries(main PRIVATE
    imgui
    implot
    nlohmann_json::nlohmann_json
    ${SDL2_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${PNG_LIBRARIES}
    ${ZMQ_LIBRARIES}
    ${LIBPQ_LIBRARY}
    pthread
)

target_compile_options(main PRIVATE ${ZMQ_CFLAGS_OTHER} ${PNG_CFLAGS_OTHER} ${GLEW_CFLAGS_OTHER})
