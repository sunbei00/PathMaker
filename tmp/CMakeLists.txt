cmake_minimum_required(VERSION 3.28)
project(PathMaker)

set(CMAKE_CXX_STANDARD 17)

# --------- third party lib -------
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(PCL REQUIRED)
# ---------------------------------

# --------- IMGUI -----------------
set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/ThirdParty/imgui/)
set(IMGUI_FILE_DIALOG_DIR ${CMAKE_SOURCE_DIR}/ThirdParty/ImGuiFileDialog/)
file(GLOB IMGUI_SOURCES
        ${IMGUI_DIR}/*.cpp
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        ${IMGUI_FILE_DIALOG_DIR}/*.cpp
)
# ---------------------------------

# --------- OpenGL ------------------
set(GLAD_DIR ${CMAKE_SOURCE_DIR}/ThirdParty/glad/)
file(GLOB GLAD_SOURCES
        ${GLAD_DIR}/glad.c
)
# ---------------------------------


set(THIRD_PARTY_INCLUDE_DIRECTORY
        ${CMAKE_SOURCE_DIR}/ThirdParty/
        ${CMAKE_SOURCE_DIR}/ThirdParty/imgui/
        ${CMAKE_SOURCE_DIR}/ThirdParty/ImGuiFileDialog/
        ${CMAKE_SOURCE_DIR}/ThirdParty/glad/
        ${CMAKE_SOURCE_DIR}/ThirdParty/glm/
)

set(THIRD_PARTY_SRC_DIRECTORY
        ${IMGUI_SOURCES}
        ${GLAD_SOURCES}
        FileIO.h
)

add_executable(PathMaker main.cpp ${THIRD_PARTY_SRC_DIRECTORY})
target_include_directories(PathMaker PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORY})
target_link_libraries(PathMaker OpenGL::GL glfw ${PCL_LIBRARIES})