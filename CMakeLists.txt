cmake_minimum_required(VERSION 3.8)
project(path_maker)
cmake_policy(SET CMP0072 NEW)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- ros2 ---------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# --------------------------------

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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


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
ament_target_dependencies(PathMaker rclcpp nav_msgs geometry_msgs)

install(TARGETS
  PathMaker
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()