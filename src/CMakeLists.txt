file(GLOB SRC_FILES "*.cpp" "*.h")
add_executable(${PROJECT_NAME} ${SRC_FILES})

# Link igl (and the glfw imgui module)
target_link_libraries(${PROJECT_NAME} PUBLIC 
  igl::glfw
  igl::imgui
  )
