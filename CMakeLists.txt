cmake_minimum_required(VERSION 3.5)
project(sas_clock)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(${PROJECT_NAME}
    src/sas_clock.cpp
    )

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_executable(${PROJECT_NAME}_example src/sas_clock_example.cpp)

target_link_libraries(${PROJECT_NAME}_example
    ${PROJECT_NAME}
    )

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

install(TARGETS
  ${PROJECT_NAME}_example
  DESTINATION lib/${PROJECT_NAME})

ament_package()
