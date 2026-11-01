cmake_minimum_required(VERSION 3.16)
project(roamview_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf worlds config rviz models maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/cliff_guard.py scripts/brake_test.py scripts/patrol.py scripts/club_world.py
                 scripts/sim_check.py scripts/watch_memory.sh
        DESTINATION lib/${PROJECT_NAME})

ament_package()
