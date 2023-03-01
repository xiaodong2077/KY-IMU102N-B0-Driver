cmake_minimum_required(VERSION 3.0.2)
project(ky_imu_driver)

install(PROGRAMS
  script/imu_data_raw_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)