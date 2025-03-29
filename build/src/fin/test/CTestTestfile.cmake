# CMake generated Testfile for 
# Source directory: /home/captainnothing/OpenROAD/src/fin/test
# Build directory: /home/captainnothing/OpenROAD/build/src/fin/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(fin.gcd_fill "/usr/bin/bash" "/home/captainnothing/OpenROAD/src/fin/test/regression" "gcd_fill")
set_tests_properties(fin.gcd_fill PROPERTIES  ENVIRONMENT "TEST_TYPE=compare_logfile;CTEST_TESTNAME=gcd_fill;DIFF_LOCATION=/home/captainnothing/OpenROAD/src/fin/test/results/gcd_fill.diff" LABELS "IntegrationTest" WORKING_DIRECTORY "/home/captainnothing/OpenROAD/src/fin/test" _BACKTRACE_TRIPLES "/home/captainnothing/OpenROAD/src/cmake/testing.cmake;2;add_test;/home/captainnothing/OpenROAD/src/fin/test/CMakeLists.txt;8;or_integration_test;/home/captainnothing/OpenROAD/src/fin/test/CMakeLists.txt;0;")
