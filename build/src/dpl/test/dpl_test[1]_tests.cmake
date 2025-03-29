add_test([=[OpendpTest.IsPlaced]=]  /home/captainnothing/OpenROAD/build/src/dpl/test/dpl_test [==[--gtest_filter=OpendpTest.IsPlaced]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[OpendpTest.IsPlaced]=]  PROPERTIES WORKING_DIRECTORY /home/captainnothing/OpenROAD/src/dpl/test SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  dpl_test_TESTS OpendpTest.IsPlaced)
