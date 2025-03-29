add_test([=[Mpl2.CanConstruct]=]  /home/captainnothing/OpenROAD/build/src/mpl2/test/cpp/mpl2_test [==[--gtest_filter=Mpl2.CanConstruct]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[Mpl2.CanConstruct]=]  PROPERTIES WORKING_DIRECTORY /home/captainnothing/OpenROAD/src/mpl2/test/cpp/.. SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  mpl2_test_TESTS Mpl2.CanConstruct)
