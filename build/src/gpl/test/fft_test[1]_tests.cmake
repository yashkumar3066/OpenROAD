add_test([=[FloatFFTTest.Basic]=]  /home/captainnothing/OpenROAD/build/src/gpl/test/fft_test [==[--gtest_filter=FloatFFTTest.Basic]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[FloatFFTTest.Basic]=]  PROPERTIES WORKING_DIRECTORY /home/captainnothing/OpenROAD/src/gpl/test SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set(  fft_test_TESTS FloatFFTTest.Basic)
