rock_testsuite(test test.cpp test_backwards.cpp DEPS base)
rock_executable(benchmark benchmark.cpp bench_func.cpp DEPS base NOINSTALL)
