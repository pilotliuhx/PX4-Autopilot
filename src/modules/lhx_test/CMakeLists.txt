px4_add_module(
	MODULE modules__lhx_test
	MAIN lhx_test
	COMPILE_FLAGS
	SRCS
		lhx_test_main.cpp
		lhx_test_start.cpp
		lhx_test.cpp
	DEPENDS
	)
