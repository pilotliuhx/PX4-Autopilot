
/**
 * @file lhx_test_main.cpp
 * Example for Linux
 *
 * @author lhx
 */

#include "lhx_test.h"

#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <stdio.h>

int PX4_MAIN(int argc, char **argv)
{
	px4::init(argc, argv, "hello");

	printf("hello\n");
	LhxTest hello;
	hello.main();

	printf("goodbye\n");
	return 0;
}
