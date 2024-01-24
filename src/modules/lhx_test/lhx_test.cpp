
/**
 * @file lhx_test.cpp
 * Example for Linux
 *
 * @author lhx
 */

#include "lhx_test.h"
#include <px4_platform_common/time.h>
#include <unistd.h>
#include <stdio.h>

px4::AppState LhxTest::appState;

int LhxTest::main()
{
	appState.setRunning(true);

	int i = 0;

	while (!appState.exitRequested() && i < 5) {
		px4_sleep(2);

		printf("  Doing work...\n");
		++i;
	}

	return 0;
}
