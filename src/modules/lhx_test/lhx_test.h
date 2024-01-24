
/**
 * @file lhx_test.h
 * Example app for Linux
 *
 * @author lhx
 */
#pragma once

#include <px4_platform_common/app.h>

class LhxTest
{
public:
	LhxTest() {}

	~LhxTest() {}

	int main();

	static px4::AppState appState; /* track requests to terminate app */
};
