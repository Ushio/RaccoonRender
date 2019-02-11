#pragma once

#include <exception>
#include <iostream>

// #define RT_ASSERT(expect_true, message) ;
#define RT_ASSERT(expect_true) if((expect_true) == 0) { __debugbreak(); }
