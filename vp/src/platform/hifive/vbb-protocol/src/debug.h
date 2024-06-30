#pragma once

#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#define DEBUG(...) if (ENABLE_DEBUG) {DEBUG_PRINT(__VA_ARGS__);}
