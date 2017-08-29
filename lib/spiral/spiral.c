#include "spiral_config.h"

#if CODE == 121-91
#include "spiral-121-91.c"

#elif CODE == 121-91-1
#include "spiral-v1-121-91.c"

#elif CODE == 133-171
#include "spiral-133-171.c"

#else

#error "Unknown code"

#endif

