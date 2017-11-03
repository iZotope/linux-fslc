#pragma once

#define __user

#include <string.h>

#define copy_to_user(to, from, n) ({memcpy(to, from, n); 0;})
#define copy_from_user(to, from, n) ({memcpy(to, from, n); 0;})
