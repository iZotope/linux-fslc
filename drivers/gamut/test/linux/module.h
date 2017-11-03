#pragma once

#include "errno-base.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#define printk(...) printf(__VA_ARGS__)
#define dev_err(X , Y , ...) printf("err: "  Y , ##__VA_ARGS__ )
#define smp_store_release(p, v) ((*(p)) = (v))

typedef uint8_t u8;
typedef uint16_t u16;
