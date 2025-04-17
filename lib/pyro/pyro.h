#ifndef PYRO_H
#define PYRO_H

#include <stdint.h>

void setupPyros(uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4);

// 0 INDEXED
void firePyro(uint8_t channel);

#endif