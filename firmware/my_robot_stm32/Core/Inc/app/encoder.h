#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

void    encoder_init(void);
int32_t encoder_get_delta(uint8_t id);    /* raw delta counts since last call */
int64_t encoder_get_total(uint8_t id);    /* cumulative counts */
void    encoder_reset_all(void);

#endif /* ENCODER_H */
