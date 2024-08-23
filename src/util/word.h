#ifndef WORD_H
#define WORD_H

// Return low word of a 32-bit integer.
static inline uint16_t lowWord(uint32_t ww) { return (uint16_t)((ww)&0xFFFF); }

// Return high word of a 32-bit integer.
static inline uint16_t highWord(uint32_t ww) { return (uint16_t)((ww) >> 16); }

#endif // WORD_H
