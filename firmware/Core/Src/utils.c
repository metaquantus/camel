/*
 * utils.c
 *  Miscelaneous functions to save space.
 *
 *  Minimal parse float / float to string functions
 *
 *  Created on: Jul 23, 2025
 *      Author: cav
 */
#include "stm32l0xx_hal.h"
#include "limits.h"

#define CONST const
// extern float __floatunsisf (unsigned long);

static CONST float pwr_p10 [6] = {
    1e+32, 1e+16, 1e+8, 1e+4, 1e+2, 1e+1,
};

static CONST float pwr_m10 [6] = {
    1e-32, 1e-16, 1e-8, 1e-4, 1e-2, 1e-1,
};

#define CASE_CONVERT    ('a' - 'A')
#define TOLOWER(c)        ((c) | CASE_CONVERT)

static const float pwr10_p [10] = {
    1, 1e+1, 1e+2, 1e+3, 1e+4, 1e+5, 1e+6, 1e+7, 1e+8, 1e+9
};
static const float pwr10_m [10] = {
    1, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9
};

// limited to power < 10
float pwr10(int power) {
  int p = power < 0 ? -power : power;
  if ( p > 9 ) {
    return 1; // fallback
  }
  return power < 0 ? pwr10_m[p] : pwr10_p[p];
}

float atoff(const char *nptr) {
  union {
    unsigned long u32;
    float flt;
  } x;
  unsigned char c;
  int exp;

  unsigned char flag;
#define FL_ANY      0x01  /* any digit was readed */
#define FL_OVFL     0x02  /* overflow was   */
#define FL_DOT      0x04  /* decimal '.' was  */
#define FL_MEXP     0x08  /* exponent 'e' is neg. */

  c = *nptr++;

  flag = 0;

  x.u32 = 0;
  exp = 0;
  while (1) {

    c -= '0';

    if (c <= 9) {
      flag |= FL_ANY;
      if (flag & FL_OVFL) {
        if (!(flag & FL_DOT))
          exp += 1;
      } else {
        if (flag & FL_DOT)
          exp -= 1;
        x.u32 = x.u32 * 10 + c;
        if (x.u32 >= (ULONG_MAX - 9) / 10)
          flag |= FL_OVFL;
      }

    } else if (c == (('.' - '0') & 0xff) && !(flag & FL_DOT)) {
      flag |= FL_DOT;
    } else {
      break;
    }
    c = *nptr++;
  }

  if (TOLOWER(c) == 'e' - '0') {
    int i;
    c = *nptr++;
    i = 2;
    if (c == '-') {
      flag |= FL_MEXP;
      c = *nptr++;
    } else if (c == '+') {
      c = *nptr++;
    } else {
      i = 1;
    }
    c -= '0';
    if (c > 9) {
      nptr -= i;
    } else {
      i = 0;
      do {
        if (i < 3200)
          i = i * 10 + c;
        c = *nptr++ - '0';
      } while (c <= 9);
      if (flag & FL_MEXP)
        i = -i;
      exp += i;
    }
  }

  x.flt = (float) x.u32; //  __floatunsisf(x.u32); /* manually */

  if (x.flt != 0) {
    int pwr;
    CONST
    float *fptr;

    if (exp < 0) {
      fptr = pwr_m10;
      exp = -exp;
    } else {
      fptr = pwr_p10;
    }
    for (pwr = 32; pwr; pwr >>= 1) {
      for (; exp >= pwr; exp -= pwr)
        x.flt *= *fptr;
      fptr++;
    }
  }

  return x.flt;
}

void fftoa(float f, char *str, uint8_t precision) {
  float ff;
//char str[30];
  int a, b, c, k, l = 0, m, i = 0;
//scanf("%f",&f);
  ff = f;

  // check for negetive float
  if (f < 0.0) {
    str[i++] = '-';
    f *= -1;
  }

  a = f;  // extracting whole number
  f -= a; // extracting decimal part
  k = 0;

 // number of digits in whole number
  while (1) {
    l = pwr10(k);
    m = a / l;
    if (m == 0) {
      break;
    }
    k++;
  }
  k--;
  // number of digits in whole number are k+1

  /*
   extracting most significant digit i.e. right most digit , and concatenating to string
   obtained as quotient by dividing number by 10^k where k = (number of digit -1)
   */

  for (l = k + 1; l > 0; l--) {
    b = pwr10(l - 1);
    c = a / b;
    str[i++] = c + 48;
    a %= b;
  }
  if (precision != 0)
    str[i++] = '.';

  /* extracting decimal digits till precision */

  for (l = 0; l < precision; l++) {
    f *= 10.0;
    b = f;
    str[i++] = b + 48;
    f -= b;
    if (f == 0) {
      break;
    }
  }

  str[i] = '\0';

//printf("\n orignal printf %f\n",ff);
//printf("\n float string %s\n",str);
}

// simple CRC-8 implementation
uint8_t _gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0xff;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}
