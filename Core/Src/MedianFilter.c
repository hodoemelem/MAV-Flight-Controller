/*
 * MedianFilter.c
 *
 *  Created on: 29 Jan 2021
 *      Author: HENRY ODOEMELEM
 */


#include "MedianFilter.h"

//float rateBuffer_7[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float MedianFilter_7(float vals[7])
{
float a = vals[0];
float b = vals[1];
float c = vals[2];
float d = vals[3];
float e = vals[4];
float f = vals[5];
float g = vals[6];
  // round 1
  sort(&a, &b);
  sort(&b, &c);
  sort(&c, &d);
  sort(&d, &e);
  sort(&e, &f);
  sort(&f, &g);

  // round 2
  sort(&a, &b);
  sort(&b, &c);
  sort(&c, &d);
  sort(&d, &e);
  sort(&e, &f);

  // round 3
  sort(&a, &b);
  sort(&b, &c);
  sort(&c, &d);
  sort(&d, &e);

  // round 4
  sort(&a, &b);
  sort(&b, &c);
  sort(&c, &d);


  return d;
}

void sort(float *a, float *b)
{
	if(*a > *b){ swap(a, b); }
}

void swap(float *a, float *b)
{
	float temp = *a;
	*a = *b;
	*b = temp;
}
