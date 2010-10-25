/*
 * Math.cpp
 *
 *  Created on: Jan 21, 2010
 *      Author: seh
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

float frand(float min, float max) {
    if (min < max)
	return min + ((max-min) * ((float)rand()) / ((float)RAND_MAX));
    else
        return max + ((min-max) * ((float)rand()) / ((float)RAND_MAX));
}
int irand(int min, int max) {
	return floor((min + ((max-min) * ((float)rand()) / ((float)RAND_MAX))));
}

