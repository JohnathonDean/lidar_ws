#ifndef _RANSAC_H_
#define _RANSAC_H_

#include <cstdlib>

#include "straight_line_fitting/ransac_fitline.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <iostream>

float Ransac(Point2D32f *points, size_t Cnt, float *lines);
float Ransac(Point2D32f *points, size_t Cnt, float *line, int numForEstimate,
             float successProbability, float maxOutliersPercentage);

#endif
