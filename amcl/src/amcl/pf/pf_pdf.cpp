/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Useful pdf functions
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf_pdf.c 6348 2008-04-17 02:53:17Z gerkey $
 *************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
//#include <gsl/gsl_rng.h>
//#include <gsl/gsl_randist.h>

#include "pf_pdf.h"
#include <random>
// Random number generator seed value
static unsigned int pf_pdf_seed;


/**************************************************************************
 * Gaussian
 *************************************************************************/
std::default_random_engine generator;

// Create a gaussian pdf
pf_pdf_gaussian_t *pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
{
  pf_matrix_t cd;
  pf_pdf_gaussian_t *pdf;

  pdf = (pf_pdf_gaussian_t *)calloc(1, sizeof(pf_pdf_gaussian_t));

  pdf->x = x;
  pdf->cx = cx;
  //pdf->cxi = pf_matrix_inverse(cx, &pdf->cxdet);

  // Decompose the convariance matrix into a rotation
  // matrix and a diagonal matrix.
  pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
  pdf->cd.v[0] = sqrt(cd.m[0][0]);
  pdf->cd.v[1] = sqrt(cd.m[1][1]);
  pdf->cd.v[2] = sqrt(cd.m[2][2]);

#ifdef BUILD_DEBUG
  printf("\n[pf_pdf_gaussian_alloc] pdf mean = %lf, %lf, yaw %lf\n", x.v[0], x.v[1], x.v[2]);
  printf("\t cov = \n");
  for (int r = 0; r < 3; ++r) {
      printf("\t\t");
      for (int c = 0; c < 3; ++c) {
          printf("%lf\t", cx.m[r][c]);
      }
      printf("\n");
  }
  printf("\n[pf_pdf_gaussian_alloc]\ncd.m = \n");
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++) {
      printf("%lf\t", i, j, cd.m[i][j]);
    }
    printf("\n");
  }
#endif

  // Initialize the random number generator
  generator.seed(pf_pdf_seed);
  srand48(++pf_pdf_seed);


  return pdf;
}


// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t *pdf)
{
  //gsl_rng_free(pdf->rng);
  free(pdf);
  return;
}

// Generate a sample from the the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t *pdf)
{
  int i, j;
  pf_vector_t r;
  pf_vector_t x;

  // Generate a random vector
  for (i = 0; i < 3; i++)
  {
    r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);
// #ifdef BUILD_DEBUG
//     printf("r.v[%d] = %lf  ", i, r.v[i]);
// #endif
  }

  for (i = 0; i < 3; i++)
  {
    x.v[i] = pdf->x.v[i];
    for (j = 0; j < 3; j++)
      x.v[i] += pdf->cr.m[i][j] * r.v[j];
  }

  // double temp = x.v[2];
  // x.v[2] = x.v[0];
  // x.v[0] = temp;
  return x;
}

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double stddev)
{
  std::normal_distribution<double> noise(0.0, stddev);
  return noise(generator);
}
