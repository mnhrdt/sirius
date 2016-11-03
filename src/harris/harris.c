/*----------------------------------------------------------------------------

  harris - Implementation of Harris & Stephens corner detector.

  Copyright 2011 rafael grompone von gioi (grompone@gmail.com)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Affero General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU Affero General Public License for more details.

  You should have received a copy of the GNU Affero General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.

  ----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/** @file harris.c
    harris module code
    @author rafael grompone von gioi (grompone@gmail.com)
 */
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include "harris.h"
#include "gauss.h"
#include "ntuple.h"
#include "image.h"
#include "misc.h"

/*----------------------------------------------------------------------------*/
/** Implementation of Harris & Stephens corner detector.

    The algorithm is described in

      "A Combined Corner and Edge Detector" by Chris Harris & Mike Stephens,
       Proceedings of the 4th Alvey Vision Conference, pp. 147-151, 1988.

    At each pixel two quantities are computed:

      T = A + B

      R = D - k T^2

    where T is the trace and D the determinant of matrix

      A C

      C B

    and k is the edge / corner selection parameter.
    A, B, and C are defined by

      A = dx^2 * Gaussian_filter(sigma)

      B = dy^2 * Gaussian_filter(sigma)

      C = dx.dy * Gaussian_filter(sigma)

    where dx and dy are the derivatives at the pixel,
    and * denotes a convolution.

    A given pixel is a corner if the following three conditions are satisfied:

    - T > flat_th
    - R > 0
    - R is a local maximum

    @param image       Input image.

    @param sigma       Sigma value used on the Gaussian filter.
                       Suggested value: 1.0

    @param k           Edge / corner selection parameter.
                       Suggested value: 0.04

    @param flat_th     Threshold on the flatness of a point.
                       Suggested value: 1000

    @param neigh       Size of the neighborhood. The neighborhood is
                       a square regions of 2neigh+1 pixels of side,
                       centered at the pixel of interest.
                       Suggested value: 1

    @return            A 2-tuple list, where each 2-tuple corresponds to
                       the coordinates of a Harris point. Even if the
                       2-tuple are of type 'double', the values are integer.
 */
ntuple_list harris( image_double image, double sigma,
                    double k, double flat_th, int neigh )
{
  image_double A,B,C,cornerR;
  unsigned int x,y,xx,yy;
  double dx,dy,a,b,c,T,D,R,max;
  ntuple_list out = new_ntuple_list(2);
  
  /* get memory */
  A = new_image_double_ini(image->xsize,image->ysize,0.0);
  B = new_image_double_ini(image->xsize,image->ysize,0.0);
  C = new_image_double_ini(image->xsize,image->ysize,0.0);
  cornerR = new_image_double_ini(image->xsize,image->ysize,-1.0);

  /* compute dx,dy,A,B,C */
  for(x=1;x<image->xsize-1;x++)
    for(y=1;y<image->ysize-1;y++)
      {
        dx = 0.5 * (   image->data[ (x+1) + y * image->xsize ]
                     - image->data[ (x-1) + y * image->xsize ] );
        dy = 0.5 * (   image->data[ x + (y+1) * image->xsize ]
                     - image->data[ x + (y-1) * image->xsize ] );
        A->data[ x + y * A->xsize ] = dx*dx;
        B->data[ x + y * B->xsize ] = dy*dy;
        C->data[ x + y * C->xsize ] = dx*dy;
      }

  /* apply Gaussian filter to A,B,C */
  gaussian_filter(A,sigma);
  gaussian_filter(B,sigma);
  gaussian_filter(C,sigma);

  /* compute corner response R */
  for(x=1;x<image->xsize-1;x++)
    for(y=1;y<image->ysize-1;y++)
      {
        a = A->data[ x + y * A->xsize ];
        b = B->data[ x + y * B->xsize ];
        c = C->data[ x + y * C->xsize ];
        T = a+b;            /* trace */
        D = a*b - c*c;      /* determinant */
        R = D - k * T * T;  /* Harris & Stephens corner response */
        if( T > flat_th && R > 0.0 ) /* possible corner */
          cornerR->data[ x + y * cornerR->xsize ] = R;
      }

  /* compute corners as local maxima of cornerR */
  for(x=neigh;x<image->xsize-neigh;x++)
    for(y=neigh;y<image->ysize-neigh;y++)
      if( cornerR->data[ x + y * cornerR->xsize ] > 0.0 )
        {
          max = -1.0;
          for( xx=x-neigh; xx<=x+neigh; xx++ )
            for( yy=y-neigh; yy<=y+neigh; yy++ )
              if( xx!=x || yy!=y ) /* strict maximum */
                if( cornerR->data[ xx + yy * cornerR->xsize ] > max )
                  max = cornerR->data[ xx + yy * cornerR->xsize ];

          /* if strict maximum, a Harris point was found */
          if( cornerR->data[ x + y * cornerR->xsize ] > max )
            {
              /* if needed, alloc more tuples to 'out' */
              if( out->size == out->max_size ) enlarge_ntuple_list(out);

              /* add new 2-tuple */
              out->values[ out->size * out->dim + 0 ] = (double) x;
              out->values[ out->size * out->dim + 1 ] = (double) y;

              /* update number of tuples counter */
              out->size++;
            }
        }

  /* free memory */
  free_image_double(A);
  free_image_double(B);
  free_image_double(C);
  free_image_double(cornerR);

  /* return list of Harris points */
  return out;
}

ntuple_list harris2( image_double image, double sigma,
                    double k, double flat_th, int neigh )
{
  image_double cornerR;
  unsigned int x,y,xx,yy;
  double max;
  ntuple_list out = new_ntuple_list(2);
  
  /* get memory */
  //A = new_image_double_ini(image->xsize,image->ysize,0.0);
  //B = new_image_double_ini(image->xsize,image->ysize,0.0);
  //C = new_image_double_ini(image->xsize,image->ysize,0.0);
  cornerR = new_image_double_ini(image->xsize,image->ysize,-1.0);

  gaussian_filter(image,sigma);

  /* compute dx,dy,A,B,C */
  for(x=1;x<image->xsize-1;x++)
    for(y=1;y<image->ysize-1;y++)
      {
	      /*
        dx = 0.5 * (   image->data[ (x+1) + y * image->xsize ]
                     - image->data[ (x-1) + y * image->xsize ] );
        dy = 0.5 * (   image->data[ x + (y+1) * image->xsize ]
                     - image->data[ x + (y-1) * image->xsize ] );
        A->data[ x + y * A->xsize ] = dx*dx;
        B->data[ x + y * B->xsize ] = dy*dy;
        C->data[ x + y * C->xsize ] = dx*dy;
	*/
        double dxx =   image->data[ (x+1) + y * image->xsize ]
                   +   image->data[ (x-1) + y * image->xsize ]
                   - 2*image->data[ x + y * image->xsize ];
        double dyy =   image->data[ x + (y+1) * image->xsize ]
                   +   image->data[ x + (y-1) * image->xsize ]
                   - 2*image->data[ x + y * image->xsize ];
	double dxy = image->data[ (x+1) + (y+1) * image->xsize ]
                   + image->data[ (x-1) + (y-1) * image->xsize ]
                   - image->data[ (x+1) + (y-1) * image->xsize ]
                   - image->data[ (x-1) + (y+1) * image->xsize ];
        double a = dxx;
        double b = dyy;
        double c = dxy;
        double T = a+b;            /* trace */
        double D = a*b - c*c;      /* determinant */
        double R = D - k * T * T;  /* Harris & Stephens corner response */
        if( T > flat_th && R > 0.0 ) /* possible corner */
          cornerR->data[ x + y * cornerR->xsize ] = R;
      }

  /* apply Gaussian filter to A,B,C */
  /*
  gaussian_filter(A,sigma);
  gaussian_filter(B,sigma);
  gaussian_filter(C,sigma);
  */

  /* compute corners as local maxima of cornerR */
  for(x=neigh;x<image->xsize-neigh;x++)
    for(y=neigh;y<image->ysize-neigh;y++)
      if( cornerR->data[ x + y * cornerR->xsize ] > 0.0 )
        {
          max = -1.0;
          for( xx=x-neigh; xx<=x+neigh; xx++ )
            for( yy=y-neigh; yy<=y+neigh; yy++ )
              if( xx!=x || yy!=y ) /* strict maximum */
                if( cornerR->data[ xx + yy * cornerR->xsize ] > max )
                  max = cornerR->data[ xx + yy * cornerR->xsize ];

          /* if strict maximum, a Harris point was found */
          if( cornerR->data[ x + y * cornerR->xsize ] > max )
            {
              /* if needed, alloc more tuples to 'out' */
              if( out->size == out->max_size ) enlarge_ntuple_list(out);

              /* add new 2-tuple */
              out->values[ out->size * out->dim + 0 ] = (double) x;
              out->values[ out->size * out->dim + 1 ] = (double) y;

              /* update number of tuples counter */
              out->size++;
            }
        }

  /* free memory */
  //free_image_double(A);
  //free_image_double(B);
  //free_image_double(C);
  free_image_double(cornerR);

  /* return list of Harris points */
  return out;
}
/*----------------------------------------------------------------------------*/
