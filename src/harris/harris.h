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
/** @file harris.h
    harris module header
    @author rafael grompone von gioi (grompone@gmail.com)
 */
/*----------------------------------------------------------------------------*/
#ifndef HARRIS_HEADER
#define HARRIS_HEADER

#include "ntuple.h"
#include "image.h"

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

      C = dx dy * Gaussian_filter(sigma)

    where dx and dy are the derivatives at the pixel.

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
                    double k, double flat_th, int neigh );
ntuple_list harris2( image_double image, double sigma,
                    double k, double flat_th, int neigh );

#endif /* !HARRIS_HEADER */
/*----------------------------------------------------------------------------*/
