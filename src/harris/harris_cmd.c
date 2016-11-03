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
/** @file harris_cmd.c
    Command line interface for harris module.
    @author rafael grompone von gioi (grompone@gmail.com)
 */
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
#define USE "                                                                  \
#name: harris                                                                  \
#author: rafael grompone von gioi                                              \
#version: 1.0 (2011.07.10)                                                     \
#year: 2011                                                                    \
#desc: Detect corners by Harris & Stephens method.                             \
#opt: sigma | s | double | 1.0  | 0 | | Sigma of Gaussian filter.              \
#opt: k | k | double | 0.04 | 0 | | Edge / corner selection parameter.         \
#opt: flat_th | f | double | 1000 | 0 | | Flat region threshold.               \
#opt: neigh | n | int | 1 | 1 | | Neighborhood size for local maximum.         \
#opt: epsfile | P | str | | | | Output line segments into EPS file 'epsfile'.  \
#req: in | | str | | | | Input image (PGM)                                     \
#req: out | | str | | | | List of Harris points (ascii file: x,y)              \
"
/*----------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "misc.h"
#include "harris.h"
#include "ntuple.h"
#include "cmd_line_handler.h"
#include "eps.h"
#include "pgm_io.h"
#include "seconds.c"

/*----------------------------------------------------------------------------*/
/*                                    Main                                    */
/*----------------------------------------------------------------------------*/
int main(int argc, char ** argv)
{
  struct arguments * arg = process_arguments(USE,argc,argv);
  image_double image;
  double sigma = get_double(arg,"sigma");
  double k = get_double(arg,"k");
  double flat_th = get_double(arg,"flat_th");
  int neigh = get_int(arg,"neigh");
  ntuple_list out;
  FILE * eps;
  FILE * txt;
  unsigned int i;
  double x,y;
  double radius = (double) neigh + 0.5; /* set EPS radius to the cover
                                           roughly the neighborhood used */

  /* read input */
  image = read_pgm_image_double(get_str(arg,"in"));

  /* compute Harris points */
  double tic = seconds();
  out = harris(image,sigma,k,flat_th,neigh);
  tic = seconds() - tic;
  fprintf(stderr, "harris took %g milliseconds (%g hz)\n", tic*1000, 1/tic);
  //free_ntuple_list(out);

  //tic = seconds();
  //out = harris2(image,sigma,k,flat_th,neigh);
  //tic = seconds() - tic;
  //fprintf(stderr, "harris2 took %g milliseconds (%g hz)\n", tic*1000, 1/tic);

  /* output */
  if( strcmp(get_str(arg,"out"),"-") == 0 ) txt = stdout;
  else txt = fopen(get_str(arg,"out"),"w");
  if(is_assigned(arg,"epsfile"))
    {
      eps = eps_open(get_str(arg,"epsfile"), image->xsize, image->ysize);
      eps_set_current_color(eps,1.0,0,0); /* set color to red */
    }
  for(i=0;i<out->size;i++)
    {
      x = out->values[i*out->dim+0];
      y = out->values[i*out->dim+1];
      fprintf(txt,"%g %g\n",x,y);
      if( eps != NULL ) eps_add_dot(eps,x,image->ysize-y,radius);
    }
  if( txt != stdout && fclose(txt) == EOF ) /* close txt outfile if needed */
    error("Error: unable to close txt output file.");
  if( eps != NULL ) eps_close(eps);

  /* free memory */
  free_image_double(image);
  free_ntuple_list(out);
  free_arguments(arg);
  return EXIT_SUCCESS;
}
/*----------------------------------------------------------------------------*/
