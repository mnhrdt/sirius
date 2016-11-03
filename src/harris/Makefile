# -----------------------------------------------------------------------------
#
# harris - Implementation of Harris & Stephens corner detector.
#
# Copyright 2011 rafael grompone von gioi (grompone@gmail.com)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# -----------------------------------------------------------------------------

SRC= harris_cmd.c cmd_line_handler.c eps.c harris.c gauss.c image.c ntuple.c pgm_io.c misc.c


harris: *.c *.h Makefile
	cc -O3 -o harris $(SRC) -lm -lrt

clean:
	rm -f harris

# -----------------------------------------------------------------------------
