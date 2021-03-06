CC=clang
CC="gcc-6 -std=gnu99"
WFLAGS="-Wall -Wextra -Wno-sign-compare"

OFLAGS="-O0"
OFLAGS="-Ofast -march=native"
OFLAGS="-Ofast -march=native -DNDEBUG"

CFLAGS="$WFLAGS $OFLAGS"
OCVFLAGS=`pkg-config opencv --cflags --libs`
$CC $CFLAGS camflow.c -o camflow $OCVFLAGS -lm
#$CC $CFLAGS demo_cdr.c -o demo_cdr $OCVFLAGS -lm

# only for the version with enabled screenshots
#IIOFLAGS="-ltiff -lpng -ljpeg"
#$CC $CFLAGS -DENABLE_SCREENSHOTS camflow.c iio.o -o camflow $OCVFLAGS $IIOFLAGS -lm
