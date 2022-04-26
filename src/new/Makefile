CFLAGS = -g
CFLAGS = -O3 -march=native
kam    : ; $(CC) $(CFLAGS) kam.c c2.c ftr.c -DWEBCAM_HIDE_MAIN -lX11 -o $@
clean  : ; $(RM) kam
