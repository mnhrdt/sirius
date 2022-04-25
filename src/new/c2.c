// https://gist.github.com/bellbind/6813905
// capturing from UVC cam using video4linux2
// build: cc c2.c -o c2

#include <stdbool.h> // true, false
#include <stdint.h> // uint8_t, uint32_t
#include <stdlib.h> // exit, malloc, calloc, free
#include <stdio.h> // FILE, fprintf, stderr
#include <string.h> // strerror, memset, memcpy
#include <errno.h> // errno, EINTR

#include <fcntl.h> // open
#include <unistd.h> // close
#include <sys/ioctl.h> // ioctl
#include <sys/mman.h> // mmap, munmap

#include <linux/videodev2.h> // v4l2_*, V4L2_*, VIDIOC_*


#define THE_NUMBER_OF_BUFFERS_SHALL_BE_FOUR 2

static void quit(const char * msg)
{
	fprintf(stderr, "[%s] %d: %s\n", msg, errno, strerror(errno));
	exit(1);
}

// keep trying to ioctl until success (or too many trials)
static int xioctl(int fd, unsigned long request, void* arg)
{
	for (int i = 0; i < 100; i++) {
		int r = ioctl(fd, request, arg);
		if (r != -1 || errno != EINTR)
		{
			//fprintf(stderr, "xioctl req=%zu, i=%d r=%d\n",
			//		request, i, r);
			return r;
		}
	}
	//fprintf(stderr, "xioctl req=%zu, i=%d r=%d\n", request, -1, -1);
	return -1;
}

// used only inside the camera struct
struct camera_buffer_t {
	uint8_t* start;
	size_t length;
};

// internal struct, just for this program
struct camera_t {
	int fd;
	uint32_t width;
	uint32_t height;
	size_t buffer_count;
	struct camera_buffer_t *buffers;
	struct camera_buffer_t head;
};

// open the device file and fill-in struct fields
// returns an allocated struct that needs to be freed afterwards
struct camera_t* camera_open(
		const char *device,
	       	uint32_t width,
		uint32_t height
		)
{
	int fd = open(device, O_RDWR | O_NONBLOCK, 0);
	if (fd == -1) quit("open");
	struct camera_t *c = malloc(sizeof*c);
	c->fd = fd;
	c->width = width;
	c->height = height;
	c->buffer_count = 0;
	c->buffers = NULL;
	c->head.length = 0;
	c->head.start = NULL;
	return c;
}

// check and setup capabilities via ioctls, memmap buffers
// (the actual webcam is still off)
void camera_init(struct camera_t* c)
{
	//// query capabilities (we need "video capture" and "streaming")
	//struct v4l2_capability cap;
	//if (xioctl(c->fd, VIDIOC_QUERYCAP, &cap) == -1)
	//	quit("VIDIOC_QUERYCAP");
	//if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
	//	quit("no capture");
	//if (!(cap.capabilities & V4L2_CAP_STREAMING))
	//	quit("no streaming");

	//// query crop capabilities
	//struct v4l2_cropcap cropcap;
	//memset(&cropcap, 0, sizeof cropcap);
	//cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//if (xioctl(c->fd, VIDIOC_CROPCAP, &cropcap) == 0) {
	//	struct v4l2_crop crop;
	//	crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//	crop.c = cropcap.defrect;
	//	if (xioctl(c->fd, VIDIOC_S_CROP, &crop) == -1) {
	//		// cropping not supported
	//	}
	//}

	// request pixel format (YUYV, but why???)
	struct v4l2_format format;
	memset(&format, 0, sizeof format);
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = c->width;
	format.fmt.pix.height = c->height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	//format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	format.fmt.pix.field = V4L2_FIELD_NONE;
	if (xioctl(c->fd, VIDIOC_S_FMT, &format) == -1)
		quit("VIDIOC_S_FMT");
	fprintf(stderr, "CAMERA INIT w,h=%d,%d\n", c->width, c->height);

	// request four buffers
	struct v4l2_requestbuffers req;
	memset(&req, 0, sizeof req);
	req.count = THE_NUMBER_OF_BUFFERS_SHALL_BE_FOUR;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	if (xioctl(c->fd, VIDIOC_REQBUFS, &req) == -1)
		quit("VIDIOC_REQBUFS");
	c->buffer_count = req.count;
	c->buffers = calloc(req.count, sizeof (struct camera_buffer_t));

	// mmap each buffer, and malloc the head one (externally visible)
	size_t buf_max = 0;
	for (size_t i = 0; i < c->buffer_count; i++) {
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (xioctl(c->fd, VIDIOC_QUERYBUF, &buf) == -1)
			quit("VIDIOC_QUERYBUF");
		if (buf.length > buf_max) buf_max = buf.length;
		fprintf(stderr, "\tBUF LENGTH = %d\n", buf.length);
		c->buffers[i].length = buf.length;
		c->buffers[i].start = 
			mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
					MAP_SHARED, c->fd, buf.m.offset);
		if (c->buffers[i].start == MAP_FAILED) quit("mmap");
	}
	fprintf(stderr, "\tBUF MAX = %ld\n", buf_max);
	c->head.start = malloc(4*buf_max);
}

// switch on the camera and start recording frames into the buffers
void camera_start(struct camera_t* c)
{
	// assign each buffer to a query
	for (size_t i = 0; i < c->buffer_count; i++) {
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1)
			quit("VIDIOC_QBUF");
	}

	// start streaming unto the buffers
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_STREAMON, &type) == -1)
		quit("VIDIOC_STREAMON");
}

// switch off the camera
void camera_stop(struct camera_t* c)
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (xioctl(c->fd, VIDIOC_STREAMOFF, &type) == -1)
		quit("VIDIOC_STREAMOFF");
}

// free buffers and stuff
void camera_finish(struct camera_t* c)
{
	for (size_t i = 0; i < c->buffer_count; i++) {
		munmap(c->buffers[i].start, c->buffers[i].length);
	}
	free(c->buffers);
	c->buffer_count = 0;
	c->buffers = NULL;
	free(c->head.start);
	c->head.length = 0;
	c->head.start = NULL;
}

// close device and free struct memory
void camera_close(struct camera_t* c)
{
	if (close(c->fd) == -1)
		quit("close");
	free(c);
}

// copy one frame from the buffer ring to the head, at any time
int camera_capture(struct camera_t* c)
{
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	if (xioctl(c->fd, VIDIOC_DQBUF, &buf) == -1)
		return false;
	//fprintf(stderr, "CAPTURE bytesused = %d\n", buf.bytesused);
	memcpy(c->head.start, c->buffers[buf.index].start, buf.bytesused);
	c->head.length = buf.bytesused;
	if (xioctl(c->fd, VIDIOC_QBUF, &buf) == -1)
		return false;
	return true;
}

// wait at most "timeout" and capture the next frame
int camera_frame(struct camera_t* c, struct timeval timeout)
{
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(c->fd, &fds);
	int r = select(c->fd + 1, &fds, 0, 0, &timeout);
	if (r == -1) quit("select");
	if (r == 0) return false;
	return camera_capture(c);
}

// save an image into a file
static void ppm(FILE* dest, uint8_t* rgb, uint32_t w, uint32_t h)
{
	fprintf(dest, "P6\n%d %d 255\n", w, h);
	fwrite(rgb, 3, w*h, dest);
}

static int bclamp(int v)
{
	if (v < 0) return 0;
	if (v > 255) return 255;
	return v;
}

uint8_t* yuyv2rgb(uint8_t* yuyv, uint32_t width, uint32_t height)
{
	//fprintf(stderr, "YUYV2RGB w,h = %d %d\n", width, height);
	uint8_t* rgb = calloc(width * height * 3, sizeof (uint8_t));
	for (size_t i = 0; i < height; i++)
	for (size_t j = 0; j < width; j += 2)
	{
		size_t ij = i * width + j;
		int y0 = yuyv[ij * 2 + 0] << 8;
		int u  = yuyv[ij * 2 + 1] - 128;
		int y1 = yuyv[ij * 2 + 2] << 8;
		int v  = yuyv[ij * 2 + 3] - 128;
		rgb[3*ij + 0 + 0] = bclamp((y0 + 359*v) >> 8);
		rgb[3*ij + 0 + 3] = bclamp((y1 + 359*v) >> 8);
		rgb[3*ij + 1 + 0] = bclamp((y0 + 88*u - 183*v) >> 8);
		rgb[3*ij + 1 + 3] = bclamp((y1 + 88*u - 183*v) >> 8);
		rgb[3*ij + 2 + 0] = bclamp((y0 + 454*u) >> 8);
		rgb[3*ij + 2 + 3] = bclamp((y1 + 454*u) >> 8);
	}
	return rgb;
}

//void fillrgb(
//		uint8_t *rgb,  // rgb  888
//		uint8_t *yuyv,   // yuyv 422
//		int w, int h
//	    )
//{
//	for (size_t i = 0; i < h; i += 1)
//	for (size_t j = 0; j < w; j += 2)
//	{
//		size_t ij = i * w + j;
//		int y0 = yuyv[ij * 2 + 0] << 8;
//		int u  = yuyv[ij * 2 + 1] - 128;
//		int y1 = yuyv[ij * 2 + 2] << 8;
//		int v  = yuyv[ij * 2 + 3] - 128;
//		rgb[3*ij + 0 + 0] = bclamp((y0 + 359*v) >> 8);
//		rgb[3*ij + 0 + 3] = bclamp((y1 + 359*v) >> 8);
//		rgb[3*ij + 1 + 0] = bclamp((y0 + 88*v - 183*u) >> 8);
//		rgb[3*ij + 1 + 3] = bclamp((y1 + 88*v - 183*u) >> 8);
//		rgb[3*ij + 2 + 0] = bclamp((y0 + 454*u) >> 8);
//		rgb[3*ij + 2 + 3] = bclamp((y1 + 454*u) >> 8);
//	}
//}

//static uint8_t* rgb2rgb(uint8_t* x, uint32_t width, uint32_t height)
//{
//	uint8_t* rgb = calloc(width * height * 3, sizeof (uint8_t));
//	for (size_t i = 0; i < 3*width*height; i++)
//		rgb[i] = x[i];
//	return rgb;
//}

#ifndef HIDE_MAIN
#define CAPTURE_MAIN
#endif

#ifdef CAPTURE_MAIN
int main()
{
	struct camera_t* c = camera_open("/dev/video0", 800, 600);
	camera_init(c);
	camera_start(c);

	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;
	for (int i = 0; i < 100; i++)
	{
		camera_frame(c, timeout);

		uint8_t *rgb = yuyv2rgb(c->head.start, c->width, c->height);
		//uint8_t *rgb = rgb2rgb(c->head.start, c->width, c->height);
		char filename[FILENAME_MAX];
		sprintf(filename, "result_%03d.ppm", i);
		FILE* out = fopen(filename, "w");
		ppm(out, rgb, c->width, c->height);
		fclose(out);
		free(rgb);
	}

	camera_stop(c);
	camera_finish(c);
	camera_close(c);
	return 0;
}
#endif//CAPTURE_MAIN
