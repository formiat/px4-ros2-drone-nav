/* Roadmap item 20: the picture of one window, taken where it is drawn.
 *
 * Preloaded into the Gazebo window and into RViz of a recorded flight. On
 * every buffer swap it
 *
 * - paces the swap to FRAME_PACE_HZ, when that is set: the Gazebo window
 *   draws the whole scene again at the display's refresh rate, 144 Hz on the
 *   reference workstation, on the GPU the simulator renders its sensors with;
 * - reads the frame back, when FRAME_CAPTURE_FIFO is set: the middle of the
 *   largest surface the process draws, in the 8:9 shape of one half of the
 *   split picture, under its top FRAME_CAPTURE_TOP pixels (the size is
 *   written to <FIFO>.size), through a pixel buffer object, so that the read
 *   of one frame is collected at the next and the drawing never waits for
 *   it. A thread writes the latest frame to the FIFO FRAME_CAPTURE_HZ times a
 *   second, bottom row first, as BGRA. How many times the window redrew in
 *   each second of the wall clock is written to <FIFO>.rate, a line for
 *   every second it redrew in: a window the desktop stops presenting redraws
 *   once a second and its recording is a slideshow (r1030 to r1058).
 *
 * Reading the windows through the X server instead cost the simulator a
 * seventh of its speed a window (r1005 to r1007). An evaluation tool: nothing
 * of the stack loads it.
 *
 * Build: cc -shared -fPIC -O2 -o frame_pace_shim.so frame_pace_shim.c -ldl -lpthread
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define GL_PIXEL_PACK_BUFFER 0x88EB
#define GL_PIXEL_PACK_BUFFER_BINDING 0x88ED
#define GL_READ_FRAMEBUFFER 0x8CA8
#define GL_READ_FRAMEBUFFER_BINDING 0x8CAA
#define GL_STREAM_READ 0x88E1
#define GL_MAP_READ_BIT 0x0001
#define GL_BGRA 0x80E1
#define GL_UNSIGNED_BYTE 0x1401
#define GL_PACK_ALIGNMENT 0x0D05
#define GL_PACK_ROW_LENGTH 0x0D02
#define GL_BACK 0x0405
#define GL_READ_BUFFER 0x0C02
#define GLX_WIDTH 0x801D
#define GLX_HEIGHT 0x801E
#define EGL_WIDTH 0x3057
#define EGL_HEIGHT 0x3056

static void* gl(const char* name) {
  return dlsym(RTLD_DEFAULT, name);
}

static void add_ns(struct timespec* t, long ns) {
  t->tv_nsec += ns;
  while (t->tv_nsec >= 1000000000L) {
    t->tv_nsec -= 1000000000L;
    ++t->tv_sec;
  }
}

static void pace(void) {
  static struct timespec next;
  static long period_ns = -1;
  struct timespec now;
  if (period_ns < 0) {
    const char* hz = getenv("FRAME_PACE_HZ");
    period_ns = hz != NULL && atoi(hz) > 0 ? 1000000000L / atoi(hz) : 0;
    clock_gettime(CLOCK_MONOTONIC, &next);
  }
  if (period_ns == 0) {
    return;
  }
  add_ns(&next, period_ns);
  clock_gettime(CLOCK_MONOTONIC, &now);
  if (now.tv_sec > next.tv_sec ||
      (now.tv_sec == next.tv_sec && now.tv_nsec > next.tv_nsec)) {
    next = now; /* late: no debt of frames is run up */
    return;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
}

static pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
static unsigned char* frame;
static size_t frame_bytes;

static void* write_frames(void* unused) {
  const char* hz = getenv("FRAME_CAPTURE_HZ");
  const long period_ns = 1000000000L / (hz != NULL && atoi(hz) > 0 ? atoi(hz) : 24);
  /* Blocks until the recorder reads; the window has its final size by then. */
  const int fifo = open(getenv("FRAME_CAPTURE_FIFO"), O_WRONLY);
  unsigned char* out = NULL;
  size_t bytes = 0;
  struct timespec next;
  (void)unused;
  if (fifo < 0) {
    return NULL;
  }
  clock_gettime(CLOCK_MONOTONIC, &next);
  for (;;) {
    size_t done = 0;
    pthread_mutex_lock(&frame_lock);
    if (bytes == 0) {
      bytes = frame_bytes;
      out = malloc(bytes);
    }
    if (out != NULL && bytes == frame_bytes) {
      memcpy(out, frame, bytes);
    }
    pthread_mutex_unlock(&frame_lock);
    if (out == NULL) {
      return NULL;
    }
    while (done < bytes) {
      const ssize_t wrote = write(fifo, out + done, bytes - done);
      if (wrote <= 0) {
        return NULL; /* the reader left: the recording is over */
      }
      done += (size_t)wrote;
    }
    add_ns(&next, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }
}

/* One line a second of the wall clock in <FIFO>.rate: the second and the
   redraws of the captured window in it. */
static void count_redraw(void) {
  static FILE* rate;
  static time_t second;
  static int count;
  struct timespec now;
  clock_gettime(CLOCK_REALTIME, &now);
  if (rate == NULL) {
    char path[4096];
    snprintf(path, sizeof(path), "%s.rate", getenv("FRAME_CAPTURE_FIFO"));
    rate = fopen(path, "w");
    if (rate == NULL) {
      return;
    }
    second = now.tv_sec;
  }
  if (now.tv_sec != second) {
    fprintf(rate, "%ld %d\n", (long)second, count);
    fflush(rate);
    second = now.tv_sec;
    count = 0;
  }
  ++count;
}

/* The frame about to be swapped on `drawable`, `width` x `height` pixels. A
   process swaps more than one surface (RViz keeps small hidden ones): the
   largest is the window. */
static void capture(unsigned long drawable, int width, int height) {
  static unsigned long target;
  static int state; /* 0 unknown, 1 capturing, -1 off */
  static unsigned int pbo[2];
  static int turn, crop_w, crop_h, crop_x, seen_width, seen_height;
  static void (*get_integer)(unsigned int, int*);
  static void (*bind_buffer)(unsigned int, unsigned int);
  static void (*bind_framebuffer)(unsigned int, unsigned int);
  static void (*read_buffer)(unsigned int);
  static void (*pixel_store)(unsigned int, int);
  static void (*read_pixels)(int, int, int, int, unsigned int, unsigned int, void*);
  static void* (*map_range)(unsigned int, long, long, unsigned int);
  static unsigned char (*unmap)(unsigned int);
  int old_pbo = 0, old_fbo = 0, old_align = 4, old_row = 0, old_read = GL_BACK;
  void* mapped;
  if (state < 0) {
    return;
  }
  if (drawable != target) {
    if (state == 1 && (long)width * height <= (long)seen_width * seen_height) {
      return;
    }
    target = drawable;
  }
  if (state == 1 && (width != seen_width || height != seen_height)) {
    /* The window took another size (RViz goes full screen after it opens):
       the capture starts over. The recorder reads the size once it has
       settled, and opens the FIFO after that. */
    state = 0;
  }
  if (state == 0) {
    const char* top_px = getenv("FRAME_CAPTURE_TOP");
    const int top = top_px != NULL ? atoi(top_px) : 0;
    void (*gen_buffers)(int, unsigned int*) = gl("glGenBuffers");
    void (*buffer_data)(unsigned int, long, const void*, unsigned int) = gl("glBufferData");
    pthread_t writer;
    if (getenv("FRAME_CAPTURE_FIFO") == NULL || width < 400 || height < 400) {
      state = getenv("FRAME_CAPTURE_FIFO") == NULL ? -1 : 0;
      return;
    }
    get_integer = gl("glGetIntegerv");
    bind_buffer = gl("glBindBuffer");
    bind_framebuffer = gl("glBindFramebuffer");
    read_buffer = gl("glReadBuffer");
    pixel_store = gl("glPixelStorei");
    read_pixels = gl("glReadPixels");
    map_range = gl("glMapBufferRange");
    unmap = gl("glUnmapBuffer");
    /* The same region the recorder computes from the window's size. */
    crop_h = (height - top) - (height - top) % 2;
    crop_w = crop_h * 960 / 1080;
    if (crop_w > width) {
      crop_w = width;
    }
    crop_w -= crop_w % 2;
    crop_x = (width - crop_w) / 2;
    pthread_mutex_lock(&frame_lock);
    free(frame);
    frame_bytes = (size_t)crop_w * (size_t)crop_h * 4U;
    frame = calloc(1, frame_bytes);
    pthread_mutex_unlock(&frame_lock);
    get_integer(GL_PIXEL_PACK_BUFFER_BINDING, &old_pbo);
    if (pbo[0] == 0) {
      gen_buffers(2, pbo);
    }
    for (turn = 0; turn < 2; ++turn) {
      bind_buffer(GL_PIXEL_PACK_BUFFER, pbo[turn]);
      buffer_data(GL_PIXEL_PACK_BUFFER, (long)frame_bytes, NULL, GL_STREAM_READ);
    }
    bind_buffer(GL_PIXEL_PACK_BUFFER, (unsigned int)old_pbo);
    turn = 0;
    fprintf(stderr, "frame capture: %dx%d of a %dx%d window to %s\n", crop_w, crop_h,
            width, height, getenv("FRAME_CAPTURE_FIFO"));
    { /* The size of what the FIFO carries, for the recorder. */
      char path[4096];
      FILE* size;
      snprintf(path, sizeof(path), "%s.size", getenv("FRAME_CAPTURE_FIFO"));
      size = fopen(path, "w");
      if (size != NULL) {
        fprintf(size, "%dx%d\n", crop_w, crop_h);
        fclose(size);
      }
    }
    if (seen_width == 0) {
      pthread_create(&writer, NULL, write_frames, NULL);
    }
    seen_width = width;
    seen_height = height;
    state = 1;
    return; /* the first frame of a size fills its buffer; the next is read */
  }
  count_redraw();
  get_integer(GL_PIXEL_PACK_BUFFER_BINDING, &old_pbo);
  get_integer(GL_READ_FRAMEBUFFER_BINDING, &old_fbo);
  get_integer(GL_PACK_ALIGNMENT, &old_align);
  get_integer(GL_PACK_ROW_LENGTH, &old_row);
  get_integer(GL_READ_BUFFER, &old_read);
  bind_framebuffer(GL_READ_FRAMEBUFFER, 0);
  read_buffer(GL_BACK);
  pixel_store(GL_PACK_ALIGNMENT, 4);
  pixel_store(GL_PACK_ROW_LENGTH, 0);
  /* This frame into one buffer; the frame before, by now read, out of the other. */
  bind_buffer(GL_PIXEL_PACK_BUFFER, pbo[turn]);
  read_pixels(crop_x, 0, crop_w, crop_h, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
  bind_buffer(GL_PIXEL_PACK_BUFFER, pbo[1 - turn]);
  mapped = map_range(GL_PIXEL_PACK_BUFFER, 0, (long)frame_bytes, GL_MAP_READ_BIT);
  if (mapped != NULL) {
    pthread_mutex_lock(&frame_lock);
    memcpy(frame, mapped, frame_bytes);
    pthread_mutex_unlock(&frame_lock);
    unmap(GL_PIXEL_PACK_BUFFER);
  }
  turn = 1 - turn;
  pixel_store(GL_PACK_ALIGNMENT, old_align);
  pixel_store(GL_PACK_ROW_LENGTH, old_row);
  bind_framebuffer(GL_READ_FRAMEBUFFER, (unsigned int)old_fbo);
  if (old_fbo == 0) {
    read_buffer((unsigned int)old_read);
  }
  bind_buffer(GL_PIXEL_PACK_BUFFER, (unsigned int)old_pbo);
}

void glXSwapBuffers(void* display, unsigned long drawable) {
  static void (*real)(void*, unsigned long);
  static void (*query)(void*, unsigned long, int, unsigned int*);
  unsigned int width = 0, height = 0;
  if (real == NULL) {
    real = (void (*)(void*, unsigned long))dlsym(RTLD_NEXT, "glXSwapBuffers");
    query = gl("glXQueryDrawable");
  }
  pace();
  if (query != NULL) {
    query(display, drawable, GLX_WIDTH, &width);
    query(display, drawable, GLX_HEIGHT, &height);
    capture(drawable, (int)width, (int)height);
  }
  real(display, drawable);
}

unsigned int eglSwapBuffers(void* display, void* surface) {
  static unsigned int (*real)(void*, void*);
  static unsigned int (*query)(void*, void*, int, int*);
  int width = 0, height = 0;
  if (real == NULL) {
    real = (unsigned int (*)(void*, void*))dlsym(RTLD_NEXT, "eglSwapBuffers");
    query = gl("eglQuerySurface");
  }
  pace();
  if (query != NULL) {
    query(display, surface, EGL_WIDTH, &width);
    query(display, surface, EGL_HEIGHT, &height);
    capture((unsigned long)surface, width, height);
  }
  return real(display, surface);
}
