/* Newlib must not allocate from RAM owned by micro T-Kernel's allocator.
 * The application uses RTOS memory services; no C-library heap is reserved.
 * Override libnosys' unbounded _sbrk rather than aliasing its `end` symbol
 * to the beginning of the RTOS memory pool. */
#include <errno.h>
#include <stddef.h>
#include <sys/types.h>

void *_sbrk(ptrdiff_t increment);
int _close(int descriptor);
off_t _lseek(int descriptor, off_t offset, int origin);
int _read(int descriptor, char *buffer, int length);
int _write(int descriptor, const char *buffer, int length);

void *_sbrk(ptrdiff_t increment)
{
    (void)increment;
    errno = ENOMEM;
    return (void *)-1;
}

/* No POSIX file descriptors are provided. Console traffic uses tm_printf
 * and the RTOS USB driver, not Newlib stdin/stdout. Fail explicitly instead
 * of claiming that input/output succeeded or linking warning-only stubs. */
int _close(int descriptor)
{
    (void)descriptor;
    errno = EBADF;
    return -1;
}

off_t _lseek(int descriptor, off_t offset, int origin)
{
    (void)descriptor;
    (void)offset;
    (void)origin;
    errno = EBADF;
    return (off_t)-1;
}

int _read(int descriptor, char *buffer, int length)
{
    (void)descriptor;
    (void)buffer;
    (void)length;
    errno = EBADF;
    return -1;
}

int _write(int descriptor, const char *buffer, int length)
{
    (void)descriptor;
    (void)buffer;
    (void)length;
    errno = EBADF;
    return -1;
}
