// Reference
// https://raw.githubusercontent.com/Snaipe/fmem/master/src/fmem-winapi-tmpfile.c

#ifdef _WIN32
#include <io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <windows.h>
#include <stdio.h>

FILE *open_memfile(const char* fname, const char *mode) {
    HANDLE handle = CreateFileA(fname,
                                GENERIC_READ | GENERIC_WRITE,
                                0,
                                NULL,
                                CREATE_ALWAYS,
                                FILE_ATTRIBUTE_TEMPORARY | FILE_FLAG_DELETE_ON_CLOSE,
                                NULL);

    if (handle == INVALID_HANDLE_VALUE)
        return NULL;

    int fd = _open_osfhandle((intptr_t) handle, _O_RDWR);
    if (fd == -1) {
        CloseHandle(handle);
        return NULL;
    }

    FILE *f = _fdopen(fd, mode);
    if (!f) _close(fd);
    return f;
}
#endif

#ifdef __linux__

#include <stdio.h>

// https://man7.org/linux/man-pages/man3/fmemopen.3.html
// FILE *fmemopen(void *buf, size_t size, const char *mode);
FILE *open_memfile(const char* fname, const char *mode)  {
	FILE *f = fmemopen(nullptr, 8192, mode);
	return f;
}

#endif
