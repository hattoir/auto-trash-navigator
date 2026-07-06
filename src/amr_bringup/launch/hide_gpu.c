#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <dlfcn.h>
#include <errno.h>
#include <dirent.h>

static int (*orig_open)(const char *, int, ...) = NULL;
static int (*orig_openat)(int, const char *, int, ...) = NULL;
static DIR* (*orig_opendir)(const char *) = NULL;
static int in_open = 0;
static int in_openat = 0;
static int in_opendir = 0;

int open(const char *pathname, int flags, ...) {
    if (pathname && (strstr(pathname, "/dev/dri") != NULL || strstr(pathname, "/dev/nvidia") != NULL)) {
        errno = ENOENT;
        return -1;
    }
    
    if (!orig_open) {
        if (in_open) return -1;
        in_open = 1;
        orig_open = (int (*)(const char *, int, ...))dlsym(RTLD_NEXT, "open");
        in_open = 0;
    }
    
    return orig_open(pathname, flags);
}

int openat(int dirfd, const char *pathname, int flags, ...) {
    if (pathname && (strstr(pathname, "/dev/dri") != NULL || strstr(pathname, "/dev/nvidia") != NULL)) {
        errno = ENOENT;
        return -1;
    }
    
    if (!orig_openat) {
        if (in_openat) return -1;
        in_openat = 1;
        orig_openat = (int (*)(int, const char *, int, ...))dlsym(RTLD_NEXT, "openat");
        in_openat = 0;
    }
    
    return orig_openat(dirfd, pathname, flags);
}

DIR *opendir(const char *name) {
    if (name && (strstr(name, "/dev/dri") != NULL || strstr(name, "/dev/nvidia") != NULL)) {
        errno = ENOENT;
        return NULL;
    }
    
    if (!orig_opendir) {
        if (in_opendir) return NULL;
        in_opendir = 1;
        orig_opendir = (DIR* (*)(const char *))dlsym(RTLD_NEXT, "opendir");
        in_opendir = 0;
    }
    
    return orig_opendir(name);
}
