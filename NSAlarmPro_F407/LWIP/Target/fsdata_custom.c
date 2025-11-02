#include "lwip/apps/httpd_opts.h"
#include "lwip/apps/fs.h"

#if HTTPD_USE_CUSTOM_FSDATA
/*
 * Provide an empty filesystem when no pre-generated fsdata.c is available.
 * This allows the HTTPD module to compile even if there are no static files
 * to serve yet.  Users can replace this file with generated content from
 * makefsdata when the web assets are ready.
 */
#define FS_ROOT NULL
#define FS_NUMFILES 0
#endif /* HTTPD_USE_CUSTOM_FSDATA */
