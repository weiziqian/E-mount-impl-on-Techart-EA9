/* optical_file.h -- the file served through messages 0x3A/0x3B. */
#ifndef OPTICAL_FILE_H
#define OPTICAL_FILE_H

#include <stdint.h>

#define OPTICAL_FILE_LEN    2584u
#define OPTICAL_PAGE08_LEN  500u

extern const uint8_t optical_file[OPTICAL_FILE_LEN];
extern const uint8_t optical_page08[OPTICAL_PAGE08_LEN];

#endif /* OPTICAL_FILE_H */
