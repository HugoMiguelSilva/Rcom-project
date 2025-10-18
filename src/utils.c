#include "utils.h"

unsigned char calcBCC1(unsigned char A, unsigned char C) {
    return A ^ C;
}

unsigned char calcBCC2(const unsigned char *data, size_t length) {
    unsigned char bcc = 0x00;
    for (size_t i = 0; i < length; i++) {
        bcc ^= data[i];
    }
    return bcc;
}

bool isValidBCC1(unsigned char A, unsigned char C, unsigned char BCC1) {
    return (A ^ C) == BCC1;
}

