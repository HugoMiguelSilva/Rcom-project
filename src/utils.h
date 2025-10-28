

#ifndef UTILS_H
#define UTILS_H

#include <stddef.h>
#include <stdbool.h>

// === Frame Special Bytes ===
#define FLAG 0x7E
#define ESC  0x7D
#define ESCAUX 0x5E
#define ESCAUX2 0x5D


// === Address Field ===
#define A_TX 0x03  // Commands from transmitter / replies from receiver
#define A_RX 0x01  // Commands from receiver / replies from transmitter

// === Control Field Values ===
#define C_SET  0x03  // Set up
#define C_UA   0x07  // Unnumbered Acknowledgment
#define C_DISC 0x0B  // Disconnect

// Information frames (I frames) with N(S) = 0 or 1
#define C_I0 0x00  // I frame, sequence number 0
#define C_I1 0x40  // I frame, sequence number 1

// RR and REJ with N(r) = 0 or 1
#define C_RR0  0x05
#define C_RR1  0x85
#define C_REJ0 0x01
#define C_REJ1 0x81

// Max frame size
#define MAX_FRAME_SIZE 2048

// Control Packet 
#define C_START 1
#define C_END 3
#define T_SIZE 0
#define T_NAME 1

// Data Packet
#define C_DATA 2

// Max data packet size
#define MAX_DATA_PACKET_SIZE 65535

#define SIZE_FIELD_LENGTH 4

// === Helper Functions ===
unsigned char calcBCC1(unsigned char A, unsigned char C);
unsigned char calcBCC2(const unsigned char *data, size_t length);

bool isValidBCC1(unsigned char A, unsigned char C, unsigned char BCC1);

#endif
