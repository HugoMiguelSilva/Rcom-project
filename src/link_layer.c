
#include "link_layer.h"
#include "serial_port.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#define TIMEOUT_SECS 3

static LinkLayer connection;
static int alarmEnabled = 0;
static int alarmCount = 0;
static int expectedNs = 0;
static unsigned char sequenceNumber = 0; // Ns

#define _POSIX_SOURCE 1 // POSIX compliant source

////////////////////////////////////////////////
// Timeout handler
////////////////////////////////////////////////
void alarmHandler(int signal)
{
    alarmEnabled = 0;
    alarmCount++;
}

////////////////////////////////////////////////
// Byte Stuffing
////////////////////////////////////////////////
int bytestuffing(const unsigned char *data, size_t length, unsigned char *out, int outMax)
{
    int outidx = 0;
    for (size_t i = 0; i < length; i++)
    {
        unsigned char byte = data[i];

        if (outidx >= outMax - 1)
        {
            fprintf(stderr, "[bytestuffing] out of space\n");
            return -1;
        }

        if (byte == FLAG)
        {
            out[outidx++] = ESC;
            out[outidx++] = ESCAUX;
        }
        else if (byte == ESC)
        {
            out[outidx++] = ESC;
            out[outidx++] = ESCAUX2;
        }
        else
        {
            out[outidx++] = byte;
        }
    }
    return outidx;
}

////////////////////////////////////////////////
// Byte Destuffing
////////////////////////////////////////////////
int destuff(const unsigned char *data, size_t length, unsigned char *out, int outMax)
{
    int outidx = 0;
    for (size_t i = 0; i < length; i++)
    {
        if (outidx >= outMax)
        {
            fprintf(stderr, "[destuff] out of space\n");
            return -1;
        }

        if (data[i] == ESC && i + 1 < length)
        {
            if (data[i + 1] == ESCAUX)
                out[outidx++] = FLAG;
            else if (data[i + 1] == ESCAUX2)
                out[outidx++] = ESC;
            i++;
        }
        else
        {
            out[outidx++] = data[i];
        }
    }
    return outidx;
}

////////////////////////////////////////////////
// Helper: send supervision frame (SET, UA, DISC, RR, REJ)
////////////////////////////////////////////////
static void sendSupervisionFrame(unsigned char address, unsigned char control)
{
    unsigned char frame[5];
    frame[0] = FLAG;
    frame[1] = address;
    frame[2] = control;
    frame[3] = calcBCC1(address, control);
    frame[4] = FLAG;

    writeBytesSerialPort(frame, 5);
}

////////////////////////////////////////////////
// Simple helpers to send RR and REJ frames
////////////////////////////////////////////////
static void sendRR(int expectedNs)
{
    unsigned char control = (expectedNs == 0) ? C_RR0 : C_RR1;
    sendSupervisionFrame(A_RX, control);
    printf("[llread] Sent RR(%d)\n", expectedNs);
}

static void sendREJ(int expectedNs)
{
    unsigned char control = (expectedNs == 0) ? C_REJ0 : C_REJ1;
    sendSupervisionFrame(A_RX, control);
    printf("[llread] Sent REJ(%d)\n", expectedNs);
}

////////////////////////////////////////////////
// Read a supervision frame (UA, RR, REJ, etc.)
////////////////////////////////////////////////
static int readSupervisionFrame(unsigned char *address, unsigned char *control)
{
    unsigned char byte;
    unsigned char buffer[5];
    int state = 0;
    int idx = 0;

    while (1)
    {
        if (readByteSerialPort(&byte) == 1)
        {
            switch (state)
            {
            case 0:
                if (byte == FLAG) state = 1;
                break;

            case 1:
                if (byte == FLAG) break;
                buffer[idx++] = byte; // A
                if (byte != A_TX && byte != A_RX) { state = 0; idx = 0; break; }
                state = 2;
                break;

            case 2:
                if (byte == FLAG) { state = 1; idx = 0; break; }
                buffer[idx++] = byte; // C
                state = 3;
                break;

            case 3:
                if (byte == FLAG) { state = 1; idx = 0; break; }
                buffer[idx++] = byte; // BCC
                state = 4;
                break;

            case 4:
                if (byte == FLAG)
                {
                    unsigned char A = buffer[0];
                    unsigned char C = buffer[1];
                    unsigned char BCC = buffer[2];
                    if (isValidBCC1(A, C, BCC))
                    {
                        *address = A;
                        *control = C;
                        return 0;
                    }
                    else { state = 0; idx = 0; }
                }
                else { state = 0; idx = 0; }
                break;
            }
        }
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    alarmEnabled = 0;
    alarmCount = 0;
    connection = connectionParameters;

    if (openSerialPort(connection.serialPort, connection.baudRate) < 0)
    {
        perror("Error opening serial port");
        return -1;
    }

    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGALRM, &sa, NULL);

    unsigned char address, control;

    if (connection.role == LlTx)
    {
        // Transmitter
        while (alarmCount < connection.nRetransmissions)
        {
            sendSupervisionFrame(A_TX, C_SET);
            printf("[llopen - TX] SET frame sent\n");

            alarmEnabled = 1;
            alarm(connection.timeout);

            while (alarmEnabled)
            {
                if (readSupervisionFrame(&address, &control) == 0 &&
                    address == A_RX && control == C_UA)
                {
                    alarm(0);
                    sequenceNumber = 0; 
                    printf("[llopen - TX] UA received\n");
                    return 0;
                }
            }

            printf("[llopen - TX] Timeout %d/%d\n", alarmCount, connection.nRetransmissions);
        }

        printf("[llopen - TX] Connection failed\n");
        return -1;
    }
    else
    {
        // Receiver
        printf("[llopen - RX] Waiting for SET...\n");
        while (1)
        {
            if (readSupervisionFrame(&address, &control) == 0 &&
                address == A_TX && control == C_SET)
            {
                printf("[llopen - RX] SET received\n");
                sendSupervisionFrame(A_RX, C_UA);
                printf("[llopen - RX] UA sent\n");
                expectedNs = 0;
                return 0;
            }
        }
    }
}

////////////////////////////////////////////////
// LLWRITE  (Stop-and-Wait implemented)
////////////////////////////////////////////////


int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = 0;

    unsigned char A = A_TX;
    unsigned char C = (sequenceNumber == 0) ? C_I0 : C_I1;

    frame[frameSize++] = FLAG;
    frame[frameSize++] = A;
    frame[frameSize++] = C;
    frame[frameSize++] = calcBCC1(A, C);

    // Append BCC2 before stuffing
    unsigned char temp[MAX_PAYLOAD_SIZE];
    memcpy(temp, buf, bufSize);
    temp[bufSize] = calcBCC2(buf, bufSize);

    unsigned char stuffed[STUFFED_BUFFER_SIZE];
    int stuffedSize = bytestuffing(temp, bufSize + 1, stuffed, STUFFED_BUFFER_SIZE);
    if (stuffedSize < 0) return -1;

    memcpy(&frame[frameSize], stuffed, stuffedSize);
    frameSize += stuffedSize;
    frame[frameSize++] = FLAG;

    // Stop-and-Wait: send and wait for RR/REJ
    alarmCount = 0;

    while (alarmCount < connection.nRetransmissions)
    {
        if (writeBytesSerialPort(frame, frameSize) < 0) {
            perror("[llwrite] Write failed");
            return -1;
        }
        printf("[llwrite] Sent I frame Ns=%d (%d bytes)\n", sequenceNumber, frameSize);
        
        alarmEnabled = 1;
        alarm(connection.timeout);
        
        unsigned char addr, ctrl;
        
        while (alarmEnabled)
        {
            if (readSupervisionFrame(&addr, &ctrl) == 0)
            {
                printf("readSupervisionFrame(&addr, &ctrl) == 0\n"); 
                int expected_rr = (sequenceNumber == 0) ? C_RR1 : C_RR0;
                int expected_rej = (sequenceNumber == 0) ? C_REJ0 : C_REJ1;
                
                if (ctrl == expected_rr)
                {
                    alarm(0);
                    printf("[llwrite] RR received -> frame accepted\n");
                    sequenceNumber ^= 1; // toggle Ns
                    return bufSize;
                }
                else if (ctrl == expected_rej)
                {
                    alarm(0);
                    printf("[llwrite] REJ received -> retransmit\n");
                    break; // retry loop
                }
                else printf("[llwrite] Unexpected frame: A=0x%02X C=0x%02X\n", addr, ctrl);
                
            }
        }
        
        alarmCount++;
        printf("[llwrite] Timeout/retry %d/%d\n", alarmCount, connection.nRetransmissions);
    }


    printf("[llwrite] Transmission failed after retries\n"); //perguntar ao stor sobre isto nao deve estar correto mas funciona

    unsigned char give_up_frame[5];
    give_up_frame[0] = FLAG;
    give_up_frame[1] = A_TX;
    give_up_frame[2] = C_DISC; 
    give_up_frame[3] = calcBCC1(A_TX, C_DISC);
    give_up_frame[4] = FLAG;
    
    writeBytesSerialPort(give_up_frame, 5);


    return -1;
}

////////////////////////////////////////////////
// LLREAD (Receiver side, sends RR/REJ)
////////////////////////////////////////////////


int llread(unsigned char *packet)
{
    unsigned char byte;
    unsigned char buffer[MAX_FRAME_SIZE];
    int state = 0;
    int idx = 0;


    while (1)
    {
        if (readByteSerialPort(&byte) == 1)
        {
            switch (state)
            {
            case 0:
                if (byte == FLAG) { state = 1; idx = 0; }
                break;

            case 1:
                if (byte == FLAG) break;
                if (byte != A_TX) { state = 0; idx = 0; break; }
                buffer[idx++] = byte;
                state = 2;
                break;

            case 2:
                if (byte == FLAG) { state = 1; idx = 0; break; }

                if (byte == C_DISC) {
                    printf("[llread] DISC frame received while waiting for data\n");
                    return -2; 
                }

                if (byte != C_I0 && byte != C_I1) { state = 0; idx = 0; break; }
                buffer[idx++] = byte;
                state = 3;
                break;

            case 3:
                if (byte == FLAG) { state = 1; idx = 0; break; }
                buffer[idx++] = byte;
                state = 4;
                break;

            case 4:
                if (byte == FLAG)
                {
                    unsigned char A = buffer[0];
                    unsigned char C = buffer[1];
                    unsigned char BCC1 = buffer[2];
                    unsigned char *stuffedData = &buffer[3];
                    int stuffedPayloadSize = idx - 3;

                    if (!isValidBCC1(A, C, BCC1))
                    {
                        printf("[llread] Invalid BCC1 -> REJ(%d)\n", expectedNs);
                        sendREJ(expectedNs);
                        state = 0; idx = 0;
                        break;
                    }

                    unsigned char receivedNs = (C == C_I1) ? 1 : 0;

                    unsigned char destuffed[STUFFED_BUFFER_SIZE];
                    int destuffedSize = destuff(stuffedData, stuffedPayloadSize, destuffed, STUFFED_BUFFER_SIZE);
                    if (destuffedSize < 0)
                    {
                        printf("[llread] Destuff failed -> REJ(%d)\n", expectedNs);
                        sendREJ(expectedNs);
                        state = 0; idx = 0;
                        break;
                    }

                    int payloadSize = destuffedSize - 1;
                    unsigned char received_bcc2 = destuffed[destuffedSize - 1];
                    
                    unsigned char calc_bcc2 = calcBCC2(destuffed, payloadSize);

                    if (calc_bcc2 != received_bcc2)
                    {
                        printf("[llread] Invalid BCC2 -> REJ(%d)\n", expectedNs);
                        sendREJ(expectedNs);
                        state = 0; idx = 0;
                        break;
                    }
                    printf("[llread] Received frame Ns=%d, expected Ns=%d\n", receivedNs, expectedNs);

                    if (receivedNs == expectedNs)
                    {
                        memcpy(packet, destuffed, payloadSize);
                        printf("[llread] Frame OK (Ns=%d), sending RR(%d)\n", receivedNs, (expectedNs ^ 1));
                        expectedNs ^= 1;
                        sendRR(expectedNs);
                        printf("[llread] RR(%d) sent successfully\n", expectedNs);
                        return payloadSize;
                    }
                    else
                    {
                        printf("[llread] Duplicate frame, resend RR(%d)\n", expectedNs);
                        sendRR(expectedNs);
                        state = 0; idx = 0;
                    }
                }
                else
                {
                    if (idx >= MAX_FRAME_SIZE)
                    {
                        printf("[llread] Frame too long\n");
                        state = 0; idx = 0;
                        break;
                    }
                    buffer[idx++] = byte;
                }
                break;
            }
        }
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    unsigned char address, control;

    if (connection.role == LlTx)
    {
        sendSupervisionFrame(A_TX, C_DISC);
        printf("[llclose - TX] DISC sent\n");

        while (1)
        {
            if (readSupervisionFrame(&address, &control) == 0 &&
                address == A_RX && control == C_DISC)
            {
                printf("[llclose - TX] DISC received\n");
                break;
            }
        }

        sendSupervisionFrame(A_TX, C_UA);
        printf("[llclose - TX] UA sent\n");
    }
    else
    {
        printf("[llclose - RX] Waiting for DISC\n");
        while (1)
        {
            if (readSupervisionFrame(&address, &control) == 0 &&
                address == A_TX && control == C_DISC)
            {
                printf("[llclose - RX] DISC received\n");
                sendSupervisionFrame(A_RX, C_DISC);
                break;
            }
        }

        while (1)
        {
            if (readSupervisionFrame(&address, &control) == 0 &&
                address == A_TX && control == C_UA)
            {
                printf("[llclose - RX] UA received\n");
                break;
            }
        }
    }

    closeSerialPort();
    return 0;
}
