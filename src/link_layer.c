// Link layer protocol implementation

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


// MISC
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

        // Leave space for two bytes if escaping
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
// Helper: send supervision frame (SET, UA, DISC)
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
// Helper: read supervision frame (blocking)
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
                if (byte == FLAG) break; // ignore duplicate flag
                buffer[idx++] = byte; // A
                if (byte != A_TX && byte != A_RX) {
                    state = 0;
                    idx = 0;
                    break;
                }
                state = 2;
                break;

            case 2:
                if (byte == FLAG) { state = 1; idx = 0; break; }
                buffer[idx++] = byte; // C
                if (byte != C_SET && byte != C_UA && byte != C_DISC) {
                    state = 0;
                    idx = 0;
                    break;
                }
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
                    else
                    {
                        state = 0;
                        idx = 0;
                    }
                }
                else
                {
                    state = 0;
                    idx = 0;
                }
                break;
            }
        }
    }
}

void sendRR0() {

    unsigned char control;

    control = expectedNs == 0 ? C_RR0 : C_RR1;

    sendSupervisionFrame(A_TX, control);

}

void sendRR1() {

    unsigned char next_ns = expectedNs == 0 ? 1 : 0;

    unsigned char control;

    control = next_ns == 0 ? C_RR0 : C_RR1;

    sendSupervisionFrame(A_TX, control);

}

void sendREJ() {

    unsigned char rej_control;

    rej_control = expectedNs == 0 : C_REJ0 = C_REJ1;

    sendSupervisionFrame(A_TX, rej_control);
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
                return 0;
            }
        }
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
static unsigned char sequenceNumber = 0; // Ns (Tx)

int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char frame[MAX_FRAME_SIZE];
    int frameSize = 0;

    // * builds Information frame Ns (Ns=0 or 1) 
    unsigned char A = A_TX;
    unsigned char C = (sequenceNumber == 0) ? C_I0 : C_I1;

    frame[frameSize++] = FLAG;
    frame[frameSize++] = A;
    frame[frameSize++] = C;

    // * implements error detection (compute BCC over the data packet)
    frame[frameSize++] = calcBCC1(A, C);
    // Add BCC2 to payload before stuffing
    unsigned char temp[MAX_PAYLOAD_SIZE];
    memcpy(temp, buf, bufSize);
    temp[bufSize] = calcBCC2(buf, bufSize);

    // * parses data packet to implement byte stuffing (transparency)
    unsigned char stuffed[STUFFED_BUFFER_SIZE];
    int stuffedSize = bytestuffing(temp, bufSize + 1, stuffed,STUFFED_BUFFER_SIZE);
    if (stuffedSize < 0) return -1;

    memcpy(&frame[frameSize], stuffed, stuffedSize);
    frameSize += stuffedSize;
    frame[frameSize++] = FLAG;

    
    alarmEnabled = 0;
    alarmCount = 0;
    
    struct sigaction sa;
    sa.sa_handler = alarmHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGALRM, &sa, NULL);
    
    unsigned char byte;
    unsigned char buffer[MAX_FRAME_SIZE];
    int state = 0;
    int idx = 0;
    
    
    while(alarmCount < connection.nRetransmissions) {
        
        // * sends I frame 
        if (writeBytesSerialPort(frame, frameSize) < 0)
        {
            perror("[llwrite] Error sending frame");
            return -1;
        }
        
        alarmEnabled = 1;
        alarm(connection.timeout);
        
        // * reads one byte at a time to receive response
        while(alarmEnabled) {

            if(readByteSerialPort(&byte) == 1) {

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
                            unsigned char received_A = buffer[0];
                            unsigned char received_C = buffer[1];
                            unsigned char BCC1 = buffer[2];

                            if (!isValidBCC1(received_A, received_C, BCC1))
                            {
                                state = 0;
                                idx = 0;
                                break;
                            }

                            int expected_rr = (sequenceNumber == 0) ? C_RR1 : C_RR0;
                            int expected_rej = (sequenceNumber == 0) ? C_REJ0 : C_REJ1;

                            // * if negative response (REJ) or no response, resends I frame up to a maximum number of times (retransmission mechanism explained in slide 46)

                            if(received_C == expected_rr) 
                            {

                                // * return success

                                alarm(0);
                                sequenceNumber ^= 1;
                                printf("[llwrite] Sent I frame (Ns=%d, stuffed size=%d)\n", sequenceNumber, frameSize);
                                return bufSize;

                            } 
                            else if (received_C = expected_rej)
                            {
                                alarm(0);
                                alarmCount++;
                                break;
                            }
                            else
                            {
                                state = 0;
                                idx = 0;
                            } 

                            state = 0; idx = 0;
                            break;
                        }
                        else
                        {
                            buffer[idx++] = byte;
                        }
                        break;
                }

            }

        }


        // * return failure 

        printf("[llwrite] Tx - Timeout");
        return bufSize;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
static int expectedNs = 0; // Ns (Rx)

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
                if (byte == FLAG) { state = 0; idx = 0; break; }
                buffer[idx++] = byte;
                state = 5;
                break;

            case 5:
                if (byte == FLAG)
                {
                    int totalSize = idx;
                    int stuffedPayloadSize = totalSize - 3; // A,C,BCC1 removed

                    if (stuffedPayloadSize <= 0)
                    {
                        printf("[llread] Frame too short\n");
                        state = 0; idx = 0;
                        break;
                    }

                    unsigned char A = buffer[0];
                    unsigned char C = buffer[1];
                    unsigned char BCC1 = buffer[2];
                    unsigned char *stuffedData = &buffer[3];

                    if (!isValidBCC1(A, C, BCC1))
                    {

                        // * ignore or send RR(Ns) 

                        sendRR0();

                        printf("[llread] Invalid BCC1\n");
                        state = 0; idx = 0;
                        break;
                    } 

                    else {

                        unsigned char receivedNs = C == C_I1 ? 1 : 0;

                        // * if expected Ns (new frame, no duplicate) {

                        if(expectedNs == receivedNs) {

                            // parse frane to implement byte de-stuffing

                            unsigned char destuffed[STUFFED_BUFFER_SIZE];
                            int destuffedSize = destuff(stuffedData, stuffedPayloadSize, destuffed, STUFFED_BUFFER_SIZE);
                            if (destuffedSize < 0)
                            {
                                printf("[llread] Destuff failed\n");
                                state = 0; idx = 0;
                                break;
                            }
        
                            int payloadSize = destuffedSize - 1;
                            unsigned char received_bcc2 = destuffed[destuffedSize - 1];

                            // compute BCC2 over the data 

                            unsigned char calc_bcc2 = calcBCC2(destuffed, payloadSize);
        
                            if (calc_bcc2 != received_bcc2)
                            {

                                // * send REJ(Ns)

                                sendREJ();
                                printf("[llread] Invalid BCC2\n");
                                state = 0; idx = 0;
                                break;
                            } else {

                                // * reply “send me new frame” by sending RR(Ns+1)
                                sendRR1();
                                // * return data packet 

                                // update 
                                expectedNs = expectedNs == 0 ? 1 : 0;

                                memcpy(packet, destuffed, payloadSize);
                                printf("[llread] Frame OK, payload = %d bytes\n", payloadSize);
                                return payloadSize;

                            }

                        } else {

                            // TODO:  ignore or send RR(Ns) 

                            sendRR0();
                            printf("[llread] Not expected Ns, duplicate");
                            state = 0; idx = 0;
                            break;

                        }

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
