// Application layer protocol implementation

#include "application_layer.h"

#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(ll.serialPort, serialPort);
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;
    ll.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;

    // Open link
    if (llopen(ll) < 0) {
        fprintf(stderr, "[APP] Failed to open link layer\n");
        return;
    }

    printf("[APP] Link layer opened successfully\n");

    if (ll.role == LlTx) {
        //Transmitter
        unsigned char testData[] = {0x11,0x7E,0x7D,0x22};
        if (llwrite(testData, sizeof(testData) - 1) < 0) {
            fprintf(stderr, "[APP] Failed to send test frame\n");
        } else {
            printf("[APP] Test frame sent: \"%s\"\n", testData);
        }

    } else {
        //Receiver
        unsigned char buffer[1024];
        int bytesRead = llread(buffer);
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // null-terminate for printing
            printf("[APP] Received frame: \"%s\"\n", buffer);
        } else {
            printf("[APP] No frame received or error\n");
        }
    }

    // 3️⃣ Close link
    llclose();
    printf("[APP] Link closed\n");

}
