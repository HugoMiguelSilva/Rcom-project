// Application layer protocol implementation

#include "application_layer.h"

#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"

// TX AUX FUNCTIONS
                    
FILE * openFile(const char *filename) 
{
    return fopen(filename, "r");
}

long getFileSize(FILE * file)
{
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET); // reset pointer to start of file
    return size;
}

int buildCtrlPck(unsigned char* packet, const char *filename, long file_size, int start)
{
    int i = 0;
    int length = strlen(filename);
    
    packet[i++] = start ? C_START : C_END;
    packet[i++] = T_SIZE; 
    packet[i++] = SIZE_FIELD_LENGTH; 
    packet[i++] = file_size & 0xFF;
    packet[i++] = (file_size >> 8) & 0xFF;
    packet[i++] = (file_size >> 16) & 0xFF;
    packet[i++] = (file_size >> 24) & 0xFF;
    packet[i++] = T_NAME;
    packet[i++] = length;
    
    memcpy(&packet[i], filename, length);
    
    i += length;

    if( i >= MAX_PAYLOAD_SIZE) {
        fprintf(stderr, "[APP] filename too long\n");
    }
    
    return i;
}

int buildDataPck(unsigned char* packet, unsigned char *buffer, int buffer_size) 
{
    int i = 0;
    
    packet[i++] = C_DATA;
    packet[i++] = (buffer_size >> 8) & 0xFF;
    packet[i++] = buffer_size & 0xFF;
    
    memcpy(&packet[i], buffer, buffer_size);
    
    i += buffer_size;
    
    return i;
}

int readFragFile(FILE * file, unsigned char *buffer, int maxSize)
{
    int nBytes = fread(buffer, 1, maxSize, file);
    
    if(ferror(file)) return -1;
    
    return nBytes;
}

// RX AUX FUNCTIONS

FILE * createFile(const char *filename)
{
    return fopen(filename, "a");
}

int writeFile(FILE* file, unsigned char *buffer, int size)
{
    if(fwrite(buffer, 1, size, file) != size) {
        return -1;
    }

    return 0;
}

int extractCtrlPck(unsigned char *packet, char *filename, long *file_size)
{
    if(packet[1] != T_SIZE) return -1;
    *file_size = (long)(packet[3] | packet[4] << 8 | packet[5] << 16 | packet[6] << 24);
    if(packet[7] != T_NAME) return -1;
    int length = packet[8];
    memcpy(filename, &packet[9], length);
    filename[length] = '\0';

    return 0;
}

int extractDataPck(unsigned char *packet, unsigned char *buffer)
{
    int data_size = (packet[1] << 8) | packet[2];
    memcpy(buffer, &packet[3], data_size);
    return data_size;
}

////////////////////////////////////////////////
// APPLICATIONLAYER
////////////////////////////////////////////////
void applicationLayer(const char *serialPort, const char *role, int baudRate,int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    strcpy(ll.serialPort, serialPort);
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;
    ll.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;
    FILE * file;
    long file_size;
    unsigned char ctrl_packet[MAX_PAYLOAD_SIZE];
    int ctrl_packet_size;
    unsigned char data_packet[MAX_DATA_PACKET_SIZE];
    int data_packet_size;
    int nBytes;
    unsigned char frag_buffer[MAX_PAYLOAD_SIZE];
    unsigned char packet_rx[MAX_DATA_PACKET_SIZE];
    int end_reached = FALSE;
    char filename_rx[256];
    char end_filename[256];
    long end_size;
    
    // Open link
    if (llopen(ll) < 0) {
        fprintf(stderr, "[APP] Failed to open link layer\n");
        return;
    }
    
    printf("[APP] Link layer opened successfully\n");
    
    if(ll.role == LlTx) {
        
        file = openFile(filename);

        if(!file) {
            fprintf(stderr, "[APP] Could not open file \n");
            return;
        }
        
        file_size = getFileSize(file);
        
        ctrl_packet_size = buildCtrlPck(ctrl_packet, filename, file_size, TRUE); // TRUE for start packet

        if(llwrite(ctrl_packet, ctrl_packet_size) < 0) {
            fprintf(stderr, "[APP] Failed to write START control packet\n");
        } else {
            printf("[APP] START Control packet written succesfully\n");
        }
        
        nBytes = readFragFile(file, frag_buffer, MAX_PAYLOAD_SIZE);

        while(nBytes > 0) {

            data_packet_size = buildDataPck(data_packet, frag_buffer, nBytes);

            if (llwrite(data_packet, data_packet_size) < 0) {
                fprintf(stderr, "[APP] Failed to write data packet\n");
                return;
            } else {
                printf("[APP] Data packet written succesfully\n");
            }

            nBytes = readFragFile(file, frag_buffer, MAX_PAYLOAD_SIZE);
            
        }   

        ctrl_packet_size = buildCtrlPck(ctrl_packet, filename, file_size, FALSE); // FALSE for end packet

        if(llwrite(ctrl_packet, ctrl_packet_size) < 0) {
            fprintf(stderr, "[APP] Failed to write END control packet\n");
        } else {
            printf("[APP] END Control packet written succesfully\n");
        }
        
    } else {
        
        while(!end_reached) {

            ctrl_packet_size = llread(packet_rx);

            if(ctrl_packet_size <= 0) break;

            switch (packet_rx[0])
            {
                case C_START:
                    if(extractCtrlPck(packet_rx, filename_rx, &file_size) < 0) {
                        fprintf(stderr, "[APP] Control packet is malformed\n");
                        return;
                    }
                    
                    file = createFile(filename);

                    if(!file) {
                        fprintf(stderr, "[APP] Could not create file \n");
                        return;
                    }

                    break;

                case C_DATA:
                    data_packet_size = extractDataPck(packet_rx, frag_buffer);
                    if(writeFile(file, frag_buffer, data_packet_size) < 0) {
                        fprintf(stderr, "[APP] File was not written\n");
                        return;
                    }
                    break;

                case C_END:
                    if(extractCtrlPck(packet_rx, end_filename, &end_size) < 0 || strcmp(end_filename, filename_rx) != 0 || end_size != file_size) {
                        fprintf(stderr, "[APP] END control packet does not match START\n");
                    }
                    end_reached = true;
                    break;
            }
            
        }
        
    }
    
    fclose(file);
    
    
    /*
    
    if (ll.role == LlTx) {
        // Transmitter - Send multiple frames to test sequence numbers
        
        // Test data that includes special bytes for stuffing
        unsigned char frame1[] = {0x11, 0x7E, 0x7D, 0x22, 0x33}; // Contains 0x7E and 0x7D
        unsigned char frame2[] = {0x44, 0x55, 0x66, 0x77, 0x88};
        unsigned char frame3[] = {0x99, 0xAA, 0xBB, 0xCC, 0xDD};
        unsigned char frame4[] = {0x12, 0x34, 0x56, 0x78, 0x90};
        
        printf("[APP] Sending frame 1: ");
        
        
        
        if (llwrite(frame1, sizeof(frame1)) < 0) {
            fprintf(stderr, "[APP] Failed to send frame 1\n");
            } else {
                printf("[APP] Frame 1 sent successfully\n");
                return;
        }
        
        printf("[APP] Sending frame 2: ");
        
        if (llwrite(frame2, sizeof(frame2)) < 0) {
            fprintf(stderr, "[APP] Failed to send frame 2\n");
        } else {
            printf("[APP] Frame 2 sent successfully\n");
        }
                
        printf("[APP] Sending frame 3: ");
                
        if (llwrite(frame3, sizeof(frame3)) < 0) {
            fprintf(stderr, "[APP] Failed to send frame 3\n");
        } else {
            printf("[APP] Frame 3 sent successfully\n");
        }
                        
        printf("[APP] Sending frame 4: ");
        
        if (llwrite(frame4, sizeof(frame4)) < 0) {
            fprintf(stderr, "[APP] Failed to send frame 4\n");
            } else {
                printf("[APP] Frame 4 sent successfully\n");
            }
                
            } else {

                printf("[APP] Waiting for frames...\n");

                unsigned char buffer[1024];
                int frameCount = 0;
                
                // Try to receive 4 frames
                while (frameCount < 4) {
                    int bytesRead = llread(buffer);
                    if (bytesRead > 0) {
                        printf("[APP] Received frame %d (%d bytes): ", frameCount + 1, bytesRead);
                //printHex(buffer, bytesRead);
                        frameCount++;
                    } else if (bytesRead == -1) {
                        printf("[APP] Error reading frame\n");
                        break;
                    } else if (bytesRead == -2) {
                        printf("[APP] DISC frame received - closing\n");
                        break;
                    }
                }
                        
                printf("[APP] Received %d frames total\n", frameCount);
            }
                        
            */
                       
    // Close link
    printf("[APP] Closing link...\n");
    llclose();
    printf("[APP] Link closed\n");
}
