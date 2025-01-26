#ifndef DEFINES_H
#define DEFINES_H

#include <stdio.h>
#include <status.h>
#include <commConfig.h>
#include <commTypes.h>

const char* ip = "199.0.1.1";
const char* ip_pc = "199.0.1.2";
const char* netmask = "255.255.0.0";
const char* gateway = "199.199.0.0";
const int port = 9600;
const int port_pc = 9601;

NetworkType nrfNetwork;
uint8_t pipeNum = 0;
packetGeneric packetSend;
packetGeneric packetFull;
packetBStConfig packetConfig;

#endif /* DEFINES_H */