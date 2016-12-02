#include "talon_rio.c"

#define PART_DEVID 0x0000003F // 63 device ids on the bus
#define PART_PACKTYPE (~PART_DEVID) // close enough

// send
#define PACKET_STARTUP 0x02041500
#define PACKET_STARTUP_LEN 6
void packetStartupVersion(uint8_t buf[8], byte fwMaj, byte fwMin);

#define PACKET_HBEAT 0x02041400
#define PACKET_HBEAT_LEN 8
void packetHbeatReportedThrottle(uint8_t buf[8], float throttle);

#define PACKET_ENCODER 0x02041480
#define PACKET_ENCODER_LEN 8
void packetEncoderPosition(uint8_t buf[8], int pos);
void packetEncoderRate(uint8_t buf[8], int rate);

// listen
#define PACKET_SPEEDCHG 0x02040000
float packetSpeedchgSpeed(uint8_t buf[8]);

#define PACKET_ENABLE 0x0280
#define PACKET_DISABLE 0x0
