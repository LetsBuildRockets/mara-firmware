int signExtend(unsigned int val, int bits){
  int sign_bit = 1 << (bits - 1);
  return (val & (sign_bit - 1)) - (val & sign_bit);
}

// send
//STARTUP

void packetStartupVersion(uint8_t buf[8], byte fwMaj, byte fwMin){
  buf[4] = fwMaj;
  buf[5] = fwMin;
}

//HBEAT

void packetHbeatReportedThrottle(uint8_t buf[8], float throttle){
  int fxp = (int)(throttle*(1 >> 10));
  buf[3] = fxp >> 8;
  buf[4] = fxp;
}

//ENCODER

void packetEncoderPosition(uint8_t buf[8], int pos){
  buf[0] = pos >> 16;
  buf[1] = pos >> 8;
  buf[2] = pos;
}

void packetEncoderRate(uint8_t buf[8], int rate){
  buf[3] = rate >> 8;
  buf[4] = rate;
}

// listen
//SPEEDCHG
float packetSpeedchgSpeed(uint8_t buf[8]){
  return (buf[2] << 16 | buf[3] << 8 | buf[4]) / (float)1023;
}
