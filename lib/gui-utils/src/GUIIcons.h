static const unsigned char PROGMEM ic_bluetooth_on[] = {
    B00000000,
    B00011100,
    B01010110,
    B00111100,
    B00111000,
    B00111100,
    B01010110,
    B00011100,
};

static const unsigned char PROGMEM ic_bluetooth_pair[] = {
    B00000000,
    B00011100,
    B01010110,
    B00111100,
    B10111001,
    B00111100,
    B01010110,
    B00011100,
};

static const unsigned char PROGMEM ic_wifi_on[] = {
    B00000000,
    B00011000,
    B00111100,
    B01000010,
    B10011001,
    B00100100,
    B01000010,
    B00011000,
};

static const unsigned char PROGMEM ic_data_on[] = {
    B00000000,
    B01100110,
    B11110110,
    B01100110,
    B01100110,
    B01101111,
    B01100110,
    B00000000,
};

static const unsigned char PROGMEM ic_pref_save[] = {
    B00000000,
    B01111000,
    B01001100,
    B01000110,
    B01000010,
    B01000010,
    B01111110,
    B00000000,
};

static const unsigned char PROGMEM ic_sensor_live[] = {
    B00000000,
    B00101010,
    B01010101,
    B10101010,
    B01010101,
    B10101010,
    B01010100,
    B00000000,
};

static unsigned char Smileface[] = {
  0x00, 0xF0, 0x0F, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x80, 0x03, 0xC0, 0x01, 
  0xC0, 0x00, 0x80, 0x03, 0x60, 0x00, 0x00, 0x06, 0x30, 0x00, 0x00, 0x0C, 
  0x18, 0x00, 0x00, 0x18, 0x0C, 0x0C, 0x30, 0x30, 0x04, 0x38, 0x1C, 0x30, 
  0x06, 0x30, 0x0C, 0x60, 0x02, 0x00, 0x00, 0x60, 0x02, 0x00, 0x00, 0x40, 
  0x03, 0x04, 0x20, 0xC0, 0x03, 0x0E, 0x70, 0xC0, 0x01, 0x1B, 0xD8, 0xC0, 
  0x01, 0x0E, 0x70, 0x80, 0x01, 0x04, 0x20, 0x80, 0x01, 0x00, 0x00, 0xC0, 
  0x03, 0x00, 0x00, 0xC0, 0x03, 0x80, 0x01, 0xC0, 0x02, 0xF0, 0x0F, 0x40, 
  0x02, 0x1C, 0x38, 0x60, 0x06, 0x06, 0x60, 0x60, 0x04, 0x02, 0x40, 0x30, 
  0x0C, 0x00, 0x00, 0x30, 0x18, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00, 0x0C, 
  0x60, 0x00, 0x00, 0x06, 0xC0, 0x00, 0x80, 0x03, 0x80, 0x03, 0xE0, 0x01, 
  0x00, 0x3E, 0x7E, 0x00, 0x00, 0xF0, 0x0F, 0x00, 
  };

  static unsigned char Smileface2[] = {
  0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 
  0x00, 0xE0, 0x0F, 0xF0, 0x07, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x0F, 0x00, 
  0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x70, 0x00, 
  0x00, 0x07, 0x00, 0x00, 0xE0, 0x00, 0x80, 0x03, 0x00, 0x00, 0xC0, 0x01, 
  0xC0, 0x01, 0x00, 0x00, 0x80, 0x03, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 
  0x70, 0x80, 0x00, 0x00, 0x01, 0x0E, 0x38, 0x80, 0x03, 0xC0, 0x01, 0x0C, 
  0x18, 0x80, 0x0F, 0xF0, 0x01, 0x18, 0x1C, 0x00, 0x1E, 0x78, 0x00, 0x38, 
  0x0C, 0x00, 0x08, 0x10, 0x00, 0x30, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x30, 
  0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 
  0x06, 0x00, 0x00, 0x00, 0x00, 0x60, 0x07, 0xC0, 0x03, 0xC0, 0x03, 0xE0, 
  0x03, 0xE0, 0x03, 0xC0, 0x07, 0xC0, 0x03, 0x60, 0x06, 0x60, 0x06, 0xC0, 
  0x03, 0x60, 0x07, 0xE0, 0x06, 0xC0, 0x03, 0xE0, 0x03, 0xC0, 0x07, 0xC0, 
  0x03, 0xC0, 0x01, 0x80, 0x03, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xC0, 
  0x03, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0xC0, 
  0x07, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x06, 0x00, 0xE0, 0x07, 0x00, 0x60, 
  0x06, 0x00, 0xFC, 0x3F, 0x00, 0x60, 0x06, 0x00, 0x3F, 0xFC, 0x00, 0x60, 
  0x0C, 0x80, 0x07, 0xE0, 0x01, 0x30, 0x0C, 0xC0, 0x01, 0x80, 0x03, 0x30, 
  0x1C, 0xC0, 0x00, 0x00, 0x03, 0x38, 0x18, 0x00, 0x00, 0x00, 0x00, 0x18, 
  0x30, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x70, 0x00, 0x00, 0x00, 0x00, 0x0E, 
  0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0x01, 0x00, 0x00, 0x80, 0x03, 
  0x80, 0x03, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x07, 0x00, 0x00, 0xE0, 0x00, 
  0x00, 0x0E, 0x00, 0x00, 0x70, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 
  0x00, 0xF8, 0x00, 0x00, 0x0F, 0x00, 0x00, 0xE0, 0x0F, 0xF0, 0x07, 0x00, 
  0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x1F, 0x00, 0x00, 
  };

static unsigned char Smileface3[] = {                                       //28x28
  0x00, 0xFC, 0x03, 0x00, 0x00, 0x0F, 0x0F, 0x00, 0xC0, 0x00, 0x38, 0x00, 
  0x60, 0x00, 0x60, 0x00, 0x10, 0x00, 0xC0, 0x00, 0x08, 0x00, 0x80, 0x01, 
  0x0C, 0x02, 0x04, 0x03, 0x04, 0x0E, 0x07, 0x02, 0x02, 0x08, 0x01, 0x06, 
  0x02, 0x00, 0x00, 0x04, 0x03, 0x00, 0x00, 0x0C, 0x01, 0x03, 0x0C, 0x0C, 
  0x81, 0x05, 0x1A, 0x08, 0x81, 0x07, 0x1E, 0x08, 0x01, 0x03, 0x0C, 0x08, 
  0x01, 0x00, 0x00, 0x08, 0x03, 0x00, 0x00, 0x0C, 0x03, 0xF8, 0x01, 0x0C, 
  0x02, 0x0E, 0x07, 0x04, 0x02, 0x03, 0x0C, 0x06, 0x04, 0x01, 0x08, 0x02, 
  0x0C, 0x00, 0x00, 0x03, 0x08, 0x00, 0x80, 0x01, 0x10, 0x00, 0xC0, 0x00, 
  0x60, 0x00, 0x60, 0x00, 0xC0, 0x00, 0x38, 0x00, 0x00, 0x0F, 0x0F, 0x00, 
  0x00, 0xFC, 0x03, 0x00, };

  static unsigned char Smileface4[] = {                                     //32x32
  0x00, 0xF0, 0x0F, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x80, 0x03, 0xC0, 0x01, 
  0xC0, 0x00, 0x80, 0x03, 0x60, 0x00, 0x00, 0x06, 0x30, 0x00, 0x00, 0x0C, 
  0x18, 0x00, 0x00, 0x18, 0x0C, 0x0C, 0x30, 0x30, 0x04, 0x38, 0x1C, 0x30, 
  0x06, 0x30, 0x0C, 0x60, 0x02, 0x00, 0x00, 0x60, 0x02, 0x00, 0x00, 0x40, 
  0x03, 0x04, 0x20, 0xC0, 0x03, 0x0E, 0x70, 0xC0, 0x01, 0x1B, 0xD8, 0xC0, 
  0x01, 0x0E, 0x70, 0x80, 0x01, 0x04, 0x20, 0x80, 0x03, 0x00, 0x00, 0xC0, 
  0x03, 0x00, 0x00, 0xC0, 0x03, 0x80, 0x01, 0xC0, 0x02, 0xF0, 0x0F, 0x40, 
  0x02, 0x1C, 0x38, 0x60, 0x06, 0x06, 0x60, 0x60, 0x04, 0x02, 0x40, 0x30, 
  0x0C, 0x00, 0x00, 0x30, 0x18, 0x00, 0x00, 0x18, 0x30, 0x00, 0x00, 0x0C, 
  0x60, 0x00, 0x00, 0x06, 0xC0, 0x00, 0x80, 0x03, 0x80, 0x03, 0xE0, 0x01, 
  0x00, 0x3E, 0x7E, 0x00, 0x00, 0xF0, 0x0F, 0x00, };

  static unsigned char Smileface5[] = {                                     //36x36
  0x00, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x01, 0x00, 0x00, 0x1E, 
  0x80, 0x07, 0x00, 0x80, 0x03, 0x00, 0x1C, 0x00, 0xC0, 0x01, 0x00, 0x38, 
  0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0x30, 0x00, 0x00, 0xC0, 0x00, 0x18, 
  0x00, 0x00, 0x80, 0x01, 0x18, 0x18, 0x80, 0x81, 0x01, 0x0C, 0x70, 0xE0, 
  0x00, 0x03, 0x04, 0xC0, 0x30, 0x00, 0x02, 0x06, 0x00, 0x00, 0x00, 0x06, 
  0x06, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0x00, 0x04, 0x03, 0x1C, 
  0x80, 0x03, 0x0C, 0x03, 0x3C, 0xC0, 0x03, 0x0C, 0x03, 0x26, 0x40, 0x06, 
  0x0C, 0x03, 0x3C, 0xC0, 0x03, 0x0C, 0x03, 0x18, 0x80, 0x01, 0x0C, 0x03, 
  0x00, 0x00, 0x00, 0x0C, 0x03, 0x00, 0x00, 0x00, 0x0C, 0x03, 0x00, 0x00, 
  0x00, 0x0C, 0x02, 0xC0, 0x1F, 0x00, 0x04, 0x06, 0xF0, 0xFF, 0x00, 0x06, 
  0x06, 0x38, 0xC0, 0x01, 0x06, 0x04, 0x0C, 0x00, 0x03, 0x02, 0x0C, 0x00, 
  0x00, 0x00, 0x03, 0x18, 0x00, 0x00, 0x80, 0x01, 0x18, 0x00, 0x00, 0x80, 
  0x01, 0x30, 0x00, 0x00, 0xC0, 0x00, 0x60, 0x00, 0x00, 0x60, 0x00, 0xC0, 
  0x01, 0x00, 0x38, 0x00, 0x80, 0x03, 0x00, 0x1C, 0x00, 0x00, 0x1E, 0x80, 
  0x07, 0x00, 0x00, 0xF8, 0xFF, 0x01, 0x00, 0x00, 0xC0, 0x3F, 0x00, 0x00, 
  };

  static unsigned char Smileface21[] = {                                     //32x32
  0x00, 0xf0, 0x0f, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00,
  0xc0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08,
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, 0x20,
  0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x0e, 0x70, 0x40,
  0x01, 0x0e, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x80, 0x01, 0x00, 0x00, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0xff, 0xff, 0x40,
  0x02, 0x01, 0x80, 0x40, 0x02, 0x02, 0x40, 0x40, 0x04, 0x04, 0x20, 0x20,
  0x08, 0x08, 0x10, 0x10, 0x08, 0x10, 0x08, 0x10, 0x10, 0xe0, 0x07, 0x08,
  0x20, 0x00, 0x00, 0x04, 0xc0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00,
  0x00, 0x0e, 0x70, 0x00, 0x00, 0xf0, 0x0f, 0x00 };


  static unsigned char Smileface22[] = {                                     //32x32
  0x00, 0xf0, 0x0f, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00,
  0xc0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08,
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, 0x20,
  0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40,
  0x01, 0x0e, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x40,
  0x02, 0x02, 0x40, 0x40, 0x02, 0x0c, 0x30, 0x40, 0x04, 0x30, 0x0c, 0x20,
  0x08, 0xc0, 0x03, 0x10, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08,
  0x20, 0x00, 0x00, 0x04, 0xc0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00,
  0x00, 0x0e, 0x70, 0x00, 0x00, 0xf0, 0x0f, 0x00 };

  static unsigned char Smileface23[] = {                                     //32x32
  0x00, 0xf0, 0x0f, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00,
  0xc0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08,
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, 0x20,
  0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40,
  0x01, 0x0e, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x80, 0x01, 0x0e, 0x70, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x40,
  0x02, 0x00, 0x00, 0x40, 0x02, 0xfc, 0x3f, 0x40, 0x04, 0x00, 0x00, 0x20,
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08,
  0x20, 0x00, 0x00, 0x04, 0xc0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00,
  0x00, 0x0e, 0x70, 0x00, 0x00, 0xf0, 0x0f, 0x00 };

  static unsigned char Smileface24[] = {                                     //32x32
  0x00, 0xF0, 0x0F, 0x00, 0x00, 0x0E, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00, 
  0xC0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08, 
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, 0x20, 
  0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 
  0x01, 0x0E, 0x70, 0x80, 0x01, 0x0A, 0x50, 0x80, 0x01, 0x0E, 0x70, 0x80, 
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0xC0, 0x03, 0x40, 
  0x02, 0x30, 0x0C, 0x40, 0x02, 0x0C, 0x30, 0x40, 0x04, 0x02, 0x40, 0x20, 
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08, 
  0x20, 0x00, 0x00, 0x04, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 
  0x00, 0x0E, 0x70, 0x00, 0x00, 0xF0, 0x0F, 0x00, };

  static unsigned char Smileface25[] = {                                     //32x32
  0x00, 0xF0, 0x0F, 0x00, 0x00, 0x0E, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00, 
  0xC0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08, 
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x0C, 0x30, 0x20, 
  0x02, 0x18, 0x18, 0x40, 0x02, 0x30, 0x0C, 0x40, 0x02, 0x60, 0x06, 0x40, 
  0x01, 0x00, 0x00, 0x80, 0x01, 0x0E, 0x70, 0x80, 0x01, 0x0A, 0x50, 0x80, 
  0x01, 0x0E, 0x70, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0xC0, 0x03, 0x40, 
  0x02, 0x30, 0x0C, 0x40, 0x02, 0x0C, 0x30, 0x40, 0x04, 0x02, 0x40, 0x20, 
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08, 
  0x20, 0x00, 0x00, 0x04, 0xC0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00, 
  0x00, 0x0E, 0x70, 0x00, 0x00, 0xF0, 0x0F, 0x00, };

  static unsigned char Smileface26[] = {                                     //32x32
  0x00, 0xf0, 0x0f, 0x00, 0x00, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x80, 0x00,
  0xc0, 0x00, 0x00, 0x03, 0x20, 0x00, 0x00, 0x04, 0x10, 0x00, 0x00, 0x08,
  0x08, 0x00, 0x00, 0x10, 0x08, 0x00, 0x00, 0x10, 0x04, 0x00, 0x00, 0x20,
  0x02, 0x00, 0x00, 0x40, 0x02, 0x00, 0x00, 0x40, 0x02, 0x11, 0x88, 0x40,
  0x01, 0x0a, 0x50, 0x80, 0x01, 0x04, 0x20, 0x80, 0x01, 0x0a, 0x50, 0x80,
  0x01, 0x11, 0x88, 0x80, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
  0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0x40,
  0x02, 0xfe, 0x7f, 0x40, 0x02, 0x00, 0x44, 0x40, 0x04, 0x00, 0x44, 0x20,
  0x08, 0x00, 0x7c, 0x10, 0x08, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x08,
  0x20, 0x00, 0x00, 0x04, 0xc0, 0x00, 0x00, 0x03, 0x00, 0x01, 0x80, 0x00,
  0x00, 0x0e, 0x70, 0x00, 0x00, 0xf0, 0x0f, 0x00 };

