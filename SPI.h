#ifndef __SPI_H__
#define __SPI_H__

#include "WProgram.h"

class SPI_Master_Class {
public:
  static void begin(int slaveselecter);
  void enable(int slaveselecter);
  void disable();
  byte write_and_read(byte data) const;
  void write(byte data) const;
  byte read() const;

private:
  static boolean initialized_;
  static const int SS = 10;
  static const int MOSI = 11;
  static const int MISO = 12;
  static const int SCK = 13;
  static int enabled_;
};

extern SPI_Master_Class SPI_Master;

#endif //__SPI_H__
