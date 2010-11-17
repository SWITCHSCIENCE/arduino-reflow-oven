#include "SPI.h"

boolean SPI_Master_Class::initialized_ = false;
int SPI_Master_Class::enabled_ = -1;

void
SPI_Master_Class::begin(int slaveselecter) {
  if (!initialized_) {
    initialized_ = true;
    enabled_ = -1;
    pinMode(SS, OUTPUT);  // Must be set as OUTPUT before SPE is asserted.
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    digitalWrite(MISO, HIGH);  // Pull-up
    pinMode(SCK, OUTPUT);
    SPCR = (1<<SPE)|(1<<MSTR);  // SPE: SPI Enable; MSTR: Master
    byte garbage;
    garbage = SPSR;
    garbage = SPDR;
  }

  if (slaveselecter != SS)
    pinMode(slaveselecter, OUTPUT);
  digitalWrite(slaveselecter, HIGH);  // Disable
}

void
SPI_Master_Class::enable(int slaveselecter) {
  disable();
  digitalWrite(slaveselecter, LOW);
  enabled_ = slaveselecter;
}

void
SPI_Master_Class::disable() {
  if (enabled_ >= 0) {
    digitalWrite(enabled_, HIGH);
    enabled_ = -1;
  }
}

byte
SPI_Master_Class::write_and_read(byte data) const {
  SPDR = data;
  while (!(SPSR & (1<<SPIF)))
    ;
  return SPDR;
}

void
SPI_Master_Class::write(byte data) const {
  write_and_read(data);
}

byte
SPI_Master_Class::read() const {
  return write_and_read(0x00);
}

SPI_Master_Class SPI_Master;
