### System Architecture

```text
BMP180 ──(I2C)──┐
                ├──► ESP32 ──► Acquisition ──► Validation ──► State ──► Actuators
Potentiometer ──┘                                                    ├──► Motor
                                                                     └──► LED
