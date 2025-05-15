#define LPP_DIGITAL_INPUT	0	 // 1 byte
#define LPP_DIGITAL_OUTPUT	1	 // 1 byte
#define LPP_ANALOG_INPUT	2	 // 2 bytes, 0.01 signed
#define LPP_ANALOG_OUTPUT	3	 // 2 bytes, 0.01 signed
#define LPP_GENERIC_SENSOR	100	 // 4 bytes, unsigned
#define LPP_LUMINOSITY	101	     // 2 bytes, 1 lux unsigned
#define LPP_PRESENCE	102	     // 1 byte, bool
#define LPP_TEMPERATURE	103	     // 2 bytes, 0.1°C signed
#define LPP_RELATIVE_HUMIDITY 104 // 1 byte, 0.5% unsigned
#define LPP_ACCELEROMETER	113   // 2 bytes per axis, 0.001G
#define LPP_BAROMETRIC_PRESSURE	115 // 2 bytes 0.1hPa unsigned
#define LPP_VOLTAGE	116	 // 2 bytes 0.01V unsigned
#define LPP_CURRENT	117	 // 2 bytes 0.001A unsigned
#define LPP_FREQUENCY	118	 // 4 bytes 1Hz unsigned
#define LPP_PERCENTAGE	120	 // 1 byte 1-100% unsigned
#define LPP_ALTITUDE	121	 // 2 byte 1m signed
#define LPP_CONCENTRATION	125	// 2 bytes, 1 ppm unsigned
#define LPP_POWER	128	 // 2 byte, 1W, unsigned
#define LPP_DISTANCE	130	 // 4 byte, 0.001m, unsigned
#define LPP_ENERGY	131	 // 4 byte, 0.001kWh, unsigned
#define LPP_DIRECTION	132	 // 2 bytes, 1deg, unsigned
#define LPP_UNIXTIME	133	 // 4 bytes, unsigned
#define LPP_GYROMETER	134	 // 2 bytes per axis, 0.01 °/s
#define LPP_COLOUR	135	 // 1 byte per RGB Color
#define LPP_GPS	136	 // 3 byte lon/lat 0.0001 °, 3 bytes alt 0.01 meter
#define LPP_SWITCH	142	 // 1 byte, 0/1
#define LPP_POLYLINE	240	 // 1 byte size, 1 byte delta factor, 3 byte lon/lat 0.0001° * factor, n (size-8) bytes deltas