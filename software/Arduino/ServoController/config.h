/**********************************************************************
 * Arduino Configuration file - I2C addresses and declination         *
 * Project: Eclipse Tracking (2017)                                   *
 * Version: 3/04/2017                                                 *
 * Max Bowman / Jeremy Seeman                                         *
 **********************************************************************/

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define DECLINATION 3.52 // Declination (degrees) in Chicago, IL.
