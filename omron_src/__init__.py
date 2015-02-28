# This is the set of functions required to interface over the i2c bus to the Omron DT6 series thermal sensors.

# First, initialze the sensor like this:
"""
(omron1_handle, omron1_result) = omron_init(RASPI_I2C_CHANNEL, OMRON_1, pi, i2c_bus) # passing in the i2c address of the sensor
if omron1_handle < 1:
   print 'I2C sensor not found!'
   logfile.write('\r\nI2C sensor not found! at '+str(datetime.now()))
   sys.exit(0);
"""
# Then, read the temperature array like this:
"""
(bytes_read, temperature_array, room_temp) = omron_read(omron1_handle, DEGREE_UNIT, OMRON_BUFFER_LENGTH, pi)
if bytes_read != OMRON_BUFFER_LENGTH: # sensor problem
   print ''
   print 'ERROR: Omron thermal sensor failure! Bytes read: '+str(bytes_read)
   print ''
"""
# Also see code in git:griffegg/raspbot.py
# make sure that you are using a level translator if the I2C bus is pulled up to anything other than 5v.
# The Adafruit 4 bit level converter adafru.it/757 works great.

import pigpio
import time

MAX_RETRIES = 5
FORCE_INIT_ERROR = 0    # for debugging the code
FORCE_BYTE_COUNT_ERROR = 0    # for debugging the code
FORCE_CRC_ERROR = 0 # for debugging the code
DEBUG = 0 # for debugging the code


# function for celcius to farenheiht conversion
def c2f (c):
   return 9.0*c/5.0 + 32

# function to initialize the Omron sensor
def omron_init(raspi_channel, omron_i2c_address, pigpio_socket, i2c_bus_address):

   # initialize the device based on Omron's appnote 1   
   retries = 0
   result = 0

   for i in range(0,MAX_RETRIES):
      time.sleep(0.05)				# Wait a short time
      handle = pigpio_socket.i2c_open(raspi_channel, omron_i2c_address) # open Omron D6T device at address 0x0a on bus 1
      if FORCE_INIT_ERROR:
         handle = -1

      if handle > 0:
         if DEBUG:
            print ''
            print 'i2c open success: handle='+str(handle)
         result=i2c_bus_address.write_byte(omron_i2c_address,0x4c);
         if DEBUG:
            print 'result='+str(result)
         break
      else:
         print ''
         print '***** Omron init error ***** handle='+str(handle)+' retries='+str(retries)
         retries += 1

   return handle, result

# function to read the omron temperature array
def omron_read(handle, degree_unit, bytes_expected, pigpio_socket):
	
   import crcmod.predefined
	
   OMRON_BUFFER_LENGTH=35				# Omron data buffer size
   OMRON_BUFFER_LENGTH_MINUS_1=34			# without the PEC
   OMRON_DATA_LIST = 16					# actual temperature array
   CRC_ERROR_BIT = 0x04 # the third bit
   CRC_CONSTANT = 0xa4	# if you take the CRC of the whole word including the PEC, this is what you get

   temperature_data_raw=[0]*OMRON_BUFFER_LENGTH
   temperature=[0.0]*OMRON_DATA_LIST		# holds the recently measured temperature
   values=[0]*OMRON_BUFFER_LENGTH
   room_temp = 0

   # read the temperature data stream - if errors, retry
   retries = 0
   for i in range(0,MAX_RETRIES):
      time.sleep(0.05)				# Wait a short time
      (bytes_read, temperature_data_raw) = pigpio_socket.i2c_read_device(handle, bytes_expected)

# test code for retries
      if FORCE_BYTE_COUNT_ERROR:
         print 'Forcing a bytes_read error to test retries'
         bytes_read = -1

      if DEBUG:
         print 'i2c bytes read: '+str(bytes_read)

# Handle i2c error transmissions
      if bytes_read != bytes_expected:
         print ''
         print '***** Omron Byte Count error ***** - bytes read: '+str(bytes_read)

         retries += 1                # start counting the number of times to retry the transmission

# Display the raw data 
         if DEBUG:
            print 'retries = '+str(retries)
            print ''

      if bytes_read == bytes_expected:
# good byte count, now check PEC
         
# Display the raw data 
         if DEBUG:
            print 'Bytes read from Omron D6T(2): '+str(bytes_read)
# Display each element's temperature in F
            print ''
            print 'PTAT: '+'%02x'%temperature_data_raw[1]+'%02x'%temperature_data_raw[0]
            print ''
            print 'Data: '
            print '%02x'%temperature_data_raw[9]+'%02x'%temperature_data_raw[8]+' ',
            print '%02x'%temperature_data_raw[7]+'%02x'%temperature_data_raw[6]+' ',
            print '%02x'%temperature_data_raw[5]+'%02x'%temperature_data_raw[4]+' ',
            print '%02x'%temperature_data_raw[3]+'%02x'%temperature_data_raw[2]+' ',
            print ''
            print '%02x'%temperature_data_raw[17]+'%02x'%temperature_data_raw[16]+' ',
            print '%02x'%temperature_data_raw[15]+'%02x'%temperature_data_raw[14]+' ',
            print '%02x'%temperature_data_raw[13]+'%02x'%temperature_data_raw[12]+' ',
            print '%02x'%temperature_data_raw[11]+'%02x'%temperature_data_raw[10]+' ',
            print ''
            print '%02x'%temperature_data_raw[25]+'%02x'%temperature_data_raw[24]+' ',
            print '%02x'%temperature_data_raw[23]+'%02x'%temperature_data_raw[22]+' ',
            print '%02x'%temperature_data_raw[21]+'%02x'%temperature_data_raw[20]+' ',
            print '%02x'%temperature_data_raw[19]+'%02x'%temperature_data_raw[18]+' ',
            print ''
            print '%02x'%temperature_data_raw[33]+'%02x'%temperature_data_raw[32]+' ',
            print '%02x'%temperature_data_raw[31]+'%02x'%temperature_data_raw[30]+' ',
            print '%02x'%temperature_data_raw[29]+'%02x'%temperature_data_raw[28]+' ',
            print '%02x'%temperature_data_raw[27]+'%02x'%temperature_data_raw[26]+' ',
            print ''
            print 'PEC: '+hex(temperature_data_raw[34])

         t = (temperature_data_raw[1] << 8) | temperature_data_raw[0]
         tPATc = float(t)/10
         if (degree_unit == 'F'):
            room_temp = c2f(tPATc)

# orient the temperature array so that upper left [0] temp is upper left when facing the sensor
         temperature[0] = c2f(float((temperature_data_raw[9] << 8) | temperature_data_raw[8])/10)
         temperature[1] = c2f(float((temperature_data_raw[7] << 8) | temperature_data_raw[6])/10)
         temperature[2] = c2f(float((temperature_data_raw[5] << 8) | temperature_data_raw[4])/10)
         temperature[3] = c2f(float((temperature_data_raw[3] << 8) | temperature_data_raw[2])/10)

         temperature[4] = c2f(float((temperature_data_raw[17] << 8) | temperature_data_raw[16])/10)
         temperature[5] = c2f(float((temperature_data_raw[15] << 8) | temperature_data_raw[14])/10)
         temperature[6] = c2f(float((temperature_data_raw[13] << 8) | temperature_data_raw[12])/10)
         temperature[7] = c2f(float((temperature_data_raw[11] << 8) | temperature_data_raw[10])/10)

         temperature[8] = c2f(float((temperature_data_raw[25] << 8) | temperature_data_raw[24])/10)
         temperature[9] = c2f(float((temperature_data_raw[23] << 8) | temperature_data_raw[22])/10)
         temperature[10] = c2f(float((temperature_data_raw[21] << 8) | temperature_data_raw[20])/10)
         temperature[11] = c2f(float((temperature_data_raw[19] << 8) | temperature_data_raw[18])/10)

         temperature[12] = c2f(float((temperature_data_raw[33] << 8) | temperature_data_raw[32])/10)
         temperature[13] = c2f(float((temperature_data_raw[31] << 8) | temperature_data_raw[30])/10)
         temperature[14] = c2f(float((temperature_data_raw[29] << 8) | temperature_data_raw[28])/10)
         temperature[15] = c2f(float((temperature_data_raw[27] << 8) | temperature_data_raw[26])/10)

# Calculate the CRC error check code
# PEC (packet error code) byte is appended at the end of each transaction. The byte is calculated as CRC-8 checksum, calculated over the entire message including the address and read/write bit. The polynomial used is x8+x2+x+1 (the CRC-8-ATM HEC algorithm, initialized to zero)

         crc8_func = crcmod.predefined.mkCrcFun('crc-8')
         crc = 0
         string = ''

# include the PEC in the list and calculate the CRC for the data stream
         if FORCE_CRC_ERROR:
            print ''
            print 'Forcing a CRC error in bit 3 of byte 6'
            temperature_data_raw[5] = temperature_data_raw[5] ^ CRC_ERROR_BIT

         for i in range(0,bytes_read):
            values[i] = temperature_data_raw[i]
            if DEBUG:
               print '%02x'%values[i],

         string = "".join(chr(i) for i in values)
         crc = crc8_func(string)

         if DEBUG:
            print ''
            print 'String: '+string
            print 'CRC: '+hex(crc)
            print ''

         if crc != CRC_CONSTANT:
            print '***** Omron CRC error ***** Expected '+'%02x'%CRC_CONSTANT+' Calculated: '+'%02x'%crc
         
            retries += 1                # start counting the number of times to retry the transmission
            if DEBUG:
               print 'retries = '+str(retries)
               print ''

            bytes_read = 0		# error is passed up using bytes read
         else:
            break	# crc is good and bytes_read is good

# the end
   return bytes_read, temperature, room_temp


