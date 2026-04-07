
#!/usr/bin/env python

import logging
import serial

from .ntrip_base import NTRIPBase

class NTRIPSerialDevice(NTRIPBase):

  def __init__(self, port, baudrate, logerr=logging.error, logwarn=logging.warning, loginfo=logging.info, logdebug=logging.debug):
    # Call the parent constructor
    super().__init__(logerr, logwarn, loginfo, logdebug)

    # Save the connection info
    self._port = port
    self._baudrate = baudrate

    # Initialize this so we don't throw an exception when closing
    self._device = None

  def connect(self):
    # Attempt to open the serial port
    try:
      self._device = serial.Serial(self._port, self._baudrate)
    except Exception as e:
      self._logerr('Unable to open serial port {} at baudrate {}'.format(self._port, self._baudrate))
      self._logerr('Exception: {}'.format(str(e)))
      return False
    
    # Right now, we can't check anything else, so assuming that the port is open, we succeeded.
    self._loginfo('Connected to serial port {} at baudrate {}'.format(self._port, self._baudrate))
    self._connected = True
    return True

  def disconnect(self):
    # Disconnect the serial port
    self._connected = False
    try:
      if self._device:
        self._device.close()
    except Exception as e:
      self._logdebug('Encountered exception when closing the serial port. This can likely be ignored.')
      self._logdebug('Exception: {}'.format(str(e)))

  def send_nmea(self, sentence):
    if not self._connected:
      self._logwarn('NMEA sent before port was connected, discarding NMEA')
      return

    # Not sure if this is the right thing to do, but python will escape the return characters at the end of the string, so do this manually
    if sentence[-4:] == '\\r\\n':
      sentence = sentence[:-4] + '\r\n'
    elif sentence[-2:] != '\r\n':
      sentence = sentence + '\r\n'

    # Check if it is a valid NMEA sentence
    if not self.nmea_parser.is_valid_sentence(sentence):
      self._logwarn("Invalid NMEA sentence, not sending to server")
      return
    
    # Encode the data and send it to the device
    try:
      self._device.write(sentence.encode('utf-8'))
    except Exception as e:
      self._logerr('Unable to send NMEA sentence to device, reconnecting...')
      self.reconnect()
      self.send_nmea(sentence)

  def recv_rtcm(self):
    if not self._connected:
      self._logwarn('RTCM requested before device was connected, returning empty list')
      return []
    
    # Check how much data is available on the device
    if self._device.in_waiting:
      try:
        data = self._device.read_all()
        self._logdebug('Read {} bytes'.format(len(data)))
        return self.rtcm_parser.parse(data) if data else []
      except Exception as e:
        self._logerr('Unable to read RTCM from device, reconnecting...')
        self.reconnect()
        return []
    else:
      return []