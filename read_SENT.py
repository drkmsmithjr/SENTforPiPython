#!/usr/bin/env python

# read_PWM.py
# 2015-12-08
# Public Domain

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

class SENTReader:
    """
    A class to read short Format SENT frames
          (see the LX3302A datasheet for a SENT reference from Microchip)
          (also using sent transmission mode where )
          from wikiPedia: The SAE J2716 SENT (Single Edge Nibble Transmission) protocol
               is a point-to-point scheme for transmitting signal values
               from a sensor to a controller. It is intended to allow for
               transmission of high resolution data with a low system cost.

    Short sensor format:
      The first is the SYNC pulse (56 ticks)
      first Nibble : Status (4 bits)
      2nd NIbble   : DAta1 (4 bits)
      3nd Nibble   : Data2 (4 bits)
      4th Nibble   : Data3 (4 bits)
      5th Nibble   : Data1 (4 bits)
      6th Nibble   : Data2 (4 bits)
      7th Nibble   : Data3 (4 bits)
      8th Nibble   : CRC   (4 bits)
      """
    def __init__(self, pi, gpio, Mode = 0):
        """
        Instantiate with the Pi and gpio of the SENT signal
        to monitor.
        SENT mode = A0: Microchip LX3302A where the two 12 bit data values are identical.  there are other modes

        """
        self.pi = pi
        self.gpio = gpio
        self.SENTMode = Mode

        # the time that pulse goes high
        self._high_tick = 0
        # the period of the low tick
        self._low_tick = 0
        # the period of the pulse (total data)
        self._period = 0
        # the time the item was low during the period
        self._low = 0
        # the time the output was high during the period
        self._high = 0
        self.syncTick = 0

        #keep track of the periods
        self.syncWidth = 0
        self.status = 0
        self.data1 = 0
        self.data2 = 0
        self.data3 = 0
        self.data4 = 0
        self.data5 = 0
        self.data6 = 0
        self.crc = 0

        #initize the sent frame .  Need to use hex for data
        self.frame = [0,0,0,'0x0','0x0','0x0','0x0','0x0','0x0',0]

        self.syncFound = False
        self.frameComplete = False
        self.nibble = 0

        pi.set_mode(gpio, pigpio.INPUT)

        self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

    def _cbf(self, gpio, level, tick):
        # depending on the system state set the tick times.
        # first look for sync pulse. this is found when duty ratio >90
        #print(pgio)
        if self.syncFound == False:
            if level == 1:
                self._high_tick = tick
                self._low = pigpio.tickDiff(self._low_tick,tick)
            elif level == 0:
                # this may be a syncpulse if the duty is 51/56
                self._period = pigpio.tickDiff(self._low_tick,tick)
                # not reset the self._low_tick
                self._low_tick = tick
                self._high = pigpio.tickDiff(self._high_tick,tick)
                if 100*self._high/self._period > 90:
                    self.syncFound = True
                    self.syncWidth = self._high
                    self.syncPeriod = self._period
                    self.syncTick = round(self.syncPeriod/56.0,2)
                    # reset the nibble to zero
                    self.nibble = 0
        else:
            # now look for the nibble information for each nibble (8 Nibbles)
            if level == 1:
                self._high_tick = tick
                self._low = pigpio.tickDiff(self._low_tick,tick)
            elif level == 0:
                # This will be a data nibble
                self._period = pigpio.tickDiff(self._low_tick,tick)
                # not reset the self._low_tick
                self._low_tick = tick
                self._high = pigpio.tickDiff(self._high_tick,tick)
                self.nibble = self.nibble + 1
                if self.nibble == 1:
                    self.status = hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 2:
                    self.data1 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 3:
                    self.data2 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 4:
                    self.data3 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 5:
                    self.data4 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 6:
                    self.data5 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 7:
                    self.data6 =  hex(int(round(self._period / self.syncTick)-12))
                if self.nibble == 8:
                    self.crc =    hex(int(round(self._period / self.syncTick)-12))
                    # now send all to the SENT Frame
                    self.frame = [self.syncPeriod,self.syncTick,self.status,self.data1,self.data2,self.data3,self.data4,self.data5,self.data6,self.crc]
                    self.syncFound = False


    def SENTData(self):
        # check that data1 = Data2 if they are not equal return fault = True
        fault = False
        SentFrame = self.frame[:]
        #gather first 12 bits of data using nibble 1,2,3
        datanibble = '0x'
        datanibble2 = '0x'
        for x in range (3,6):
            datanibble = datanibble + str((self.frame[x]))[2:]
            #datanibble = datanibble + str((self.frame[x]))
        for x in range (6,9):
            datanibble2 = datanibble2 + str((self.frame[x]))[2:]
            #datanibble2 = datanibble2 + str((self.frame[x]))
        # if using SENT mode 0, then data nibbles should be equal
        if self.SENTMode == 0 :
            if datanibble != datanibble2:
                fault = True
        # also need section to check the CRC code in this format

        # converting the datanibble values to a binary bit string. 
        # remove the first two characters.  Not needed for crcCheck        
        InputBitString = bin(int((datanibble + datanibble2),16))[2:]
        # converting Crcvalue to bin
        crcBitValue = bin(int(self.crc))
        #checking the crcValue
        if self.crcCheck(InputBitString,'0101',crcBitValue) == False:
            fault = True
            

        # converter to decimnal
        returnData = int(datanibble,16)
        returnData2 = int(datanibble2,16)
        #returns both Data values and if there is a FAULT
        return (returnData, returnData2, fault)



    def tick(self):
        return self.syncTick

    def crcNibble(self):
        return self.crc

    def statusNibble(self):
        return self.status

    def syncPulse(self):
        return self.syncWidth

    def syncNibble(self):
        return self.syncPeriod

    def syncFrame(self):
            return self.frame

    def cancel(self):
        self._cb.cancel()
        
    def crcCheck(self, InputBitString, PolyBitStream, PadValue ):
        # the input string will be a binary string all 6 nibbles of the SENT data
        # the seed value (padValue) is appended to the input string.  Do not use zeros for SENT protocal 
        # this uses the SENT CRC recommended implementation.  
        checkOK = False
        
        LenPolyBitStream = len(PolyBitStream)
        PolyBitStream = PolyBitStream.lstrip('0')
        #print(PolyBitStream)
        LenInput = len(InputBitString)
        InputPaddedArray = list(InputBitString + PadValue)
        while '1' in InputPaddedArray[:LenInput]:
            cur_shift = InputPaddedArray.index('1')
            for i in range(len(PolyBitStream)):
            InputPaddedArray[cur_shift + i] = str(int(PolyBitStream[i] != InputPaddedArray[cur_shift + i]))
        
        if (InputPaddedArray[LenInput:] == crcValue):
            checkOK = True
        print(InputPaddedArray[LenInput:])
        return checkOK

if __name__ == "__main__":

    import time
    import pigpio
    import read_SENT

    SENT_GPIO = 18
    RUN_TIME = 60.0
    SAMPLE_TIME = 0.1

    pi = pigpio.pi()

    p = read_SENT.SENTReader(pi, SENT_GPIO)

    start = time.time()

    while (time.time() - start) < RUN_TIME:

        time.sleep(SAMPLE_TIME)

        crc = p.crcNibble()
        pw = p.syncPulse()
        tick = p.tick()
        status = p.statusNibble()

        z = p.SENTData()
        print("Sent 12-bit DATA 1= %s - DATA 2= %s - tickTime(uS)= %s - CRC= %s - Errors= %s" % (z[0],z[1],tick,crc,z[2]))


    p.cancel()

    pi.stop()
