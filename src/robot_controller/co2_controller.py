import logging
import math
import sys
import time

import serial

logging.basicConfig(level = logging.INFO)

class co2_handler:
    def __init__(self, COM: str, sim: bool = False) -> None:
        self.sim = sim
        self.radius = 1 #mm

        self.pH_threshold = 6.8
        self.pH_sample_time = 30 #s
        self.pH_sample_size = 10

        if self.sim is False:
            logging.info("Configuring CO2 kit serial port..")
            self.ser = serial.Serial(COM) 
            self.ser.baudrate = 9600
            self.ser.bytesize = 8 
            self.ser.parity = 'N' # No parity
            self.ser.stopbits = 1

            logging.info("Attempting to open CO2 kit serial port..")

            if self.ser.isOpen() is False:
                self.ser.open()

            logging.info("Response from pH probe: " + self.get_data())
            logging.info("Response from pH probe: " + self.get_data())

            if self.get_data() == "CO2 Kit Ready":
                logging.info("Serial connection to CO2 kit established.")
            else:
                logging.error("Failed to establish serial connection to CO2 kit.")
                sys.exit()

        else:
            logging.info("No serial connection to CO2 kit established.")

    def get_data(self) -> str:
        while self.ser.in_waiting == 0:
            pass
        
        return self.ser.readline().decode().rstrip().replace("\x00", "")
        
    def get_response(self) -> None:
        data = self.get_data()
        # Wait for response and check that command was understood
        if data == "Unknown command":
            logging.error("CO2 kit failed to recognise command.")
            sys.exit()
        else:
            logging.info("Response from CO2 kit: " + data)

    def close_ser(self) -> None:
        logging.info("Closing serial connection to CO2 kit.")
        if self.sim is False:
            if self.ser.isOpen():
                self.ser.close()

    def releaseCO2(self, duration: float = 0) -> None:
        logging.info(f"Opening CO2 valve for {duration}mins..")
        
        if self.sim is False:
            self.ser.write(f"releaseCO2({duration*60})".encode())
            self.get_response()

    def refreshWater(self, duration: float = 0) -> None:
        logging.info(f"Sending to water to pH chamber via DC motor 1 ({duration}s)..")
        
        if self.sim is False:
            self.ser.write(f"dcMotor1({duration})".encode())
            self.get_response()

    def sendToWaste(self, duration: float = 0) -> None:
        logging.info(f"Sending to waste from pH chamber via DC motor 2 ({duration}s)..")
        
        if self.sim is False:
            self.ser.write(f"dcMotor2({duration})".encode())
            self.get_response()

    def getTubeVol(self, tube_length: float) -> float:
        # 2mm ID tubing (Area = Pi)
        return math.pi * tube_length * 1e-3 * self.radius**2 #ml
    
    def sendToPH(self, fluid_vol: float = 0, tube_length: float = 0, overpump: float = 1.0) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of chemical to pH chamber..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        if self.sim is False:
            self.ser.write(f"sendToPH({vol})".encode())
            self.get_response()

    def addChemical(self, fluid_vol: float = 0, tube_length: float = 0, overpump: float = 1.0) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of chemical to mixing pot..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        if self.sim is False:
            self.ser.write(f"addChemical({vol})".encode())
            self.get_response()

    def addWater(self, fluid_vol: float = 0, tube_length: float = 0, overpump: float = 1.0) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of water to mixing pot..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        if self.sim is False:
            self.ser.write(f"addWater({vol})".encode())
            self.get_response()

    def transferToECMS(self, fluid_vol: float, tube_length: float = 500.0, overpump: float = 1.2) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of saturated mixture to EC-MS..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        if self.sim is False:
            self.ser.write(f"transferToECMS({vol})".encode())
            self.get_response()

    def getPH(self) -> float:
        if self.sim is False:
            self.ser.write("getPH()".encode())
            return float(self.get_data())
        else:
            return 0

    def checkPH(self) -> bool:
        average = 0
        delay = self.pH_sample_time / self.pH_sample_size
        logging.info(f"Collecting pH data for {self.pH_sample_time}s..")

        for _ in range(self.pH_sample_size):
            average += self.getPH()
            time.sleep(delay)

        average /= self.pH_sample_size
        logging.info(f"Average pH found to be {average}.")

        if average < self.pH_threshold:
            logging.info(f"pH test passed!")
            return True
        else:
            logging.info(f"pH test failed!")
            return False