import logging
import time
import serial
import math

logging.basicConfig(level = logging.INFO)

def skip_if_sim(default_return = None):
    def decorator(func):
        def wrapper(self, *args, **kwargs):
            if self.sim:
                return default_return
            return func(self, *args, **kwargs)
        return wrapper
    return decorator

class ECMSController:
    def __init__(self, COM: str, baud: int = 115200, sim: bool = False, timeout: float = 60.0) -> None:
        self.sim = sim
        self.timeout = timeout

        self.radius = 1 #mm

        self.pH_threshold = 6.8
        self.pH_sample_time = 20 #s
        self.pH_sample_size = 10

        self.pH_volume = 15 #ml

        self.waste_time = 20 #s
        self.refresh_time = 10 #s

        if self.sim:
            logging.info("Simulated connection to EC-MS controller established.")

        else:
            logging.info("Configuring EC-MS controller serial port..")
            self.ser = serial.Serial(COM) 
            self.ser.baudrate = baud
            self.ser.bytesize = 8 
            self.ser.parity = 'N' # No parity
            self.ser.stopbits = 1
            self.ser.timeout = self.timeout

            # Windows compatibilty (disable flow control)
            self.ser.rtscts=False
            self.ser.dsrdtr=False
            self.ser.xonxoff=False

            logging.info("Attempting to open EC-MS controller serial port..")

            if self.ser.isOpen() is False:
                self.ser.open()

            self.ser.setDTR(False)  # IO0 stays high
            self.ser.setRTS(False)  # EN stays high

            # Check connection (blocking)
            self.check_response()

    @skip_if_sim()
    def reset_esp(self) -> None:
        # Simple reset: pull EN low then high
        self.ser.setRTS(True)
        time.sleep(0.2)
        self.ser.setRTS(False)

    @skip_if_sim(default_return="0")
    def get_data(self) -> str:
        while self.ser.in_waiting == 0:
            pass

        return self.ser.readline().decode().rstrip().replace("\x00", "")
        
    @skip_if_sim()
    def check_response(self) -> None:
        while True:
            data = self.get_data()
            if '#' in data:
                logging.info("Successful response from EC-MS controller: " + data)
                break

            if "Unknown command" in data:
                raise RuntimeError("EC-MS controller failed to recognise command: " + data)

    @skip_if_sim()
    def close_ser(self) -> None:
        logging.info("Closing serial connection to EC-MS controller.")
        if self.ser.isOpen():
            self.ser.close()

    @skip_if_sim(default_return = True)
    def check_status(self) -> bool:
        self.ser.write("statusCheck()".encode())
        self.check_response()

    @skip_if_sim(default_return = 25)
    def get_temperature(self) -> float:
        self.ser.write("getTemperature()".encode())
        data = self.get_data()
        try:
            return float(data) if data is not None else None
        except (TypeError, ValueError):
            return None
        
    @skip_if_sim(default_return = 50)
    def get_humidity(self) -> float:
        self.ser.write("getHumidity()".encode())
        data = self.get_data()
        try:
            return float(data) if data is not None else None
        except (TypeError, ValueError):
            return None

        
    @skip_if_sim()
    def releaseCO2(self, duration: float = 0) -> None:
        logging.info(f"Opening CO2 valve for {duration}mins..")
        
        self.ser.write(f"releaseCO2({duration})".encode())
        self.check_response()

    @skip_if_sim()
    def refreshWater(self) -> None:
        logging.info(f"Sending to water to pH chamber ({self.refresh_time}s)..")
        
        self.ser.write(f"refreshPhWater({self.refresh_time})".encode())
        self.check_response()

    @skip_if_sim()
    def sendToWaste(self) -> None:
        logging.info(f"Sending to waste from pH chamber ({self.waste_time}s)..")

        self.ser.write(f"sendToWaste({self.waste_time})".encode())
        self.check_response()

    @skip_if_sim()
    def getTubeVol(self, tube_length: float) -> float:
        # 2mm ID tubing (Area = Pi)
        return math.pi * tube_length * 1e-3 * self.radius**2 #ml
    
    @skip_if_sim()
    def sendToPH(self, tube_length: float = 0, overpump: float = 1.0) -> None:
        logging.info(f"Pumping {self.pH_volume}ml of chemical to pH chamber..")
        vol = overpump * (self.pH_volume + self.getTubeVol(tube_length)) #ml
        
        self.ser.write(f"sendToPh({vol})".encode())
        self.check_response()

    @skip_if_sim()
    def addChemical(self, fluid_vol: float = 0, tube_length: float = 0, overpump: float = 1.0) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of chemical to mixing pot..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        self.ser.write(f"addChemical({vol})".encode())
        self.check_response()

    @skip_if_sim()
    def addWater(self, fluid_vol: float = 0, tube_length: float = 0, overpump: float = 1.0) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of water to mixing pot..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
 
        self.ser.write(f"addWater({vol})".encode())
        self.check_response()

    @skip_if_sim()
    def transferToECMS(self, fluid_vol: float, tube_length: float = 500.0, overpump: float = 1.2, speed: float = 0.01) -> None:
        # Fluid volume in uL -> sent volume in mL
        logging.info(f"Pumping {fluid_vol}ml of saturated mixture to EC-MS..")
        vol = overpump * (fluid_vol + self.getTubeVol(tube_length)) #ml
        
        self.ser.write(f"transferToECMS({vol},{speed})".encode())
        self.check_response()

    @skip_if_sim(default_return=7.0)
    def getPH(self) -> float:
        self.ser.write("getPH()".encode())
        data = self.get_data()
        try:
            return float(data) if data is not None else None
        except (TypeError, ValueError):
            return None
        
    @skip_if_sim()
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
            logging.info("pH test passed!")
            return True
        else:
            logging.info("pH test failed!")
            return False