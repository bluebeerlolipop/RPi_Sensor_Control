import threading
from IMUmanager import IMUmanager
from RocketProtocol import RocketProtocol
import time

import numpy as np

IMU_ON = True
COMMUNICATION_ON = True
ROCKETPROTOCOL_ON = True


if __name__== "__main__":
    start = time.time()

    mRocketProtocol = RocketProtocol()
    try:
        mIMUmanager= IMUmanager(mRocketProtocol)

        if(IMU_ON):
            IMUthread= threading.Thread(target=mIMUmanager.getData)

            IMUthread.start()

        if(COMMUNICATION_ON):
            mIMUmanager.initConnect()
            COMMUNICATIONthread= threading.Thread(target=mIMUmanager.communicationData)
            COMMUNICATIONthread.start()

        if(ROCKETPROTOCOL_ON):
            while not mRocketProtocol.AlgorithmProcess(mIMUmanager.mSensorDataQueue):
                continue
            
    except KeyboardInterrupt as e:
        mRocketProtocol.Cleanup()
    
