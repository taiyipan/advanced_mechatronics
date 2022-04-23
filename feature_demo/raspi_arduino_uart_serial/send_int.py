from time import sleep
import random
from pySerialTransfer import pySerialTransfer as txfer




if __name__ == '__main__':
    try:
        link = txfer.SerialTransfer('/dev/ttyACM0')

        link.open()
        sleep(5)

        while True:
            num = random.randint(0, 9)
            print(num)
            sendSize = link.tx_obj(num)
            link.send(sendSize)
            sleep(1)

    except KeyboardInterrupt:
        link.close()
