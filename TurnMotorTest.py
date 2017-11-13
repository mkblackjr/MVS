from pyduino import *
import time

def get_next_step(current_step):
    
    # Big ugly function brute forcing the steps.  Find a better way!
    if current_step == [1, 0, 0, 1]:
        return [0, 0, 0, 1]
    elif current_step == [0, 0, 0, 1]:
        return [0, 0, 1, 1]
    elif current_step == [0, 0, 1, 1]:
        return [0, 0, 1, 0]
    elif current_step == [0, 0, 1, 0]:
        return [0, 1, 1, 0]
    elif current_step == [0, 1, 1, 0]:
        return [0, 1, 0, 0]
    elif current_step == [0, 1, 0, 0]:
        return [1, 1, 0, 0]
    elif current_step == [1, 1, 0, 0]:
        return [1, 0, 0, 0]
    else: # current_step == [1, 0, 0, 0]
        return [1, 0, 0, 1]





if __name__ == '__main__':
    
    a = Arduino()
    # if your arduino was running on a serial port other than '/dev/ttyACM0/'
    # declare: a = Arduino(serial_port='/dev/ttyXXXX')

    time.sleep(3)
    # sleep to ensure ample time for computer to make serial connection 

    step = [1, 0, 0, 1]
    PIN = [4, 5, 6, 7]
    a.DDRD = 'B11110000'
    # for pin in PIN:
    #     a.set_pin_mode(pin, 'O')
    # # initialize the digital pin as output

    time.sleep(1)
    # allow time to make connection

    for i in range(0,100000):
        
        step = get_next_step(step)
        STEP = ''.join(str(x) for x in step) + '0000'
        print(STEP)
        a.PORTD = 'B'+STEP

        # for x,val in enumerate(PIN):
        #         a.digital_write(PIN[x],STEP[x]) # advance to next step
        #         time.sleep(0.0008)

        time.sleep(0.008)

