import DOGZILLALib as dog
import time
control = dog.DOGZILLA()
while True:
    angle = control.read_motor()
    
    print(angle)
    time.sleep(0.1)
