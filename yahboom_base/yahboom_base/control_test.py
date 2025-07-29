import DOGZILLALib as dog

control = dog.DOGZILLA()

control.move('x',1)
a = control.read_motor()
b= control.read_pitch()
control.stop()
print(a)
