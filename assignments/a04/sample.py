import pychrono as chrono
from pychrono import irrlicht as chronoirr

my_vect1 = chrono.ChVectorD()
my_vect1.x=5
my_vect1.y=2
my_vect1.z=3
my_vect2 = chrono.ChVectorD(3,4,5)
my_vect4 = my_vect1*10 + my_vect2
print(my_vect1)
print(my_vect2)
print(my_vect4)

