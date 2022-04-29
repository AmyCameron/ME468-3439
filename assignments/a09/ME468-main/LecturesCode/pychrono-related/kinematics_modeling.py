import pychrono as chrono
import math
import numpy

def printMat(str, A) :
    npmat = numpy.asarray(A.GetMatr())
    numpy.set_printoptions(suppress=True)
    print(str, '\n', npmat)

# 3D vectors

v1_s = chrono.ChVectorF(1, 2, 3)  # create vector with given (x, y, z) single precision components
v1 = chrono.ChVectorD(1, 2, 3)    # create vector with given (x, y, z) double precision components
v2 = chrono.ChVectorD(4, 1, 2)    # 
v3 = chrono.ChVectorD             # create default vector (0, 0, 0)
v4 = chrono.ChVectorD(v1 + v2)    # create vector by copying from another one (result of '+' here)

v3 = v1 + v2                      # vector operations: +, -
v3 += v1                          # in-place operations
v3 = v2 * 0.03                    # multiplication by scalar (scalar to the right only)
v3 = v1 * v2                      # component-wise vector product (not common!)
v3 = v1 % v2                      # cross product operator
v3.Normalize()                    # vector functions
a1 = v1 ^ v2                      # inner product (dot product)
a3 = v1.Dot(v2)                   #   another way to do dot product
a2 = chrono.ChVectorD.Dot(v1, v2) #   yet another way to do dot product

print(' v1 = ', v1)
print(' v2 = ', v2)
print(' v3 = ', v3)
print(' v4 = ', v4)
print(' dot = ', a1, '  ', a2, '  ', a3)

# Quaternions

q1_s = chrono.ChQuaternionD(1,2,3,4)    # create quaternion with (w, x, y, z) single precision components
q1 = chrono.ChQuaternionD(1,2,3,4)      # create quaternion with (w, x, y, z) double precision components
angle = math.pi / 3
axis = chrono.ChVectorD(1, 0, 0)
q2 = chrono.Q_from_AngAxis(angle, axis) # create (unit) quaternion from angle and axis (Euler)
q2 = chrono.Q_from_AngX(angle)          # create a quaternion to represent a rotation about X axis

q1.Normalize()                          # normalize quaternion -> unit quaternion
qc2 = ~q2                               # conjugate quaternion
q3 = q1 * q2                            # quaternion product
q3 = q1 % q2                            #   another operator for quaternion product
a = q1 ^ q2                             # quaternion dot product

v = chrono.ChVectorD(1, 2, 3)
vr = q2.Rotate(v)                       # rotate a vector on the basis of this quaternion
vrr1 = q2.RotateBack(vr)                # rotate a vector on the basis of the conjughate of this quaternion
vrr2 = qc2.Rotate(vr)                   # check back rotation

print('q1  = ', q1)
print('q2  = ', q2)
print('qc2 = ', qc2)
print('q3  = ', q3)
print('dot  = ', a)

print('v   = ', v)
print('vr  = ', vr)
print('vrr1 = ', vrr1)
print('vrr2 = ', vrr2)

# Rotation matrices

R1 = chrono.ChMatrix33D()                                         # default matrix is uninitialized
R2 = chrono.ChMatrix33D(math.pi/3, chrono.ChVectorD(1, 0, 0))     # rotation matrix from Euler axis and angle
f = chrono.ChVectorD(0,1,0)                                       # x-axis of a rotated frame
g = chrono.ChVectorD(-1,0,0)                                      # y-axis of a rotated frame
h = chrono.ChVectorD(0,0,1)                                       # z-axis of a rotated frame
R3 = chrono.ChMatrix33D(f, g, h)                                  # rotation matrix from axis unit vectors
euler_angles = chrono.ChVectorD(math.pi/2, math.pi/2, math.pi/2)  # Euler angles (yaw, pitch, roll)
q_nasa = chrono.Q_from_NasaAngles(euler_angles)                   #   corresponding unit quaternion
R4 = chrono.ChMatrix33D(q_nasa)                                   #   corresponding rotation matrix
R1.Set_A_Hpb(euler_angles)

f_ = R3.Get_A_Xaxis()                                             # get the unit vector along X-axis
q_nasa_ = R4.Get_A_quaternion()                                   # get the equivalent unit quaternion
#R = R3 * R4                                                      # matrix multiplication NOT exposed
v = R2 * chrono.ChVectorD(0, 1, 0)                                # rotate vector

printMat('R1 = ', R1)
printMat('R2 = ', R2)
printMat('R3 = ', R3)
printMat('R4 = ', R4)
print('f = ', f, 'f_ =', f_)
print('q_nasa = ', q_nasa, 'q_nasa_ = ', q_nasa_)
print('v = ', v)

# Coordinate transformations

r = chrono.ChVectorD(1, 1, 1)          # origin of body frame
q = chrono.Q_from_AngX(math.pi / 6)    # orientation of body frame
R = chrono.ChMatrix33D(q)              # corresponding rotation matrix
s = chrono.ChVectorD(1, 1, 1)          # body-fixed vector (expressed in body frame)
s1 = r + q.Rotate(s)                   # body-fixed vector expressed in global
s2 = r + R * s                         # body-fixed vector expressed in global

qa = chrono.Q_from_AngX(math.pi / 6)   
qb = chrono.Q_from_AngY(math.pi / 4)
qab1 = qa * qb                         # concatenation of two rotation (first qb, then qa) 
qab2 = qb >> qa                        # concatenation of two rotation (first qb, then qa) 
s3 = r + qab1.Rotate(s)                # vector transformation using qab1
s4 = r + qab2.Rotate(s)                # vector transformation using qab2

print('s1 = ', s1)
print('s2 = ', s2)
print('s3 = ', s3)
print('s4 = ', s4)
print('  ')

# ChCoordsysD and ChFrameD

d_ba = chrono.ChVectorD(1, 1, 1)                    # translation vector
q_ba = chrono.Q_from_AngZ(math.pi / 4)              # rotation quaternion
csys0 = chrono.ChCoordsysD                          # default csys (zero translation, no rotation)
csys1 = chrono.ChCoordsysD(d_ba, q_ba)              # csys from translation and rotation
frame0 = chrono.ChFrameD                            # default frame (zero translation, no rotation)
frame1 = chrono.ChFrameD(d_ba, q_ba)                # frame from translation and rotation
frame2 = chrono.ChFrameD(d_ba,                      # frame from translation, angle, rotation axis
                         math.pi / 4, 
                         chrono.ChVectorD(0, 0, 1))

s = chrono.ChVectorD(1, 0, 0)                       # vector in body frame
s1 = csys1.TransformPointLocalToParent(s)           # vector in global frame
s2 = csys1.pos + csys1.rot.Rotate(s)                # vector in global frame
s3 = frame1.TransformPointLocalToParent(s)          # vector in global frame
s4 = frame1 * s                                     # vector in global frame
s5 = frame2 * s                                     # vector in global frame
s6 = s >> frame1                                    # vector in global frame

R1 = chrono.ChMatrix33D(csys1.rot)                  # rotation matrix from csys quaternion
R2 = frame1.GetA()                                  # get rotation matrix from frame

print('s1 = ', s1)
print('s2 = ', s2)
print('s3 = ', s3)
print('s4 = ', s4)
print('s5 = ', s5)
print('s6 = ', s6)
printMat('R1 = ', R1)
printMat('R2 = ', R2)

# Moving frame

X_ba = chrono.ChFrameMovingD()                       # construct default moving frame
                                                  
X_ba.SetPos(chrono.ChVectorD(2, 3, 5))               # set translation
X_ba.SetRot(chrono.Q_from_AngZ(math.pi / 3))         # set rotation quaternion
                                                  
X_ba.SetPos_dt(chrono.ChVectorD(100, 20, 53))        # linear velocity
X_ba.SetWvel_loc(chrono.ChVectorD(0, 40, 0))         # angular velocity in local frame
X_ba.SetWvel_par(chrono.ChVectorD(0, 40, 0))         # angular velocity in parent frame
                                                  
X_ba.SetPos_dtdt(chrono.ChVectorD(13, 16, 22))       # linear acceleration
X_ba.SetWacc_loc(chrono.ChVectorD(80, 50, 0))        # angular acceleration in local frame
X_ba.SetWacc_par(chrono.ChVectorD(80, 50, 0))        # angular acceleration in parent frame

# Concatenation of transforms

X_ba = chrono.ChFrameD()
X_cb = chrono.ChFrameD()
X_ca1 = X_ba * X_cb        # first X_cb and then X_ba
X_ca2 = X_cb >> X_ba       # first X_cb and then X_ba

# Same for moving frames, but velocities and accelerations are also transformed
Xm_ba = chrono.ChFrameMovingD()
Xm_cb = chrono.ChFrameMovingD()
Xm_ca1 = Xm_ba * Xm_cb     # first X_cb and then X_ba
Xm_ca2 = Xm_cb >> Xm_ba    # first X_cb and then X_ba

