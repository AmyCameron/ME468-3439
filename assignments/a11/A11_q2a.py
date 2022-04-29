# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import math
import os
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
chrono.SetChronoDataPath(os.getenv("CHRONO_DATA_DIR"))
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# --------------------
# Save Data
# --------------------

home_dir = os.getenv('HOME')
save_dir = "me468_output"
save_name = "frame"
if(not os.path.exists(os.path.join(home_dir,save_dir))):
    os.mkdir(os.path.join(home_dir,save_dir))

output_dir = os.path.join(home_dir,save_dir, save_name)
if(not os.path.exists(output_dir)):
    os.mkdir(output_dir)


# ---------------------
# Simulation parameters
# ---------------------

# Initial vehicle location and orientation
initLoc = chrono.ChVectorD(0, 0, 0.4)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_MESH
suspension_vis_type = veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_NONE
tire_vis_type = veh.VisualizationType_MESH

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Simulation end time
tend = 30

# view camera images
vis = True
save = False
noise = False

# =============================================================================


print( "Copyright (c) 2017 projectchrono.org\n")

# --------------
# Create systems
# --------------

# Create the vehicle, set parameters, and initialize
gator = veh.Gator()
gator.SetContactMethod(chrono.ChContactMethod_NSC)
gator.SetChassisFixed(False)
gator.SetInitPosition(chrono.ChCoordsysD(initLoc, initRot))
gator.SetBrakeType(veh.BrakeType_SHAFTS)
gator.SetTireType(veh.TireModelType_TMEASY)
gator.SetTireStepSize(tire_step_size)
gator.SetInitFwdVel(0.0)
gator.Initialize()

gator.SetChassisVisualizationType(chassis_vis_type)
gator.SetSuspensionVisualizationType(suspension_vis_type)
gator.SetSteeringVisualizationType(steering_vis_type)
gator.SetWheelVisualizationType(wheel_vis_type)
gator.SetTireVisualizationType(tire_vis_type)

# ------------------
# Create the terrain
# ------------------
terrain = veh.RigidTerrain(gator.GetSystem())
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 0, 1), 
                         600, 600)
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 1200, 1200)
terrain.Initialize()

asset = patch.GetGroundBody().GetAssets()[0]
visual_asset = chrono.CastToChVisualization(asset)

vis_mat = chrono.ChVisualMaterial()
vis_mat.SetKdTexture(chrono.GetChronoDataFile("sensor/textures/mud.png"))
vis_mat.SetRoughness(0.99)

visual_asset.material_list.append(vis_mat)

# -----------------------
# Create a sensor manager
# -----------------------
manager = sens.ChSensorManager(gator.GetSystem())
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)
b = sens.Background()
b.mode = sens.BackgroundMode_ENVIRONMENT_MAP
b.env_tex = chrono.GetChronoDataFile("sensor/textures/quarry_01_4k.hdr")
manager.scene.SetBackground(b)

# ------------------------------------------------
# TO DO: Create a camera and add it to the vehicle
# ------------------------------------------------
# Parameters to use:
# offset_pose: chrono.ChFrameD(chrono.ChVectorD(.1, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))
# update rate: 30
# image size: 1280x720
# field of view: pi/2
# save data using a save filter
cam_update_rate = 30.0
image_width = 1280.0
image_height = 720.0
cam_fov = np.pi/2.0

offset_pose = chrono.ChFrameD(chrono.ChVectorD(.1, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))
cam = sens.ChCameraSensor(gator.GetChassisBody(), cam_update_rate, offset_pose, image_width, image_height, cam_fov)
cam.SetName("Canera Sensor")

if(not os.path.exists(os.path.join(output_dir,"camera/"))):
    os.mkdir(os.path.join(output_dir,"camera/"))
    cam.PushFilter(sens.ChFilterSave(os.path.join(output_dir,"camera1")))
cam.PushFilter(sens.ChFilterRGBA8Access())

# cam.SetLag(cam_lag)
# cam.SetCollectionWindow(cam_collection_time)

cam.PushFilter(sens.ChFilterRGBA8Access())
manager.AddSensor(cam)


# ------------------------------------------------
# TO DO: Create a GPS and add it to the vehicle
# ------------------------------------------------
# Parameters to use:
# offset_pose: chrono.ChFrameD(chrono.ChVectorD(.1, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))
# update rate: 10
# GPS reference point: (-89.400, 43.070, 260.0)
# save data by adding an access filter and getting the data to plot directly or save

gps_update_rate = 10
gps_reference = chrono.ChVectorD(-89.400, 43.070, 260.0)
noise_none = sens.ChNoiseNone()

offset_pose = chrono.ChFrameD(chrono.ChVectorD(.1, 0, 1.45), chrono.Q_from_AngAxis(.2, chrono.ChVectorD(0, 1, 0)))
gps = sens.ChGPSSensor(gator.GetChassisBody(), gps_update_rate, offset_pose, gps_reference, noise_none)
gps.SetName("GPS Sensor")
# gps.SetLag(gps_lag)
# gps.SetCollectionWindow(gps_collection_time)

gps.PushFilter(sens.ChFilterGPSAccess())
manager.AddSensor(gps)

# ---------------
# Simulation loop
# ---------------
time = 0.0
gps_log = []
camera_log = []

while (time < tend) :
    time = gator.GetSystem().GetChTime()

    # Collect output data from modules (for inter-module communication)
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_braking = 0.0
    driver_inputs.m_steering = 0.2
    driver_inputs.m_throttle = 0.4

    # Update modules (process inputs from other modules)
    terrain.Synchronize(time)
    gator.Synchronize(time, driver_inputs, terrain)

    # ------------------------------------------------
    # TO DO: Update sensor manager
    # ------------------------------------------------
    manager.Update()
    # ------------------------------------------------
    # TO DO: Get GPS data from the sensor so you can save and/or plot it
    # ------------------------------------------------
    gps_data = gps.GetMostRecentGPSBuffer()
    camera_data_RGBA8 = cam.GetMostRecentRGBA8Buffer()

    if gps_data.HasData():
        print("GPS:",gps_data.GetGPSData().shape,"max:",np.max(gps_data.GetGPSData()))
        gps_log.append(gps_data.GetGPSData())

    if camera_data_RGBA8.HasData():
        print("Camera RGBA8:",camera_data_RGBA8.GetRGBA8Data().shape,"max:",np.max(camera_data_RGBA8.GetRGBA8Data()))
        camera_log.append(camera_data_RGBA8.GetRGBA8Data())


    # Advance simulation for one timestep for all modules
    terrain.Advance(step_size)
    gator.Advance(step_size)
