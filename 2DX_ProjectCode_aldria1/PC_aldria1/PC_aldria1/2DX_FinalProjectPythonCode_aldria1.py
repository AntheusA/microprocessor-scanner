
# Name: Antheus Aldridge
# Student Number: 400339569
# MacID: aldria1

# To operate the program, first run it. While it still says,
# "Press Enter to start communication" I recommend pressing reset on your
# microcontroller. Then press enter and wait for it to output 0, 1, 2, 3.
# It might take a little while since the sensor has to boot.
# At that point you can press the push button to start the
# data acquisition from the microcontroller

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import numpy as np
import open3d as o3d
import math

numSlices = 10      #Change this if you want more or less slices
                        # - you will also have to change the number of
                        # iterations in Keil program

numVertices = numSlices * 16

s = serial.Serial('COM3', baudrate = 115200, timeout = 15)
print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission

f = open("demofile2dx.xyz", "w")
s.write('s'.encode())
# recieve 10 measurements from UART of MCU
for i in range(19):
        print(i)
        if (i < 3):
            x = s.readline()
        
        if (i >= 3):
            x = s.readline()
            l = x.decode().split(', ')
            l.pop()
            l = [float(i) for i in l]
            y = math.sin(math.radians(l[2]))*l[1]
            z = math.cos(math.radians(l[2]))*l[1]
            f.write('0 {0:f} {1:f}\n'.format(y, z))
            print(l)
            print(y, z)


for xCoord in range(1, numSlices):
    for i in range(16):
        print(i)
        
        if (i >= 0):
            x = s.readline()
            l = x.decode().split(', ')
            l.pop()
            l = [float(i) for i in l]
            y = math.sin(math.radians(l[2]))*l[1]
            z = math.cos(math.radians(l[2]))*l[1]
            f.write('{0:d} {1:f} {2:f}\n'.format(xCoord*300, y, z))
            print(l)
            print(math.radians(l[2]))
       
# the encode() and decode() function are needed to convert string to bytes
# because pyserial library functions work with type "bytes"
f.close()

#close the port
print("Closing: " + s.name)
s.close()

#Read the test data in from the file we created        
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("demofile2dx.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

#OK, good, but not great, lets add some lines to connect the vertices
#   For creating a lineset we will need to tell the package which vertices need connected
#   Remember each vertex actually contains one x,y,z coordinate


#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0, numVertices):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice        
lines = []  
for x in range(0, numVertices - 1,16):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+1]])
    lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+2]])
    lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+3]])
    lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+4]])

    lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+5]])
    lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+6]])
    lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+7]])
    lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+8]])

    lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+9]])
    lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+10]])
    lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+11]])
    lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+12]])

    lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+13]])
    lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+14]])
    lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+15]])
    lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x]])

#Define coordinates to connect lines between current and next yz slice        
for x in range(0, numVertices - 31,16):
        lines.append([yz_slice_vertex[x], yz_slice_vertex[x+16]])
        lines.append([yz_slice_vertex[x+1], yz_slice_vertex[x+17]])
        lines.append([yz_slice_vertex[x+2], yz_slice_vertex[x+18]])
        lines.append([yz_slice_vertex[x+3], yz_slice_vertex[x+19]])
        
        lines.append([yz_slice_vertex[x+4], yz_slice_vertex[x+20]])
        lines.append([yz_slice_vertex[x+5], yz_slice_vertex[x+21]])
        lines.append([yz_slice_vertex[x+6], yz_slice_vertex[x+22]])
        lines.append([yz_slice_vertex[x+7], yz_slice_vertex[x+23]])

        lines.append([yz_slice_vertex[x+8], yz_slice_vertex[x+24]])
        lines.append([yz_slice_vertex[x+9], yz_slice_vertex[x+25]])
        lines.append([yz_slice_vertex[x+10], yz_slice_vertex[x+26]])
        lines.append([yz_slice_vertex[x+11], yz_slice_vertex[x+27]])

        lines.append([yz_slice_vertex[x+12], yz_slice_vertex[x+28]])
        lines.append([yz_slice_vertex[x+13], yz_slice_vertex[x+29]])
        lines.append([yz_slice_vertex[x+14], yz_slice_vertex[x+30]])
        lines.append([yz_slice_vertex[x+15], yz_slice_vertex[x+31]])

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
