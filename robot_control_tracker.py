###
#
# Grid based robot tracking
#
#   Via bluetooth communication, Rover reports
#   its current turn assignment and when it engages
#   in a turn, from which this supervisor can compute
#   current relative location and heading.
#
###

print("importing cv2")
import cv2
print("importing operator")
import operator
print("importing time")
import time
print("importing numpy")
import numpy as np
print("importing scipy")
from scipy import ndimage
print("importing serial")
import serial

#Start serial port
ser = serial.Serial("COM12")

#Image level threshold value
level_thresh = 100

print("activating camera")
cam = cv2.VideoCapture(1)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 120);
cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 180);

#heading transitions for different turns
Red = {1:4,2:1,3:2,4:3} #Left
Green = {1:2,2:3,3:4,4:1} #Right
Blue = {1:3,2:4,3:1,4:2} #Turnabout

#Grid movement for different headings- h:(dx,dy)
move = {1:(0,1),2:(1,0),3:(0,-1),4:(-1,0)}
card_head = {1:"N",2:"E",3:"S",4:"W"}

#initial heading, turn assignment, and position
di = 1
color = "Red"
pos = (0,-2)

#Cage state variable
cage_state = 0

#Color readings for color reporting
comm_col = {108:"Red",114:"Green",116:"Blue"}

#Motion behavior keys
key_list = {108:'l',114:'r',116:'t',99:'c'}

#listen for init from rover
print("waiting for init...")

#First lines from startup
n = 0
while n < 4:
    print(ser.readline(),n)
    n+=1

print("opening window")
winName = "robot tracking"
cv2.namedWindow(winName, cv2.CV_WINDOW_AUTOSIZE)

print(card_head[di],color,pos)
window_mask = np.zeros((240, 360))

#init tracking variables
dmin = 100000
dmlabel = 0
comr = [120,180]
coms = [-1,-1]

#Loop indefinitely
while True:

    #Read image
    t = cam.read()[1]

    #Produce labeled copy of levels
    o_level = np.copy(t)
    t = 1*(cv2.blur(cv2.cvtColor(t, cv2.COLOR_RGB2GRAY),(4,4)))
    labels,num_labels = ndimage.label((t<level_thresh))

    #Looping over the labels
    dmin = 10000 #Init minimum at over-high value
    ds = [] #list of areas
    dmlabel = -1 #selected label
    for a in range(num_labels+1):

        #calculate the area of the label
        area = np.sum(1.0*(labels == a))

        #If a low area (not the background)
        if (area < 10000):
            ds.append(area) #Add to areas list

            #compute dist to most recent COM
            com = ndimage.measurements.center_of_mass(1.0*(labels == a))
            xd = com[0] - comr[0]
            yd = com[1] - comr[1]
            d = xd**2 + yd**2

            #update if less than current computed minimum
            if d < dmin:
                dmin = d
                dmlabel = a
                coms[0] = com[0]
                coms[1] = com[1]

    #Update reference COM
    comr[0] = coms[0]
    comr[1] = coms[1]

    #Display image
    key = cv2.waitKey(10)
    if key == 27: #On escape press, close window and end loop
       cv2.destroyWindow(winName)
       break
    else: #Otherwise
        if key in key_list: #for each key in the list
            if key in comm_col: #If in the colors
                color = comm_col[key] #Grab the color
            ser.write(key_list[key]) #Write the label
            if key == 116: #on a 116 (blue)
                pos = (pos[0] + move[di][0],pos[1] + move[di][1]) #update position
                di = Blue[di] #flop directions
            if key == 99: #for a cage state change
                cage_state = 1 - cage_state #Toggle cage state

    #Read in serial transmissions
    s = ser.readline()
    st = s.split(",") #split by ,

    #If the state is a tuen
    if "Turning" in s:
        #Update position
        pos = (pos[0] + move[di][0],pos[1] + move[di][1])

        #Sort di codes by color
        if (color == "Red"):
            di = Red[di]
        elif (color == "Green"):
            di = Green[di]
        elif (color == "Blue"):
            di = Blue[di]

    #show tracked blob region
    out = 0*np.copy(o_level)
    f = 1.0*(cv2.blur(1.0*(labels == dmlabel),(6,6))>0)
    out[:,:,0] = o_level[:,:,0]*(f)
    out[:,:,1] = o_level[:,:,1]*(f)
    out[:,:,2] = o_level[:,:,2]*(f)

    #Mark output target with a dot
    out[comr[0]-3:comr[0]+3,comr[1]-3:comr[1]+3,0] = 255
    out[comr[0]-3:comr[0]+3,comr[1]-3:comr[1]+3,2] = 0
    out[comr[0]-3:comr[0]+3,comr[1]-3:comr[1]+3,1] = 0

    #print output values
    print(comr,card_head[di],pos)

    #Show the output image
    cv2.imshow( winName, out)

#After loop
print "cleaning up"
ser.close() #Close serial and remove it
del ser
