###
#
# Visual field tracking
#   Takes in reference image of viewpoint, and
#   used movement tracking to identify and follow
#   individual additions to the field of view by
#   examining presence of specific blob segments in
#   current frame vs those in reference.
#   Can track multiple objects.
#   Will reset viewpoint if tracking shows visual
#   movement below a threshold for certain period
#   of time.
#
###

print "importing cv2"
import cv2
print "importing operator"
import operator
print "importing time"
import time
print "importing numpy"
import numpy as np
print "importing scipy"
from scipy import ndimage

print "activating camera"
cam = cv2.VideoCapture(1)
cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 120);
cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 180);

print "opening window"
winName = "tracking"
cv2.namedWindow(winName, cv2.CV_WINDOW_AUTOSIZE)
window_mask = np.zeros((240, 360))

# Set level thresholds for imaging
level_thresh1 = 100
level_thresh2 = 140
b_thresh = 0.5

# reset flag
resetting = 0

#Get reference background
print "grabbing reference"
for i in range(10): #give 10 images for everything to settle
    t = cam.read()[1] #read images
    time.sleep(0.25) #wait 1/4 second

o_level = np.copy(t)*0  #visual level array
t = 1*(cv2.blur(cv2.cvtColor(t, cv2.COLOR_RGB2GRAY),(4,4))) #blur the background image
labels1,num_labels1 = ndimage.label((t<level_thresh1)) #Label over/under regions
labels2,num_labels2 = ndimage.label((t>level_thresh2))

#Make the reference images relative to the background threshold
o_ref1 = 255.0*(cv2.blur(1.0*(labels1 > 0),(3,3))>b_thresh)
o_ref2 = 255.0*(cv2.blur(1.0*(labels2 > 0),(3,3))>b_thresh)

#Previous centers of mass
com_p = [0]*20

#Run indefinitely
while True:

    #Read the camera
    t = cam.read()[1]

    #Copy the current image frame
    to = np.copy(t)
    o_level = np.copy(t)*0 #Output level

    #Blur image for blobbing
    t = 1*(cv2.blur(cv2.cvtColor(t, cv2.COLOR_RGB2GRAY),(4,4)))

    #two inverse blob IDs segment the whole image
    labels1,num_labels1 = ndimage.label((t<level_thresh1))
    labels2,num_labels2 = ndimage.label((t>level_thresh2))

    #Calculate blob levels from blob threshold
    l1 = 255.0*(cv2.blur(1.0*(labels1 > 0),(3,3))>b_thresh)
    l2 = 255.0*(cv2.blur(1.0*(labels2 > 0),(3,3))>b_thresh)

    #compare reference segmented vs current segmented    
    o_l = 1.0*(cv2.blur(1.0*(o_ref1 != l1) + 1.0*(o_ref2 != l2),(3,3))>b_thresh)

    #segment comparison image
    labels,num_labels = ndimage.label((o_l>0.0))

    #Run through list of labeled difference zones and find those over a given threshold
    amax = []
    llist = []
    for a in range(num_labels): #Check each label
        sel = 1.0*(labels==a+1) #get current selection

        #pruning for size
        if np.sum(sel) > 1500:
            com = ndimage.measurements.center_of_mass(sel) #get that label's COM
            llist = llist + [com[0],com[1]] #Add to the point list
            amax.append(np.sum(sel)) #Add area to area list
            o_level[:,:,1] = o_level[:,:,1] + sel #Add to the output level

    #LP filtered COM tracking
    if len(amax)>0:
        com_p = com_p*0.95 + np.array(llist+[0]*(20-len(llist)))*0.05
    else:
        #reset list if no tracking
        com_p = np.array([0.0]*20)

    #output areas and reset metric
    print amax,np.sum(np.abs(com_p - np.array(llist+[0]*(20-len(llist)))))

    #reset reference if criteria met
    if (len(amax)>0) and np.sum(np.abs(com_p - np.array(llist+[0]*(20-len(llist)))))<2.0 and resetting:
        #Take two cycles for motion settling
        for i in range(2):
            t = cam.read()[1]
            time.sleep(0.25)

        #Make new output level
        o_level = np.copy(t)*0
        t = 1*(cv2.blur(cv2.cvtColor(t, cv2.COLOR_RGB2GRAY),(4,4))) #Blur image

        labels1,num_labels1 = ndimage.label((t<level_thresh1)) #Make new labels
        labels2,num_labels2 = ndimage.label((t>level_thresh2))

        o_ref1 = 255.0*(cv2.blur(1.0*(labels1 > 0),(3,3))>b_thresh) #Make new reference levels
        o_ref2 = 255.0*(cv2.blur(1.0*(labels2 > 0),(3,3))>b_thresh)

        amax_p = np.array([0.0]*10) #make new area list

    #generate output image w/ tracked area highlighted
    to[:,:,0] = to[:,:,0]*(o_level[:,:,1] != 1.0)
    to[:,:,2] = to[:,:,2]*(o_level[:,:,1] != 1.0)
    to[:,:,1] = to[:,:,1]*(o_level[:,:,1] != 1.0) + 255.0*(o_level[:,:,1] == 1.0)

    #Show tracking output
    cv2.imshow( winName, to)
    key = cv2.waitKey(10)
    if key == 27: #On escape press, close output image and end loop
       cv2.destroyWindow(winName)
       break

    
