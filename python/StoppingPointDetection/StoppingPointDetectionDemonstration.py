import numpy as np
import cv2 as cv


BOX_SIZE = 50
THRESH = 0.01
def score(img, x, y):
    return np.sum(img[x-BOX_SIZE:x+BOX_SIZE, y-BOX_SIZE:y+BOX_SIZE])

filename = 'StopMark.jpg'
img = cv.imread(filename)
#img = cv.GaussianBlur(img,(5,5),0)
hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
gray = cv.inRange(hsv, (100, 0, 0), (255, 255, 255))
gray = cv.GaussianBlur(gray,(7,7),0)
cv.namedWindow('dst2', cv.WINDOW_NORMAL)
cv.imshow('dst2',gray)
gray = np.uint8(gray)
grayf = np.float32(gray)
#grayf = cv.dilate(grayf,None)
#grayf = cv.GaussianBlur(grayf,(10,10),0)

dst = cv.cornerHarris(grayf,2,3,0.01)
#result is dilated for marking the corners, not important
#dst = cv.dilate(dst,None)
# Threshold for an optimal value, it may vary depending on the image.


# Create a mask to identify corners
#mask = np.zeros_like(grayf)
 
# All pixels above a certain threshold are converted to white         
#mask[dst>THRESH*dst.max()] = 255
ret, dst = cv.threshold(dst,0.01*dst.max(),255,0)
 
# Convert corners from white to red.
#img[dst > 0.01 * dst.max()] = [0, 0, 255]
 
# Create an array that lists all the pixels that are corners
coordinates = np.argwhere(dst)
 
# Convert array of arrays to lists of lists
coordinates_tuples = [tuple(l.tolist()) for l in list(coordinates)]
 
# Convert list to tuples
#coordinates_tuples = [tuple(l) for l in coordinates_list]

#filter
filtered = [(x, y) for x, y in coordinates_tuples if 0.8*255*(2*BOX_SIZE)**2 > score(gray, x, y) > 0.7*255*(2*BOX_SIZE)**2]

clusters = []
CLUSTER_RADIUS_SQ = 10**2
for x1, y1 in filtered:
    for i in range(0, len(clusters)):
        c, x2, y2 = clusters[i]
        if((x1-x2)**2+(y1-y2)**2 <= CLUSTER_RADIUS_SQ):
            x2 = (x1/(c+1)) + (x2*c/(c+1))
            y2 = (y1/(c+1)) + (y2*c/(c+1))
            c = c + 1
            clusters[i] = (c, x2, y2)
            break
    else:
        clusters.append((1, x1, y1))
clusters.sort(reverse = True)

dst = cv.dilate(dst,None)
dst = cv.dilate(dst,None)
img[dst>THRESH*dst.max()]=[0,255,255]

for c, x, y in clusters:
    img[int(x),int(y)] = [0,0,255];
    cv.circle(img, (int(y),int(x)), 25, (0,0,255), 3)

if(len(clusters) >= 4):
    
    centroid_x = int(np.mean([x for c, x, y in clusters[0:4]]))
    centroid_y = int(np.mean([y for c, x, y in clusters[0:4]]))
    img[centroid_x-10:centroid_x+10,centroid_y-10:centroid_y+10] = [0,255,0];
    
    cv.namedWindow('dst', cv.WINDOW_NORMAL)
    cv.imshow('dst',img)
    
    
if cv.waitKey(0) & 0xff == 27:
   cv.destroyAllWindows()
    