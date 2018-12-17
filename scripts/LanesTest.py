# File to test the Lanes code
from Lanes import FindLanes
import glob
import cv2


finelanes = FindLanes("calibrate_matrix.pickle")

images = glob.glob('images/*.png')
for image in images:
    img = cv2.imread(image) # BGR image
    filename = 'output/' + image.split("/")[1]
    img_RGB, cte = finelanes.img_process(img)
    print (filename, cte)
    cv2.imwrite(filename, img_RGB) #cv2.cvtColor(img_RGB, cv2.COLOR_RGB2BGR))

