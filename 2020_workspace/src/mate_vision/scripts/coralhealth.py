from math import sqrt
import requests
from coralimages import PipeImage, PipeImgDiff
import sys

#Specify file paths as command line inputs
#e.g. python coralhealth.py beforefilepath afterfilepath
if(len(sys.argv)==3):
    before = sys.argv[1]
    after = sys.argv[2]
    before_img = PipeImage(before)
    after_img = PipeImage(after)
#Specify file urls as command line inputs with any one additional argument
#e.g. python coralhealth.py beforeurl afterurl urls
elif(len(sys.argv)>3):
    with open("before", "wb") as image:
        image.write(requests.get(sys.argv[1]).content)
    with open("after", "wb") as image:
        image.write(requests.get(sys.argv[2]).content)
        before_img = PipeImage("before")
        after_img = PipeImage("after")
#default before and after iamges
else:
    with open("before", "wb") as image:
        image.write(requests.get("http://www.columbia.edu/~km3533/before.jpg").content)
    with open("after", "wb") as image:
        image.write(requests.get("http://www.columbia.edu/~wsh2117/after9.jpg").content)
    before_img = PipeImage("before")
    after_img = PipeImage("after")

after_img.align(before_img.orig_img)

#use default threshing function
before_img.get_thresh()
after_img.get_thresh()

#Show before or after image with their thresholds
#before_img.show()
#after_img.show()


img_difference = PipeImgDiff(before_img,after_img)
img_difference.find_changes()
img_difference.draw_changes()

#Show just the before, after, and difference
#img_difference.show_diff()

#Show just the labeled after image
#img_difference.show_labeled()

#Show the before, after, difference, and labeled after image
img_difference.show_all()
