from styx_msgs.msg import TrafficLight
import matplotlib.pyplot as plt
from PIL import Image
from scipy import ndimage
import numpy as np
import time

def rgb2hsv(image):
    image_hsv = np.array(image.convert("HSV"))
    image_hsv[:,:,0] = np.round(np.array(np.array(
                        image_hsv[:,:,0], dtype=np.int)*180 / 255.), 0
    ).astype(np.uint8)
    return image_hsv


def mask_boundary(image_hsv, M_boundaries, i_hsv):
    mask_h = (image_hsv[:,:,0].astype(np.int) - M_boundaries[i_hsv][0][0] >= 0) *\
             (image_hsv[:,:,0].astype(np.int) - M_boundaries[i_hsv][1][0] <= 0)
    mask_s = (image_hsv[:,:,1].astype(np.int) - M_boundaries[i_hsv][0][1] >= 0) *\
             (image_hsv[:,:,1].astype(np.int) - M_boundaries[i_hsv][1][1] <= 0)
    mask_v = (image_hsv[:,:,2].astype(np.int) - M_boundaries[i_hsv][0][2] >= 0) *\
             (image_hsv[:,:,2].astype(np.int) - M_boundaries[i_hsv][1][2] <= 0)
    return mask_h*mask_s*mask_v


class TLClassifier(object):
    def __init__(self, tl_body_boundary, tl_light_boundary):
        self.tl_body_boundary = tl_body_boundary
        self.tl_light_boundary = tl_light_boundary
        self.counters = [0,0,0] #Keeps track of the frequency of occurence for each of the 3 colors
        self.colors = {0: TrafficLight.RED, 1: TrafficLight.YELLOW, 2: TrafficLight.GREEN}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = Image.fromarray(image, 'RGB')
        img_hsv = rgb2hsv(image)
        time_idx = time.time()
        for i,boundary in enumerate(self.tl_light_boundary):
            #convert to numpy arrays
            lower_color = np.array(boundary[0])
            upper_color = np.array(boundary[1])
            #create mask from colour bounds
            #mask = cv2.inRange(img_hsv, lower_color, upper_color)
            mask = mask_boundary(img_hsv, self.tl_light_boundary, i)

            # For real condition, detect the tl_body region first and determine the light in tl_body region.
            if self.tl_body_boundary != "None":
                image_hsv = np.array(image.convert("HSV"))
                mask_tlbody = mask_boundary(image_hsv, self.tl_body_boundary, 0)
                mask_tlbody = ndimage.binary_erosion(mask_tlbody, iterations=2)
                mask_tlbody = ndimage.binary_erosion(mask_tlbody, iterations=1)
                mask_tlbody = ndimage.binary_dilation(mask_tlbody, iterations=1)
                mask_tlbody = ndimage.binary_dilation(mask_tlbody, iterations=100)
                mask = mask * mask_tlbody
                mask[0:350,:] = 0.

                #fileName = "/home/huboqiang/Udacity_SelfDriving_Car_System_Integration/ros/image/tl.%f.%d.png" % (time_idx, i)
                #fig = plt.figure(figsize=(7,7))
                #ax = fig.add_subplot(111)
                #ax.imshow(mask_tlbody)
                #ax.set_title(np.sum(mask))
                #fig.savefig(fileName)


            #count found colour pixels
            #self.counters[i] = cv2.countNonZero(mask)
            self.counters[i] = np.sum(mask)

        return self.colors[self.counters.index(max(self.counters))]
