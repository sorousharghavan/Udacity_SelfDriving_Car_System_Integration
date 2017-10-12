from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        self.boundaries = [
                ([0, 160, 60], [180, 255, 255]), #Red
                ([20, 100, 100], [30, 255, 255]), #Yellow
                ([38, 50, 50], [75, 255, 255]) #Green
        ]
        self.counters = [0,0,0] #Keeps track of the frequency of occurence for each of the 3 colors
        self.colors = {0: TrafficLight.RED, 1: TrafficLight.YELLOW, 2: TrafficLight.GREEN}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for i,boundary in enumerate(self.boundaries):
            #convert to numpy arrays
            lower_color = np.array(boundary[0])
            upper_color = np.array(boundary[1])
            #create mask from colour bounds
            mask = cv2.inRange(img_hsv, lower_color, upper_color)
            #count found colour pixels
            self.counters[i] = cv2.countNonZero(mask)
        
        return self.colors[self.counters.index(max(self.counters))]
