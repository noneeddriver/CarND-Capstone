# from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np
import cv2
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
	self.model=load_model('/home/paul/Documents/Udacity/CarND-Capstone/ros/src/tl_detector/light_classification/traffic_light_classifier_model.h5')
	self.graph = tf.get_default_graph()
        

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	prediction = 4
	image_array=np.expand_dims(image, axis=0)
	with self.graph.as_default():
	    prediction=self.model.predict(image_array)
	prediction=round(prediction)
	if prediction < 0.5:
	    prediction = 0
	elif prediction < 1.5:
	    prediction = 1
	elif prediction < 3:
	    prediction = 2
	else:
	    prediction = 4
        return int(round(prediction))
'''
if __name__ == '__main__':
    classifier=TLClassifier()
    img=cv2.imread('/home/paul/Documents/Udacity/CarND-Capstone/imgs/Simulator_Dataset/no_traffic_light/frame0134.jpg') # just as an example to run alone
    print "prediction: ", classifier.get_classification(img)
'''
