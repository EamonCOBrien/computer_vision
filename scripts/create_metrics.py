import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt
import math
import os
import time

from candy_classifier import ImageClassifier

if __name__ == '__main__':

	times = []
	for i in range (10):
		detector = ImageClassifier()
		detector.train = detector.create_data(i+1)
		detector.run()
		end = time.time()
		diff = end-detector.start_time
		print(diff)
		times.append(diff)

	# Plot the results
	xs = np.arange(1, 11)
	plt.plot(xs, times)
	plt.ylabel('Runtime')
	plt.xlabel('Number of images per class in dataset')
	plt.show()
