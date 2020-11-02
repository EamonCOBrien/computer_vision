#!/usr/bin/env python3

""" This is a script that walks through some of the basics of working with
    images with opencv in ROS. """


import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt
import math

class ImageClassifier():
	def __init__(self):
		self.query = self.load_image('../imgs/snickers_query.jpg') 
		candy_names = ['skittles', 'snickers', 'nerds', 'reeses', 'almondjoy']
		train_imgs = self.get_data(candy_names)
		avgs = np.zeros(len(train_imgs))
		self.train = pd.DataFrame(list(zip(candy_names, train_imgs, avgs)), columns=['names', 'imgs', 'avgs'])

	def get_data(self, names):
		train_imgs = []
		for n in names:
			img = self.load_image('../imgs/'+n+'.jpg')
			train_imgs.append(img)
		return train_imgs

	def load_image(self, url):
		return cv.imread(url,cv.IMREAD_GRAYSCALE) 

	def ORB_detection(self, query, train):
		# Initiate ORB detector
		orb = cv.ORB_create()

		# find the keypoints and descriptors with ORB
		kp1, des1 = orb.detectAndCompute(query,None)
		kp2, des2 = orb.detectAndCompute(train,None)

		# create BFMatcher object
		bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

		# Match descriptors.
		matches = bf.match(des1,des2)

		# Sort them in the order of their distance.
		matches = sorted(matches, key = lambda x:x.distance)

		# Draw first 10 matches.
		# res = cv.drawMatches(query,kp1,train,kp2,matches[:10],None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
		# plt.imshow(res),plt.show()

		# Average distances
		total = 0
		for m in matches[:5]:
			total += m.distance
		avg = total/5
		return avg

	def run(self):
		# Iterate through all the images
		for index, img in enumerate(self.train['imgs']):
			avg = self.ORB_detection(self.query, img)
			self.train.at[index, 'avgs'] = avg
		print("List of averages", self.train['avgs'])

		# Get the minimum avg (best match)
		min_index = self.train['avgs'].argmin()
		print("Best match: ", self.train['names'][min_index])


if __name__ == '__main__':
	detector = ImageClassifier()
	detector.run()
