#!/usr/bin/env python3

""" This is a script that walks through some of the basics of working with
	images with opencv in ROS. """


import numpy as np
import pandas as pd
import cv2 as cv
import matplotlib.pyplot as plt
import math
import os
import time

class ImageClassifier():
	def __init__(self):
		self.query = self.load_image('../imgs/nerds_dark.jpg')
		self.candy_names = ['Haribo', 'Nerds', 'Reeses', 'Skittles', 'Starburst', 'Snickers', 'Swedish_Fish', 'Twizzlers']
		self.dataset_size = 1
		self.train_imgs = self.get_data()
		avgs = np.zeros(len(self.candy_names))
		self.train = pd.DataFrame(list(zip(self.candy_names, self.train_imgs, avgs)), columns=['names', 'imgs', 'avgs'])

		# For timing: 
		self.start_time = time.time()

	def get_data(self):
		train_imgs = []
		for name in self.candy_names:
			candy_imgs = []
			for root, dirs, files in os.walk('../imgs/'+name):
				for i in range(self.dataset_size):
					if i >= len(files):
						break
					candy_imgs.append(self.load_image('../imgs/'+name+'/'+files[i]))
			train_imgs.append(candy_imgs)
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
		for index, imgs in enumerate(self.train['imgs']):
			match_values = []
			for img in imgs:
				match_values.append(self.ORB_detection(self.query, img))
			avg = sum(match_values)/len(match_values)
			self.train.at[index, 'avgs'] = avg
		print("List of averages", self.train['avgs'])

		# Get the minimum avg (best match)
		min_index = self.train['avgs'].argmin()
		candy_name = self.train['names'][min_index]
		end = time.time()
		print("Best match: ", candy_name)
		print("Time elapsed: ", end-self.start_time)
		return candy_name



if __name__ == '__main__':
	detector = ImageClassifier()
	detector.run()
