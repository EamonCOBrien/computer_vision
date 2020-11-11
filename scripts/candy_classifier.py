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
		self.query = self.load_image('../imgs/snickers_query.jpg')
		self.candy_names = ['Haribo', 'Nerds', 'Reeses', 'Skittles', 'Starburst', 'Snickers', 'Swedish_Fish', 'Twizzlers']
		self.dataset_size = 5
		self.train = self.create_data(self.dataset_size)

		# For timing:
		self.start_time = time.time()

	def create_data(self, size):
		"""
		Initialize the dataset
		params:
			size: specifies the number of images to include from each class
		"""
		self.dataset_size = size
		train_imgs = []
		for name in self.candy_names:
			candy_imgs = []
			# Get images from /imgs in repo
			for root, dirs, files in os.walk('../imgs/'+name):
				for i in range(size):
					# Only keep as many images as specified in size. Stops if not enough are available
					if i >= len(files):
						break
					candy_imgs.append(self.load_image('../imgs/'+name+'/'+files[i]))
			train_imgs.append(candy_imgs)

		avgs = np.zeros(len(self.candy_names))
		# Store all data in a DataFrame for easier manipulation
		self.train = pd.DataFrame(list(zip(self.candy_names, train_imgs, avgs)), columns=['names', 'imgs', 'avgs'])
		return self.train

	def load_image(self, url):
		return cv.imread(url,cv.IMREAD_GRAYSCALE)

	def ORB_detection(self, query, train):
		# Initiate ORB detector
		orb = cv.ORB_create()

		# find the keypoints and descriptors with ORB
		kp1, des1 = orb.detectAndCompute(query,None)
		kp2, des2 = orb.detectAndCompute(train,None)

		bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

		# Match descriptors.
		matches = bf.match(des1,des2)

		# Sort them in the order of their distance.
		matches = sorted(matches, key = lambda x:x.distance)

		# Calculate the average of the top 5 matches, as our accuracy metric
		total = 0
		for m in matches[:5]:
			total += m.distance
		avg = total/5
		return avg

	def run(self):
		self.start_time = time.time()
		# Iterate through all the images
		for index, imgs in enumerate(self.train['imgs']):
			match_values = []
			for img in imgs:
				match_values.append(self.ORB_detection(self.query, img))
			# Get average of the avg. match value for each picture in the dataset
			avg = sum(match_values)/len(match_values)
			self.train.at[index, 'avgs'] = avg
		print("List of averages"+'\n', self.train['avgs'])

		# Get the minimum total avg (best match)
		min_index = self.train['avgs'].argmin()
		candy_name = self.train['names'][min_index]
		end = time.time()
		print("Best match: ", candy_name)
		print("Time elapsed: ", end-self.start_time)
		return candy_name



if __name__ == '__main__':
	detector = ImageClassifier()
	detector.run()
