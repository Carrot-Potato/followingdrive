#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 30 19:22:29 2021

@author: robot
"""

import matplotlib.pyplot as plt
import cv2
import pickle
import numpy as np
import math

img = cv2.imread('test_image.png')

cv2.imshow('Result',img)
cv2.waitKey(0)
