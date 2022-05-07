from google.colab import drive
drive.mount('/content/drive')
!pip install paddlepaddle-gpu==2.2.2
import os
from posixpath import dirname
from cv2 import NORMAL_CLONE
import numpy as np 
import cv2
from math import sqrt, floor, ceil
from paddle import digamma
from google.colab.patches import cv2_imshow
def read_image(path):
      image = cv2.imread(path)
      size = image.shape
      dimension = (size[0], size[1])
      return image, size, dimension
      def nearest_interpolation(image, dimension):

      new_image = np.zeros((dimension[0]*2, dimension[1]*2, image.shape[2])) # 2 times zooming
      large_time = int(sqrt(((dimension[0]*2) * (dimension[1]*2)) / (image.shape[0] * image.shape[1])))

      for i in range(dimension[0]*2):
            for j in range(dimension[1]*2):
                  row = floor(i / large_time)
                  column = floor(j / large_time)
                  new_image[i, j] = image[row, column]
      return new_image
      # read image from path
image, size, dimension = read_image("//content//cameraman.tif")
# output_image_1 = nearest_interpolation(image, dimension).astype('uint8')
# cv2.imshow("out", output_image_1)
# cv2.waitKey(0)
def bilinear_interpolation(image, dimension):

      height = image.shape[0]
      width = image.shape[1]

      scale_x = (width) / (dimension[1]*2)
      scale_y = (height) / (dimension[0]*2)

      new_image = np.zeros((dimension[0]*2, dimension[1]*2, image.shape[2]))

      for k in range(3):
            for i in range(dimension[0]*2):
                  for j in range(dimension[1]*2):
                        x = (j+0.5) * (scale_x) - 0.5
                        y = (i+0.5) * (scale_y) - 0.5

                        x_int = int(x)
                        y_int = int(y)

                        # prevent crossing
                        x_int = min(x_int, width-2)
                        y_int = min(y_int, height-2)

                        x_diff = x - x_int
                        y_diff = y - y_int

                        a = image[y_int, x_int, k]
                        b = image[y_int, x_int+1, k]
                        c = image[y_int+1, x_int+1, k]
                        d = image[y_int+1, x_int+1, k]

                        pixle = a * (1-x_diff) * (1-y_diff) + b * (x_diff) *\
                               (1-y_diff) + c*(1-x_diff) * (y_diff) + d*x_diff*y_diff

                        new_image[i, j, k] = pixle.astype('uint8')
      return new_image
      # read image from path
image, size, dimension = read_image("//content//drive//MyDrive//cameraman.tif")
output_image_1 = nearest_interpolation(image, dimension).astype('uint8')
cv2_imshow(output_image_1)
def bilinear_interpolation(image, dimension):

      height = image.shape[0]
      width = image.shape[1]

      scale_x = (width) / (dimension[1]*2)
      scale_y = (height) / (dimension[0]*2)

      new_image = np.zeros((dimension[0]*2, dimension[1]*2, image.shape[2]))

      for k in range(3):
            for i in range(dimension[0]*2):
                  for j in range(dimension[1]*2):
                        x = (j+0.5) * (scale_x) - 0.5
                        y = (i+0.5) * (scale_y) - 0.5

                        x_int = int(x)
                        y_int = int(y)

                        # prevent crossing
                        x_int = min(x_int, width-2)
                        y_int = min(y_int, height-2)

                        x_diff = x - x_int
                        y_diff = y - y_int

                        a = image[y_int, x_int, k]
                        b = image[y_int, x_int+1, k]
                        c = image[y_int+1, x_int+1, k]
                        d = image[y_int+1, x_int+1, k]

                        pixle = a * (1-x_diff) * (1-y_diff) + b * (x_diff) *\
                               (1-y_diff) + c*(1-x_diff) * (y_diff) + d*x_diff*y_diff

                        new_image[i, j, k] = pixle.astype('uint8')
      return new_image
      output_image_2 = bilinear_interpolation(image, dimension).astype('uint8')
cv2_imshow(output_image_2)
def W(x):
      a = -0.5
      pos_x = abs(x)
      if -1 <= abs(x) <= 1:
            return  ((a+2)*(pos_x**3)) - ((a+3)*(pos_x**2)) + 1
      elif 1 < abs(x) < 2 or -2 < x < -1:
            return ((a * (pos_x**3)) - (5*a*(pos_x**2)) + (8 * a * pos_x) - 4*a)
      else:
            return 0
            def bicubic_interpolation(image, dimension):
      

      nrows = dimension[0]
      ncols = dimension[1]

      output = np.zeros((nrows*2, ncols*2, image.shape[2]), np.uint8)
      for k in range(image.shape[2]):
            for i in range (nrows*2):
                  for j in range (ncols*2):
                        xm = (i + 0.5) * (image.shape[0]/ (dimension[0]*2)) - 0.5
                        ym = (j + 0.5) * (image.shape[1]/ (dimension[1]*2)) - 0.5

                        xi = floor(xm)
                        yi = floor(ym)

                        u = xm - xi
                        v = ym - yi

                        out = 0
                        for n in range(-1, 3):
                              for m in range(-1, 3):
                                    if ((xi + n < 0) or (xi + n >= image.shape[1]) or (yi + m < 0) or (yi +m >= image.shape[0])):
                                          continue

                                    out += (image[xi+n, yi+m, k] * (W(u-n) * W(v-m)))
                        
                        output[i, j, k] = np.clip(out, 0, 255)
      return output
      output_image_3 = bicubic_interpolation(image, dimension)
cv2_imshow(output_image_3)
