import shape
from pyefd import elliptic_fourier_descriptors, plot_efd
import cv2 as cv
import numpy as np
import math
import pickle

class Manipulandum(object):

    def __init__(self) -> None:
        self.__phi = 0
        self.__m = 10
        self.__contour = self.__generate_contour()

    def __generate_contour(self):
        #contour_original = shape.get_bezier_curve()
        file_path = '/home/romi-lab-2/manipulandums_ws/src/bezier/bezier/scripts/bezier_curve.pkl'
        with open(file_path, 'rb') as f:
            contour_original = pickle.load(f)
        # th = np.linspace(0, 2 * np.pi, 100)
        # contour_original = [0.5 * np.cos(th), 0.3 * np.sin(th)]

        contour_original = np.array(contour_original[:2])
        contour_original -= self.__com(contour_original)

        self.__coeffs = elliptic_fourier_descriptors(contour_original.T, order = self.__m)

        z = []
        s_array = np.linspace(0, 1, 500)
        for i in range(self.__m):
            z_k = np.zeros((len(s_array),2))
            j = 0
            coef = self.__coeffs[i,:].reshape(2, 2)
            for s in s_array:
                arg = 2 * np.pi * (i + 1) * s
                exp = np.array([[np.cos(arg)], [np.sin(arg)]])
                z_k[j, :] = np.matmul(coef, exp).T
                j += 1
            z.append(z_k)

        contour_fourier = sum(z).T
        # contour_fourier -= self.__com(contour_fourier)

        self.__com = self.__com(contour_fourier)

        return contour_fourier

    def __com(self, contour):
        points = []
        for i in range(contour[0].shape[0]):
            x = int(contour[0, i] * 1000)
            y = int(contour[1, i] * 1000)

            point = [x, y]
            points.append([point])

        M = cv.moments(np.array(points))
        x = int(M["m10"] / M["m00"]) / 1000
        y = int(M["m01"] / M["m00"]) / 1000

        com = np.array([[x, y]]).T

        return com

    def get_point(self, s) -> list:
        coords = []

        for h in range(self.__m):
            arg = 2 * np.pi * (h + 1) * s
            exp = np.array([[np.cos(arg)], [np.sin(arg)]])

            coef = self.__coeffs[h,:].reshape(2, 2)
            coord_h = np.matmul(coef, exp).T

            coords.append(coord_h)

        point = sum(coords).T

        return point

    def get_tangent(self, s) -> float:
        diffs = []

        for h in range(self.__m):
            c = 2 * np.pi * (h + 1)
            arg = c * s
            exp = np.array([[-c * np.sin(arg)], [c * np.cos(arg)]])

            coef = self.__coeffs[h,:].reshape(2, 2)
            diffs_h = np.matmul(coef, exp).T

            diffs.append(diffs_h)

        diff = sum(diffs).T
        theta = math.atan(diff[1]/diff[0])

        return theta

    def get_x_hat_direc(self, s) -> float:
        theta = self.get_tangent(s)
        p1 = self.get_point(s)

        p2 = [p1[0] + np.cos(theta), p1[1] + np.sin(theta)]

        p3 = [p1[0] + np.cos(theta - np.pi/2), p1[1] + np.sin(theta - np.pi/2)]

        cross_prod1 = (p1[0] - p2[0]) * (self.com[1] - p2[1]) - (p1[1] - p2[1]) * (self.com[0] - p2[0])
        cross_prod2 = (p1[0] - p2[0]) * (p3[1] - p2[1]) - (p1[1] - p2[1]) * (p3[0] - p2[0])

        condition = (abs(cross_prod1 * cross_prod2) - cross_prod1 * cross_prod2) / (-2 * cross_prod1 * cross_prod2)

        return theta + condition * np.pi

    @property
    def m(self) -> int:
        return self.__m

    @property
    def com(self) -> list:
        return self.__com

    @property
    def phi(self) -> float:
        return self.__phi

    @property
    def contour(self) -> object:
        return self.__contour

    @property
    def coeffs(self) -> object:
        return self.__coeffs
