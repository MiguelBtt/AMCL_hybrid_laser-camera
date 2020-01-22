#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
from detector.msg import messagedet
from detector.msg import marker as msg_marker
from detector.msg import pixels_cloud, pixels_corners, num_markers, num_points_cloud
from geometry_msgs.msg import Point32
from std_msgs.msg import UInt8


class image_converter:

    
    def __init__(self, numImages):

        self.bridge = CvBridge()
        self.pub_marker = rospy.Publisher('detected_markers', messagedet, queue_size=1)
        self.pub_projection = rospy.Publisher('projection_points_cloud', Image, queue_size=1)
        self.pub_num_marker = rospy.Publisher('num_detecte_markers', num_markers, queue_size=10)
        self.pub_num_points_cloud = rospy.Publisher('num_points_cloud', num_points_cloud, queue_size=20)
        self.colours = ((255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 255, 255)) ## BGR coners

        # Ajustes iniciales
        self.numMarker = numImages
        numFeatures = 200 # max image features numbers
        self.thresh = 70 # umbral para pasar la imagen a escala de grises
        self.min_w_h_image = 50 # minimun weidth and height image
        self.thresh_biary = cv2.THRESH_BINARY_INV # cv2.THRESH_BINARY_INV busca objetos oscuros con fondo claro y cv2.THRESH_BINARY_ busca objetos claros con fondo oscuro
        self.numMatches = 15
        self.minNumMatches = 15
        self.maxDist = 60
        self.minDist = 5 # minimun distance between the first and second distance
                        
        
        #Iniciar detector SIFT
        self.detector = cv2.ORB_create(numFeatures)
        # crea BFMatcher object
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
           
        self.markers = (list(), list(), list())        
        for i in range(0, self.numMarker):
            marker = str("marker" + str(i))
            path = str("/home/miguel/.gazebo/models/materials_2/textures/" + marker + ".png")
            print(path)
            img = cv2.imread(path, 0)
            #cv2.imshow(str("Marker" + str(i)), img)
            self.markers[0].append(img)
            kp, des = self.detector.detectAndCompute(img, None)
            self.markers[1].append(kp)
            self.markers[2].append(des)
        
        print("total images in folder: " + str(len(self.markers[0])))

        self.img_canvas = "Squares"
        #self.img_point_cloud = "Point Cloud"
        cv2.namedWindow(self.img_canvas, flags=cv2.WINDOW_NORMAL)
        #cv2.namedWindow(self.img_point_cloud, flags=cv2.WINDOW_NORMAL)

        self.image_sub = rospy.Subscriber("/camera/RGB/image_raw", Image, self.callback)
        #self.corners_sub = rospy.Subscriber("/pixels_clouds", pixels_cloud, self.cornersCloudCallback)
        

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_width = cv_image.shape[1]
            self.original_height = cv_image.shape[0]
        except CvBridgeError as e:
            print(e)
        
        #cv2.imshow("Image window", cv_image)
        self.findContours(cv_image)
        cv2.waitKey(3)

    def findContours(self, img):
        ##(2) convert to hsv-space, then split the channels
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
        """cv2.imshow("image", hsv)
        cv2.imshow("image_h", h)
        cv2.imshow("image_s", s)"""

        #Scv2.imshow("image_v", v)

        #(3) threshold the S channel using adaptive method(`THRESH_OTSU`) or fixed thresh
        #threshold(src, thresh, maxval, type[, dst]) = retval, dst
        #th, threshed = cv2.threshold(v, 100, 255, cv2.THRESH_BINARY_INV)
        #cv2.THRESH_BINARY_INV busca objetos oscuros con fondo claro
        #cv2.THRESH_BINARY_ busca objetos claros con fondo oscuro
        th, threshed = cv2.threshold(v, self.thresh, 255, self.thresh_biary)
        ##cv2.imshow("threshed", threshed)

        ##(4) find all the external contours on the threshed S
        #_, cnts, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        ## cnts is a list with len iqual to contours number 
        canvas  = img.copy()
        self.canvas_cloud = img.copy()
        cv2.drawContours(canvas, cnts, -1, (0,255,0), 1)
        #cv2.imshow("Contours", canvas)

        self.findSquares(cnts, canvas, img)

    def findSquares(self, cnts, canvas, original_img):
        ## A loop for each finded contour
        #print("#########################################")
        #print("contours number is: " + str(len(cnts)))
        #count = 0
        new_msg_marker = messagedet()
        for cnt in cnts: # Un bucle por cada contorno
            ## approx the contour, so the get the corner points
            arclen = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02* arclen, True)
            if len(approx) == 4: ## solo entramos si es un rectangulo, four corners
                #print(approx) # prints contours
                #cv2.drawContours(canvas, [cnt], -1, (255,0,0), 1, cv2.LINE_AA)
                cv2.drawContours(canvas, [approx], -1, (255, 0, 0), 1, cv2.LINE_AA)
                w_img_ = list()
                h_img_ = list()
                count2 = 0

                for i in approx:
                    w_img_.append(i[0][0])
                    h_img_.append(i[0][1])
                    #cv2.circle(canvas, (i[0][0],i[0][1]), 2, self.colours[count2], -1)
                    count2+=1
                w_img = sorted(w_img_)
                h_img = sorted(h_img_)
                """print("w_img-> ")
                print(w_img)
                print("y_img-> ")
                print(h_img)
                print("----")"""
                width = w_img[3]-w_img[0]
                height = h_img[3]-h_img[0]
                w_img_max = w_img[0]+width
                h_img_max = h_img[0]+height
                #print("width: " + str(width) + ", height: " + str(height))
                ## The image has to have unless ten pixels
                #print("w_img[0], h_img[0]: " + str(w_img[0]) + ", " + str(h_img[0]))
                #print("w_img_max, h_img_max: " + str(w_img_max) + ", " + str(h_img_max))
                #print("width, h_img_max: " + str(width) + ", " + str(h_img_max))
                #print("self.original_width, self.original_height: " + str(self.original_width) + ", " + str(self.original_height))
                if w_img[0] > 5  and  h_img[0] > 5  and  w_img_max < (self.original_width - 5)  and h_img_max < (self.original_height - 5)  and  width >= self.min_w_h_image  and  height >= self.min_w_h_image:
                    ## Recortamos la imagen del contorno
                    crop_img = original_img[h_img[0]:h_img_max, w_img[0]:w_img_max]
                    #cv2.imshow("Crop", crop_img)

                    w_center = w_img[0] + int(width/2)
                    h_center = h_img[0] + int(height/2)

                    # Aplicamos el clasificador SILF
                    marker = self.compareImage(crop_img)
                    if  marker >= 0: # >=0 si ha encontrado una marca
                        print("marca encontrada: " + str(marker))
                        cv2.drawContours(canvas, [approx], -1, (0, 0, 255), 4, cv2.LINE_AA)
                        text = str("Id:" + str(marker))
                        cv2.putText(canvas, text, (w_center, h_center), 2, 1, (255, 0, 0), 2, lineType=cv2.LINE_AA)
                        corners = self.sortCorners(canvas, approx, w_center, h_center)
                        #print(corners)
                        if len(corners) == 4:
                            new_msg_marker.DetectedMarkers.append(self.makeMsgMarker(marker, corners)) 
                #else:
                #    print("not valid")
        if len(new_msg_marker.DetectedMarkers)>0:
            #print("publico -> " + str(len(new_msg_marker.DetectedMarkers)))
            new_msg_marker.header.seq = rospy.Duration()
            new_msg_marker.header.stamp = rospy.Time.now()
            new_msg_marker.header.frame_id = "camera_link"
            #print(new_msg_marker)
            self.pub_marker.publish(new_msg_marker)
            
        #cv2.imshow(self.img_canvas, canvas)
        #print(self.img_canvas)
        cv2.imshow(self.img_canvas, cv2.resize(canvas,(950,540)))
        image_message = self.bridge.cv2_to_imgmsg(canvas, encoding="passthrough")
        self.pub_projection.publish(image_message)
        n_markers = num_markers()
        n_markers.header.stamp = rospy.Time.now()
        n_markers.number = UInt8(len(new_msg_marker.DetectedMarkers))
        self.pub_num_marker.publish(n_markers)

    def compareImage(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kp, des = self.detector.detectAndCompute(img, None)
        if len(kp) > 0  and  len(des) > 20:
            print("###################################")
            print("img[" + str(img.shape[0]) + ", " + str(img.shape[1]) + "]")

            # Match descriptors.
            pos = list()
            matches = list()
            dist = list()
            count = 0
            for i in range (0, self.numMarker):
                marches_ = self.bf.match(self.markers[2][i], des)
                if len(marches_) >= self.minNumMatches: # si no hay minimo 10 coincidencias se descarta
                    matches.append(marches_)
                    matches[count] = sorted(matches[count], key = lambda x:x.distance)
                    pos.append(i)
                    dist.append(list())             
                    for m in matches[count]:
                        dist[count].append(m.distance)
                    count += 1
                #else:
                    #print("no valid matches " + str(i))

            if len(pos) > 0: # al menos tiene que haber una
                # Coincidencias.
                poseDist = list()
                for i in range (0, len(pos)):
                    poseDist.append(int(sum(dist[i][:(self.numMatches - 1)])/self.numMatches)) 
                    cncd = int (len(matches[i]) * 100) / int (len(self.markers[1][i]))
                    print("des" + str(pos[i]) + ": " + str(len(self.markers[1][i])) + "  ,des: " + str(len(kp)) + "  ,matches: " + str(len(matches[i])) + " -> " + str(cncd) + ", d(Total): " + str(int(sum(dist[i])/len(dist[i]))) + " , d(:" + str(self.numMatches) + ")-> " + str(poseDist[i]) + " , d(:19)-> " + str(int(sum(dist[i][:19])/20)) + " , d(:39)-> " + str(int(sum(dist[i][:39])/40)))
                
                poseDistShorted = sorted(poseDist)
                if (poseDistShorted[1] - poseDistShorted[0]) < self.minDist: # distancia minima entre el primero y el segundo con menos distancia
                    print("no min distance" )
                    return -1

                for j in range (0, len(pos)):
                    if poseDist[j] == poseDistShorted[0] and poseDistShorted[0] <= self.maxDist:
                        #print("***** It is marker: " + str(pos[j]))
                        img3 = cv2.drawMatches(self.markers[0][pos[j]], self.markers[1][pos[j]], img, kp, matches[j][:self.numMatches], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                        #cv2.imshow("Classifed Marker", img3) esta muestra la clasificacion
                        return pos[j]
            else:
                print("there are not valid matches")
                return -1
                #img3 = cv2.drawMatches(self.marker0,self.kp0,img,kp,matches[:60],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                #plt.imshow(img3),plt.show()
        else:
            return -1

    def sortCorners(self, img, corners, w_center, h_center):
        sorted_corner = [list(), list(), list(), list()]
        #print (corners)
        for corner in corners:
            if corner[0][0] <= w_center and corner[0][1] <= h_center: # primer cuadrante
                sorted_corner[0] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[0], -1)
                #print("First Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] <= h_center: # segundo cuadrante
                sorted_corner[1] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[1], -1)
                #print("Second Cuadrant")
            elif corner[0][0] > w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[2] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[2], -1)
                #print("Third Cuadrant")
            elif corner[0][0] <= w_center and corner[0][1] > h_center: # tercero cuadrante
                sorted_corner[3] = corner[0]
                cv2.circle(img, (corner[0][0], corner[0][1]), 5, self.colours[3], -1)
                #print("Fourth Cuadrant")
            else:
                print("Error of Cuadrant")
                return 0
        else:
            #print("Sorted corners")
            #print(sorted_corner)
            return sorted_corner

    def makeMsgMarker(self, numMarker, corners):
        #print("Make msg marker")
        #print(corners)
        new_marker = msg_marker()
        #new_marker = [corners, 0, 0, numMarker]
        for corner in corners:
            pixel = Point32()
            #print(corner)
            pixel.x = corner[0]
            pixel.y = corner[1]
            pixel.z = 0
            #print(pixel)
            new_marker.Corners.append(pixel)
        else:
            new_marker.map = UInt8(0)
            new_marker.sector = UInt8(0)
            new_marker.ID = UInt8(numMarker)
            #print(new_marker)
            return new_marker
        print("Error make marker msg")

    def cornersCloudCallback(self, cloud_data):
        for corner in cloud_data.pixels_cloud:
            #print("-- marker --")
            #print(corner.pixels_corners[0])
            cv2.circle(self.canvas_cloud, (int(corner.pixels_corners[0].x), int(corner.pixels_corners[0].y)), 3, self.colours[0], -1)
            cv2.circle(self.canvas_cloud, (int(corner.pixels_corners[1].x), int(corner.pixels_corners[1].y)), 5, self.colours[1], -1)
            cv2.circle(self.canvas_cloud, (int(corner.pixels_corners[2].x), int(corner.pixels_corners[2].y)), 5, self.colours[2], -1)
            cv2.circle(self.canvas_cloud, (int(corner.pixels_corners[3].x), int(corner.pixels_corners[3].y)), 5, self.colours[3], -1)
            """for corner in marker.pixels_corners: # they are four corners
                #print("--- corner ->" + str(corner))
                for i in range(0, 2):"""
        #cv2.imshow(self.img_point_cloud, self.canvas_cloud)






def main(args):
    rospy.init_node('detector_SILF', anonymous=True)
    image_converter(17)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)