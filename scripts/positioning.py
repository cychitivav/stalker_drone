#?/usr/bin/python

import cv2
import cv_bridge
import rospy
import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from numpy.lib.function_base import extract
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge,CvBridgeError

class processing:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/proccesed_camera', Image, queue_size=10)

        self.sub = rospy.Subscriber('/image_raw', Image, self.get_distance_cb)

    def get_distance_cb(self, data):
        enc = data.encoding
        try:
            frame = self.bridge.imgmsg_to_cv2(data,enc)
        except CvBridgeError as e:
            rospy.logerr(e)

        p,boxes=self.found_polyg(frame)
        dis=0
        f=629 #889 cm
        w=20 #cm
        if p>0:
            dis = (w*f)/p

        if dis<400:
            msj = "distancia: "+str(round(dis,2)) + " cm"

        cv2.putText(frame,msj,(10,70),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),2,cv2.LINE_AA)

        cent_x,cent_y = self.centroid(boxes)
        cen_img_x, cen_img_y = frame.shape[0]/2,frame.shape[1]/2
        angle = 45
        angle2 = 20
        sep_x, sep_y= (cent_x-cen_img_x), (cent_y-cen_img_y)

        angx, angy= sep_x*angle*0.5/cen_img_x , sep_y*angle2/cen_img_y
        try:
            self.pub.publish(self.bridge.cv2_to_imgmsg(frame,enc))
        except CvBridgeError as e:
            rospy.logerr(e)

    def centroid(self,*points):
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        _len = len(points)
        centroid_x = sum(x_coords)/_len
        centroid_y = sum(y_coords)/_len
        return [centroid_x, centroid_y]



    #---------------------------------FUNCIONES
    def area_cont(self,cnt):    #Filtra los contornos por area en pixeles, para eliminar contornos de ruido (se suponen menores al area de la señal)
        contornos = []     #En caso de surgir otros tipos de ruido filtrar tambien el numero de lados (recordando un limite para evitar borrar los circulos)
        for c in range(len(cnt)):
            area = cv2.contourArea(cnt[c]) 
            if area > 900:
                contornos.append(cnt[c])
        
        return contornos

    #Buscar una relacion entre resolucion (ej 1920x1080) y pixeles de area minima, ya que con iamgenes de menor calidad esto
    #puede variar (el area minima cambia)

    def extract_green(self,hsv):
        #amarillos
        hMin = 62
        hMax = 120
        sMin = 70
        sMax = 255
        vMin = 60
        vMax = 255
        color_bajos1=np.array([hMin,sMin,vMin]) 
        color_altos1=np.array([hMax,sMax,vMax])

        #Detectamos los colores
        mask = cv2.inRange(hsv, color_bajos1, color_altos1)
        
        kernel = np.ones((3,3),np.uint8)

        mask = cv2.erode(mask, None, iterations=5)
        mask = cv2.dilate(mask, None, iterations=5)
        #mask = cv2.morphologyEx(,cv2.MORPH_CLOSE,kernel)
        #mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        

        return mask

    def extrac(self,hsv): #Recibe la imagen HSV y devuelve la imagen binarizada con las regiones más cercanas a ser
            #masks=[]
            #masks.append(extract_black(hsv))
            #masks.append(extract_white(hsv))
            masks=extract_green(hsv)

            return masks


    #Se hallan los contornos
    def getContours(self,img):
        edged = cv2.Canny(img, 30, 200) 
        cv2.imshow("canny",edged)
        binary,contour, hierarchy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return area_cont(contour)  #Entrega los contornos que superen el area minima


    def resizeimg(self,img):

        heig = img.shape[0]
        wid = img.shape[1]
        return cv2.resize(img, (wid//2,heig//2),interpolation=cv2.INTER_CUBIC) 


    def found_polyg(self,frame):
        frame2 = self.extrac(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        
        contornos = self.getContours(frame2)
        #cv2.drawContours(frame, contornos, -1, (0,255,0), 3)

        i=0
        p=0
        boxes=None
        for c in contornos:
            epsilon = 0.1*cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c,epsilon,True)
            nlados = len(approx)
            if nlados>=4:
                #x,y,w,h = cv2.boundingRect(approx)
                #cv2.drawContours(frame,[approx], 0, (0,255,0),4)
                medidas=cv2.minAreaRect(c)
                box=cv2.boxPoints(medidas)
                box = np.int0(box)
                cv2.drawContours(frame,[box],0,(0,0,255),2)
                
                if medidas[1][0]>p:
                    boxes=box
                    p=medidas[1][0]
            i=i+1
            #medidas = cv2.minAreaRect(c)
            # cv2.putText(frame,'Lados'+str(len(approx)), (x,y-5),1,1.5,(0,255,0),2)

        return p, boxes

    










