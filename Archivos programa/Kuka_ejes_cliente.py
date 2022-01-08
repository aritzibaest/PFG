#!/usr/bin/env python
 
from __future__ import print_function
 
import sys
import rospy
from suscriptor.srv import Ejeskuka, EntradasKuka

def entrada_kuka_client(nom, numero, entrada):
     rospy.wait_for_service('entradas_kuka')
     entradas = rospy.ServiceProxy('entradas_kuka', EntradasKuka)
     try:
         resp_ = entradas.call(nom, numero, entrada)         
         return resp_.salida
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)

def ejes_kuka_client(nom, x, y, z, a, b, c, tip):
     rospy.wait_for_service('ejes_kuka')
     ejes = rospy.ServiceProxy('ejes_kuka', Ejeskuka)
     try:
         if tip == 3:
                data = input('\nPunto final del movimiento circular: ')
                parts = data.split(' ')
                cir = str("CIR")
                x_ =float(parts[0])
                y_ =float(parts[1])
                z_ =float(parts[2])
                a_ =float(parts[3])
                b_ =float(parts[4])
                c_ =float(parts[5])
                resp1 = ejes(nom, x, y, z, a, b, c, tip)
                resp2 = ejes(cir, x_, y_, z_, a_, b_, c_, tip)
                return resp2.salida
         resp1 = ejes(nom, x, y, z, a, b, c, tip)
         print(resp1.salida)
         return resp1.salida
     except rospy.ServiceException as e:
         print("Service call failed: %s"%e)
 
def usage():
     print("El formato a utilizar es el siguiente")
     return "%s Nombre_de_variable x y z a b c Tipo_movimiento(1(Lineal), 2(PTP), 3(Circular)"%sys.argv[0]
 
if __name__ == "__main__":
     if len(sys.argv) == 9:
         nom = str(sys.argv[1]) #ARR
         x = float(sys.argv[2])
         y = float(sys.argv[3])
         z = float(sys.argv[4])
         a = float(sys.argv[5])
         b = float(sys.argv[6])
         c = float(sys.argv[7])
         tip = float(sys.argv[8])
         ejes_kuka_client(nom, x, y, z, a, b, c, tip)
         quit()
     if len(sys.argv) == 4:
         nom_ = str(sys.argv[1])#ENTRADA
         numero = int(sys.argv[2])
         if sys.argv[3].lower()=='true': entrada_ = True
         elif sys.argv[3].lower()=='false': entrada_ = False
        #  entrada_ = bool(sys.argv[2])
         
         entrada_kuka_client(nom_ , numero, entrada_ )
     else:
         print(usage())
         quit()
         sys.exit(1)
    
# E6POS:  959.32 -227.74 1332.81 152.62 -0.34 -179.93  
# 359290.0 -125790.0 1227.06