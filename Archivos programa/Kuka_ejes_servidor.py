#!/usr/bin/env python
 
from __future__ import print_function
 
from suscriptor.srv import Ejeskuka, EjeskukaResponse, EntradasKuka, EntradasKukaResponse
import rospy

import sys
import struct
import random
import socket

__version__ = '1.1.7'
ENCODING = 'UTF-8'

PY2 = sys.version_info[0] == 2
#if PY2: input = raw_input

class openshowvar(object):
    # Inicializacion
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.msg_id = random.randint(1, 100)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((self.ip, self.port))
        except socket.error:
            pass

    # Conexion
    def test_connection(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            ret = sock.connect_ex((self.ip, self.port))
            return ret == 0
        except socket.error:
            print('socket error')
            return False

    can_connect = property(test_connection)

    def read(self, var, debug=True):#lectura de variable
        if not isinstance(var, str):#Verifica que la variable es del tipo especificado
            raise Exception('Var name is a string')
        else:
            self.varname = var if PY2 else var.encode(ENCODING)
        return self._read_var(debug)

    def write(self, var, value, debug=True):#Escritura de variable
        if not (isinstance(var, str) and isinstance(value, str)):#Verifica que las variables son del tipo especificado
            raise Exception('Var name and its value should be string')
        print(var, value)
        self.varname = var if PY2 else var.encode(ENCODING)
        self.value = value if PY2 else value.encode(ENCODING)
        return self._write_var(debug)

    # Leer variable
    def _read_var(self, debug):
        req = self._pack_read_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print(_value)
        return _value

    # Escritura de variable
    def _write_var(self, debug):
        req = self._pack_write_req()
        self._send_req(req)
        _value = self._read_rsp(debug)
        if debug:
            print(_value)
        return _value

    # Solicitud de envio
    def _send_req(self, req):
        self.rsp = None
        self.sock.sendall(req)
        self.rsp = self.sock.recv(256)

    # Solicitud del paquete de lectura
    def _pack_read_req(self):
        var_name_len = len(self.varname)
        flag = 0
        req_len = var_name_len + 3
        # Estructura el paquete de los datos
        return struct.pack(
            '!HHBH'+str(var_name_len)+'s',
            self.msg_id,
            req_len,
            flag,
            var_name_len,
            self.varname
            )

    # Solicitud del paquete de escritura
    def _pack_write_req(self):
        var_name_len = len(self.varname)
        flag = 1
        value_len = len(self.value)
        req_len = var_name_len + 3 + 2 + value_len
        # Estructura los datos
        return struct.pack(
            '!HHBH'+str(var_name_len)+'s'+'H'+str(value_len)+'s',
            self.msg_id,
            req_len,
            flag,
            var_name_len,
            self.varname,
            value_len,
            self.value
            )

    # Respuesta
    def _read_rsp(self, debug=False):
        if self.rsp is None: return None
        var_value_len = len(self.rsp) - struct.calcsize('!HHBH') - 3
        result = struct.unpack('!HHBH'+str(var_value_len)+'s'+'3s', self.rsp)#Estructura de la trama
        _msg_id, body_len, flag, var_value_len, var_value, isok = result
        if debug:
            print('[DEBUG]', result)
        if result[-1].endswith(b'\x01') and _msg_id == self.msg_id:
            self.msg_id = (self.msg_id + 1) % 65536  # El formato char 'H' tiene 2 bytes de longitud
            return var_value

    def close(self):#Cerrar la aplicacion
        self.sock.close()


############### test ###############
def mover_kuka(req):
    while True:
        nombre = str(req.Nombre)
        x = str(req.x)
        y = str(req.y)
        z = str(req.z)
        a = str(req.a)
        b = str(req.b)
        c = str(req.c)
        tip = str(req.tipo)
        tip_nom= "TIPO"
        ejes = "{E6POS: X " + x +", Y "+ y +", Z "+ z +", A "+ a +", B "+ b +", C "+ c + "}"
        
        #print(ejes)
        if nombre == 'q':#Si la respuesta es una "q" se cerrara la aplicacion
            print('Bye')
            client.close()
            break
        else:
            #parts = data.split(',')#Sino separa la informacion entregada mediante una coma
            #pos = len(parts[0])+1
            if len(ejes) == 1:#En caso de solo tener un valor (nombre de la variable), se hara una lectura del valor de esta
                client.read(nombre.strip(), True)
                                   
            else:#En caso de tener mas vaolores, el priero se usara como nombre de la variable y el segundo como valor que se le quiera dar a este
                client.write(nombre, ejes, True)
                client.write(tip_nom, tip, True)
                return EjeskukaResponse(True)
    print("Returning " , req.Nombre, req.x, req.y, req.z, req.a, req.b, req.c, req.tip) 
    return EjeskukaResponse(True)

def entrada_kuka(req):
    nombre = str(req.Nombre)
    numero = str(req.Numero)
    entrada = str(req.entrada)    
    nom_num = "NUM_SALIDA"
    client.write(nombre, entrada, True)
    client.write(nom_num, numero, True)
    print("Returning " , req.Nombre, req.entrada)
    resp = EntradasKukaResponse(True)
    return resp
 
def ejes_kuka_server():
     rospy.init_node('Kuka_server')
     s = rospy.Service('ejes_kuka', Ejeskuka, mover_kuka)
     s_ = rospy.Service('entradas_kuka', EntradasKuka, entrada_kuka)
     print("Preparado para recibir órdenes")
     rospy.spin()
    


if __name__ == '__main__':
    ip = "10.172.21.202"#Pide la IP
    port = 7000#Pide el Puerto
    client = openshowvar(ip, port)
    if not client.can_connect:
        print('Connection error')
        import sys
        sys.exit(-1)
    print('\nConnected KRC Name: ', end=' ')#Muestra si se ha establecido correctamente la conexion
    client.read('$ROBNAME[]', False)#Lee el nombre del Robot
    ejes_kuka_server()

