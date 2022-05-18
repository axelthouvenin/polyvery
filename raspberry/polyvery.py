# Librairies pour le serveur web et la caméra
from flask import Flask, render_template, Response, request
from camera import VideoCamera
import time
import threading
import os
# Librairies pour le pilotage des moteurs
import RPi.GPIO as GPIO
import time
import sys
# Librairies pour la transmission série
import serial

# Initialisation de la liaison série
ser = serial.Serial(
 port='/dev/ttyACM0',
 baudrate = 19200,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 bytesize=serial.EIGHTBITS,
 timeout=1
)
counter=0

# Initialisation des variables des capteurs
angle, FL, FM, FR, FU, BU, SL, SR = 0, 0, 0, 0, 0, 0, 0, 0
# Variable envoyée à l'arduino pour l'informer du déplacement en cours
commande = 0
# Variable d'autorisation pour que le robot effectue des déplacements( de base à false)
autorisation_mouvement = False 

# Déclaration de la pi_camera
pi_camera = VideoCamera(flip=False)

# Création de l'application Flask
app = Flask(__name__)

# Réccupération de la page HTML du site
@app.route('/')
def index():
    return render_template('index.html') 

# Configuration de la caméra
def gen(camera):
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

#Lancement du flux vidéo
@app.route('/video_feed')
def video_feed():
    return Response(gen(pi_camera),mimetype='multipart/x-mixed-replace; boundary=frame')

# ----- Déclaration des fonctions de commandes des moteurs ----------
#--- Cablage --------------------
MOTORA_IN1 = 20 
MOTORA_IN2 = 16 
MOTORA_ENABLE = 21

MOTORB_IN1 = 23 
MOTORB_IN2 = 24 
MOTORB_ENABLE = 18

#--- Initialisation -------------
GPIO.setmode(GPIO.BCM)
GPIO.setup( MOTORA_IN1, GPIO.OUT )
GPIO.setup( MOTORA_IN2, GPIO.OUT )
GPIO.setup( MOTORA_ENABLE, GPIO.OUT )

GPIO.setup( MOTORB_IN1, GPIO.OUT )
GPIO.setup( MOTORB_IN2, GPIO.OUT )
GPIO.setup( MOTORB_ENABLE, GPIO.OUT )

M1_Vitesse = GPIO.PWM(MOTORA_ENABLE, 100)
M2_Vitesse = GPIO.PWM(MOTORB_ENABLE, 100)
M1_Vitesse.start(45)
M2_Vitesse.start(45)

# --- Controle du L298 --------------------------
# Sens de rotation du moteur
SENS_AVANT = 2
SENS_ARRIERE = 1
SENS_ARRET = 3

def Desactiver():
    GPIO.output( MOTORA_ENABLE, GPIO.LOW )
    GPIO.output( MOTORB_ENABLE, GPIO.LOW )

def Activer():
	GPIO.output( MOTORA_ENABLE, GPIO.HIGH )
	GPIO.output( MOTORB_ENABLE, GPIO.HIGH )

def MarcheMotorA( sens ):
	if( sens == SENS_AVANT ):
		GPIO.output( MOTORA_IN1, GPIO.HIGH )
		GPIO.output( MOTORA_IN2, GPIO.LOW )
	elif( sens == SENS_ARRIERE ):
		GPIO.output( MOTORA_IN1, GPIO.LOW )
		GPIO.output( MOTORA_IN2, GPIO.HIGH )
	elif( sens == SENS_ARRET ):
		GPIO.output( MOTORA_IN1, GPIO.LOW )
		GPIO.output( MOTORA_IN2, GPIO.LOW )

def MarcheMotorB( sens ):
	if( sens == SENS_AVANT ):
		GPIO.output( MOTORB_IN1, GPIO.HIGH )
		GPIO.output( MOTORB_IN2, GPIO.LOW )
	elif( sens == SENS_ARRIERE ):
		GPIO.output( MOTORB_IN1, GPIO.LOW )
		GPIO.output( MOTORB_IN2, GPIO.HIGH )
	elif( sens == SENS_ARRET ):
		GPIO.output( MOTORB_IN1, GPIO.LOW )
		GPIO.output( MOTORB_IN2, GPIO.LOW ) 

#---- Fonctions de pilotage et d'autorisation des déplacements
@app.route('/Avancer')
def Avancer():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        Desactiver()
        MarcheMotorA( SENS_AVANT )
        MarcheMotorB( SENS_AVANT )
        Activer()
        return "1"

@app.route('/Reculer')
def Reculer():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        Desactiver()
        MarcheMotorA( SENS_ARRIERE )
        MarcheMotorB( SENS_ARRIERE )
        Activer()
        return "1"

@app.route('/Droite')
def Droite():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        Desactiver()
        MarcheMotorA( SENS_ARRIERE )
        MarcheMotorB( SENS_AVANT )
        Activer()
        print("droite")
        return "1"

@app.route('/Gauche')
def Gauche():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        Desactiver()
        MarcheMotorA( SENS_AVANT )
        MarcheMotorB( SENS_ARRIERE )
        Activer()
        return "1"
		
@app.route('/Stop')
def Stop():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        Desactiver()
        MarcheMotorA( SENS_ARRET )
        MarcheMotorB( SENS_ARRET )
        Activer()
        return "1"

# a refaire et valider
@app.route('/Droite90')
def Droite90():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        angle = LireAngle()
        cible = angle - 90
        Droite()
        while angle> cible:
            angle = LireAngle()
        print("droite90")
        return "1"

# à refaire et valider
@app.route('/Gauche90')
def Gauche90():
    global autorisation_mouvement
    if autorisation_mouvement == True:
        angle = LireAngle()
        cible = angle + 90
        Gauche()
        while angle < cible:
            angle = LireAngle()
        print("gauche90")
        return "1"
        
# à refaire et valider
@app.route('/DemiTour')
def DemiTour():
    return "1"

@app.route('/Immobile')
def Immobile():
    global autorisation_mouvement
    autorisation_mouvement = not autorisation_mouvement
    if autorisation_mouvement==True:
        print("Le robot peut se déplacer")
    else:
        print("Le robot ne peut pas se déplacer")
    return "1"

# déplacement en cas d'obstacle rencontré (à refaire et valider)
def EviterObstacle():
    while 1:
        global FL, FM, FR, FU, BU, SL, SR, angle
        # mur devant
        if  FM == 1 :
            Reculer()
            while FM == 1 and BU ==2:
                pass
            Stop()
        # mur avant droite
        if FR == 1 :
            Reculer()
            while FR ==1 and BU == 2:
                pass
            Stop()
        # vide devant
        elif FU == 1:
            Reculer()
            while FU == -1 and BU ==2:
                pass
            Stop()
        #vide arriere
        elif BU == 1:
            Avancer()
            while BU ==1 and FM!=1 and FU ==2:
                pass
            Stop()
            
        
            
            
# -----------------------------------------------------------------
# Fonction de lecture du port série
def LirePortSerie():
    while 1:
        trame=ser.readline()
        global commande
        ser.write("b'commande="+str(commande)+";'")  
        str_trame = str(trame)
        # C'est une trame d'angle et de capteurs US
        if "angle" in str_trame and "FM" in str_trame and "FL" in str_trame and "FR" in str_trame and "FU" in str_trame and "BU" in str_trame :
            print(str_trame)
            # Reccupération des informations des capteurs US
            global FL, FM, FR, FU, BU, SL, angle
            FL = RecupVal(str_trame,str_trame.find("FL=")+3,str_trame.find(";FM"))
            FM = RecupVal(str_trame,str_trame.find("FM=")+3,str_trame.find(";FR"))
            FR = RecupVal(str_trame,str_trame.find("FR=")+3,str_trame.find(";SL"))
            SL = RecupVal(str_trame,str_trame.find("SL=")+3,str_trame.find(";FU"))
            FU = RecupVal(str_trame,str_trame.find("FU=")+3,str_trame.find(";BU"))
            BU = RecupVal(str_trame,str_trame.find("BU=")+3,str_trame.find(";angle"))
            # Reccupération de l'angle du capteur boussole
            angle = RecupVal(str_trame,str_trame.find("angle=")+6,str_trame.find("\r\n")-4)
        # c'est une trame de niveau de batterie
        elif "batterie" in str_trame :
            print("batterie")
        # c'est une trame inconnue ou erronée
        else:
            print("erreur trame reçue")
            
# Fonction pour récupérer une valeur entre 2 positions dans une trame
def RecupVal(trame,pos1,pos2):
    if "-" in trame[pos1:pos2]:
        val = float(trame[pos1+1:pos2])*-1
    else:
        val = float(trame[pos1:pos2])         
    return val
        
#Fonction de démarrage du serveur web
def DemarrageServWeb():
    if __name__ == '__main__':
        app.run(host='0.0.0.0',port="8000",debug=False)

# Création et lancement du thread de lecture du port série et serveur web
thread_port_serie = threading.Thread(target = LirePortSerie)
thread_serveur = threading.Thread(target = DemarrageServWeb)
thread_evitement = threading.Thread(target = EviterObstacle)
thread_port_serie.start()
thread_serveur.start()
thread_evitement.start()

# En cas de fin d'un des thread
#if

    


