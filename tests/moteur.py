
import RPi.GPIO as GPIO
import time
import sys

GPIO.setmode(GPIO.BCM)

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

# --- Controle du L298 --------------------------
# Sens de rotation du moteur
SENS_AVANT = 1
SENS_ARRIERE = 2
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

def Avant():
	Desactiver()
	MarcheMotorA( SENS_AVANT )
	MarcheMotorB( SENS_AVANT )
	Activer()

def Arriere():
	Desactiver()
	MarcheMotorA( SENS_ARRIERE )
	MarcheMotorB( SENS_ARRIERE )
	Activer()

def TournerDroite():
	Desactiver()
	MarcheMotorA( SENS_ARRIERE )
	MarcheMotorB( SENS_AVANT )
	Activer()

def TournerGauche():
	Desactiver()
	MarcheMotorA( SENS_AVANT )
	MarcheMotorB( SENS_ARRIERE )
	Activer()
		
def Stop():
	Desactiver()
	MarcheMotorA( SENS_ARRET )
	MarcheMotorB( SENS_ARRET )
	Activer()

while True :

    Activer()
    MarcheMotorA(1)
