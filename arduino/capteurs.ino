#include <HCSR04.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <HMC5883L.h>
#include <math.h>

// Initialisation des pins des capteurs (trig pin, echo pin)
UltraSonicDistanceSensor CU_Front_Mid(5, 4);  
UltraSonicDistanceSensor CU_Front_Left(3,2);
UltraSonicDistanceSensor CU_Front_Right(7,6);

// Variables pour les capteurs Ultra Son
// F pour FRONT; L pour LEFT; M pour MIDDLE; R pour RIGHT
int dist_FL, dist_FM, dist_FR;

// Variables pour le capteur boussole HMC5883L
HMC5883L capteur;
 // déclaration des variables sur les axes x, y, z
int16_t mx, my, mz;
float angle;

void setup () {
   // initialisation de la liaison série I2C
  Wire.begin();
  // Initialisation de la liaison série
  Serial.begin(19200);  
  while (!Serial) {
  }

   // initialisation du capteur HMC5883L
  //capteur.initialize();
  delay(1000);
  /*while (!capteur.testConnection()) {
    Serial.println("erreur connexion capteur HMC5883L..");
    delay(500);
  }*/
}

void loop () {
  // Lecture des distances des capteurs US
  dist_FL = CU_Front_Left.measureDistanceCm();
  dist_FM = CU_Front_Mid.measureDistanceCm();
  dist_FR = CU_Front_Right.measureDistanceCm();  

  // lire les données sur les axes x,y,z du champs magnétique
  //capteur.getHeading(&mx, &my, &mz);
  // calcul de l'angle en degrés par rapport au nord
  //angle = atan2((double)mx, (double)my);
  //angle = angle * 180 / PI;
  envoi_mesures(dist_FL,dist_FM,dist_FR,0);
  // Délais, 10 microsecondes est le min des capteurs US
  delay(50);
    
}


// Fonction qui recoit les données mesurées et les envoie sur le bus série sous forme d'une trame
// Structure de la trame :
void envoi_mesures(int dist_FL,int dist_FM,int dist_FR, float angle){
  String trame ="";
  // Capteur avant gauche
  if(dist_FL>15){
     trame += "FL=3;";
  }
  else if(dist_FL<=15){
    if(dist_FL==-1){
      trame += "FL=-1;";
    }
    else if(dist_FL<10){
      trame += "FL=1;";
    }
    else{
      trame += "FL=2;";
    }
  }
  else{
    trame += "FL=0;";
  }
  // Capteur avant milieu
  if(dist_FM>15){
     trame += "FM=3;";
  }
  else if(dist_FM<=15){
    if(dist_FM==-1){
      trame += "FM=-1;";
    }
    else if(dist_FM<10){
      trame += "FM=1;";
    }
    else{
      trame += "FM=2;";
    }
  }
  else{
    trame += "FM=0;";
  }

  // Capteur avant droit
  if(dist_FR>15){
     trame += "FR=3;";
  }
  else if(dist_FR<=15){
    if(dist_FR==-1){
      trame += "FR=-1;";
    }
    else if(dist_FR<10){
      trame += "FR=1;";
    }
    else{
      trame += "FR=2;";
    }
  }
  else{
    trame += "FR=0;";
  }
  Serial.println(trame+"angle:"+angle);
}
