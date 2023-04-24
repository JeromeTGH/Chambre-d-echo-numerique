/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestDAC-LibrairieSPI.ino

  Description :   Programme permettant de tester le DAC

  Remarques :     --> la mémoire SRAM et l'ADC ne seront pas utilisés ici
                  --> Méthode utilisée : utlilsation de la librairie SPI.h

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       24.04.2023

*/

#include <SPI.h>

#define brocheSSduDAC     3     // Pour activer la communication SPI avec le DAC (actif au niveau bas)
#define pin_POTENTIOMETRE A0    // La tension présente sur A0, modifiable par potentiomètre, sera envoyée au DAC, pour "reproduction"

// Ligne slave-select du DAC (branchée sur sortie D3 de la carte Arduino, soit la pin PD3 du µC, donc le bit 3 sur PORTD[7..0])
#define selectionner_DAC        PORTD &= 0b11110111
#define desactiver_DAC          PORTD |= 0b00001000

void setup() {
  pinMode(brocheSSduDAC, OUTPUT);         // Défini la broche arduino comme sortie, pilotant la broche SS du DAC
  digitalWrite(brocheSSduDAC, HIGH);      // ... et la fixe à l'état haut (communication SPI/DAC désactivée)
  
  SPI.begin();                            // Initialise la librairie SPI
  SPI.setClockDivider(SPI_CLOCK_DIV2);    // On divise l'horloge 16 MHz du µC par 2, pour fonctionner à 8 MHz (ce DAC pouvant fonctionner jusqu'à 20 MHz)
  
  Serial.begin(9600);                     // Initialise la communication série, avec le PC
  Serial.println(F("================================================================================"));
  Serial.println(F("Test du DAC modèle MCP 4921, de chez Microchip"));
  Serial.println(F("- Méthode de communication : librairie SPI.h"));
  Serial.println(F("================================================================================"));
  Serial.println("");
}

void loop() {

  uint16_t valeurPOT = 0;

  // Lecture valeur d'entrée arduino, sur 10 bits (0..1023), et conversion en valeur 12 bits (0..4095)
  valeurPOT = analogRead(pin_POTENTIOMETRE);
  valeurPOT = map(valeurPOT, 0, 1023, 0, 4095);

  // Affichage de cette valeur sur le moniteur série
  Serial.print(F("Valeur de l'entrée A0 (potentiomètre) = "));    // Affichage sur le moniteur série de l'IDE Arduino
  Serial.println(valeurPOT);

  // Activation de la communication avec le DAC (en abaissant sa ligne /SS)
  selectionner_DAC;

  // Ajout de 4 bits de configuration du DAC, "à gauche" des 12 bits de données
      // 1er bit de config : toujours à 0
      // 2ème bit de config : 0=unbuffered, 1=buffered (ici choix unbuffured, donc 0)
      // 3ème bit de config : 0=gain double, 1=pas de gain (ici choix "pas de gain", donc 1)
      // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active (ici choix "sortie du DAC active", donc 1)
      // ... d'où le masque "0011" qui sera rajouté "devant" les 12 bits de poids faible (conservés avec les "111111111111" qui suivent)
  valeurPOT |= 0b0011000000000000;      // On rajoute donc "0011" devant

  // Transmission des données
  SPI.transfer((uint8_t)(valeurPOT >> 8));                    // Transfert des 8 bits de "gauche", en premier
  SPI.transfer((uint8_t)(valeurPOT & 0b0000000011111111));    // Puis transfert des 8 bits de "droite", ensuite

  // Désactivation de la communication avec le DAC (en remontant sa ligne /SS)
  desactiver_DAC;
 
  delay(1000);   // Petit délai
}
