/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestADC-LibrairieSPI.ino

  Description :   Programme permettant de tester l'ADC

  Remarques :     --> la mémoire SRAM et le DAC ne seront pas utilisés ici
                  --> Méthode utilisée : utlilsation de la librairie SPI.h

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       23.04.2023

*/

#include <SPI.h>

#define brocheSSdeLadc  2

void setup() {
  pinMode(brocheSSdeLadc, OUTPUT);        // Défini la broche arduino comme sortie, pilotant la broche SS de l'ADC
  digitalWrite(brocheSSdeLadc, HIGH);     // ... et la fixe à l'état haut (communication SPI/ADC désactivé)
  
  SPI.begin();                            // Initialise la librairie SPI
  SPI.setClockDivider(SPI_CLOCK_DIV16);   // On divise l'horloge 16 MHz du µC par 16, pour fonctionner à 1 MHz (cet ADC étant limité à 1,6 MHz environ)
  
  Serial.begin(9600);                     // Initialise la communication série, avec le PC
  Serial.println(F("================================================================================"));
  Serial.println(F("Test de l'ADC modèle MCP 3201, de chez Microchip"));
  Serial.println(F("- Méthode de communication : librairie SPI.h"));
  Serial.println(F("================================================================================"));
  Serial.println("");
}

void loop() {
  uint16_t valeurADC = 0;
  uint8_t valeurHaute = 0;
  uint8_t valeurBasse = 0;

  digitalWrite(brocheSSdeLadc, LOW);    // Active la communication avec l'ADC (en abaissant sa ligne /SS)
  
  valeurHaute = SPI.transfer(0xFF);     // Lit les 8 premiers bits (HiZ, HiZ, 0, B11, B10, B9, B8, B7)
  valeurBasse = SPI.transfer(0xFF);     // Lit les 8 bits suivants (B6, B5, B4, B3, B2, B1, B0, B1)

  valeurADC = (valeurHaute << 8) | valeurBasse;
  
  valeurADC = valeurADC >> 1;                   // On efface le dernier bit (B1), en décalant le tout "vers la droite"
  valeurADC &= valeurADC &0b0000111111111111;   // On garde que les 12 derniers bits, en "effaçant" les 4 premiers
  
  digitalWrite(brocheSSdeLadc, HIGH);   // Désactive la communication avec l'ADC (en remontant sa ligne /SS)

  Serial.print(F("Valeur de l'ADC = "));
  Serial.println(valeurADC);
  delay(1000);   // Petit délai
}
