/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestDAC-BitBanging.ino

  Description :   Programme permettant de tester l'DAC

  Remarques :     --> la mémoire SRAM et l'ADC ne seront pas utilisés ici
                  --> Méthode utilisée : bit banging

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       24.04.2023

*/

// Lignes SPI de l'Arduino Nano <--> DAC
#define pin_MOSI          11    // Le signal MOSI sort sur la pin D11 de la carte Arduino (cette pin est reliée en interne à la broche PB3 du µC ATmega328P)
#define pin_SCK           13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)
#define pin_SS_DAC        3     // Le Slave-Select du DAC sort sur la pin D3 de la carte Arduino (cette pin est reliée en interne à la broche PD3 du µC ATmega328P)
#define pin_POTENTIOMETRE A0    // La tension présente sur A0, modifiable par potentiomètre, sera envoyée au DAC, pour "reproduction"

// Ligne slave-select du DAC (branchée sur sortie D3 de la carte Arduino, soit la pin PD3 du µC, donc le bit 3 sur PORTD[7..0])
#define selectionner_DAC        PORTD &= 0b11110111
#define desactiver_DAC          PORTD |= 0b00001000

// Ligne MOSI (branchée sur sortie D11 de la carte Arduino, soit la pin PB3 du µC, donc le bit 3 sur PORTB[7..0])
#define mettre_MOSI_a_etat_bas  PORTB &= 0b11110111
#define mettre_MOSI_a_etat_haut PORTB |= 0b00001000

// Ligne SCK (branchée sur sortie D13 de la carte Arduino, soit la pin PB5 du µC, donc le bit 5 sur PORTB[7..0])
#define mettre_SCK_a_etat_bas   PORTB &= 0b11011111
#define mettre_SCK_a_etat_haut  PORTB |= 0b00100000

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {
  
  // Désactivation de la communication SPI/ADC, pour commencer, au démarrage
  pinMode(pin_SS_DAC, OUTPUT);          // Défini la broche arduino comme sortie, pilotant la broche SS du DAC
  digitalWrite(pin_SS_DAC, HIGH);       // ... et la fixe à l'état haut (communication SPI/DAC désactivé)

  // Mise au repos du bus SPI
  pinMode(pin_MOSI, OUTPUT);      digitalWrite(pin_MOSI, LOW);          // Mise à l'état bas de la ligne MOSI (ligne de données maitre vers esclave)
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)
  
  Serial.begin(9600);                   // Initialise la communication série, avec le PC
  Serial.println(F("================================================================================"));
  Serial.println(F("Test du DAC modèle MCP 4921, de chez Microchip"));
  Serial.println(F("- Méthode de communication : bit-banging SPI, purement logiciel"));
  Serial.println(F("================================================================================"));
  Serial.println("");
  
}


// ===========================================================
// Fonction permettant d'écrire 1 bit sur le bus SPI, à la vitesse maximale du µC
// ===========================================================
void ecritureBitSPIvitesseMaximale(uint16_t valeur) {
  valeur == 0 ? mettre_MOSI_a_etat_bas : mettre_MOSI_a_etat_haut; 
  mettre_SCK_a_etat_haut;
  mettre_SCK_a_etat_bas;

  // Nota : à 16 MHz, chaque cycle du µC dure 62,5 ns
  //       (bien qu'on soit à la vitesse maximale, c'est encore suffisamment "lent" pour communiquer avec le DAC ou la SRAM, à plein régime !)

}


// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
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
  
  // Envoi des données au DAC
    // Ecriture des 4 bits de configuration du DAC
  ecritureBitSPIvitesseMaximale(0);     // 1er bit de config : toujours à 0
  ecritureBitSPIvitesseMaximale(0);     // 2ème bit de config : 0=unbuffered, 1=buffered (ici choix unbuffured, donc 0)
  ecritureBitSPIvitesseMaximale(1);     // 3ème bit de config : 0=gain double, 1=pas de gain (ici choix "pas de gain", donc 1)
  ecritureBitSPIvitesseMaximale(1);     // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active (ici choix "sortie active", donc 1)

    // Ecriture des 12 bits de données, préalablement lues par l'ADC
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 11));    // Bit 11
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 10));    // Bit 10
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 9));     // Bit 9
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 8));     // Bit 8
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 7));     // Bit 7
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 6));     // Bit 6
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 5));     // Bit 5
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 4));     // Bit 4
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 3));     // Bit 3
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 2));     // Bit 2
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 1));     // Bit 1
  ecritureBitSPIvitesseMaximale(valeurPOT & (1 << 0));     // Bit 0


  // Désactivation de la communication avec le DAC (en remontant sa ligne /SS)
  desactiver_DAC;

  delay(1000);   // Petit délai
}
