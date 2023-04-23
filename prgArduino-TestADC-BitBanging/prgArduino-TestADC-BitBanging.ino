/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestADC-BitBanging.ino

  Description :   Programme permettant de tester l'ADC

  Remarques :     --> la mémoire SRAM et le DAC ne seront pas utilisés ici
                  --> Méthode utilisée : bit banging

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       23.04.2023

*/

// Lignes SPI de l'Arduino Nano <--> ADC
#define pin_MISO      12    // Le signal MISO entre sur la pin D12 de la carte Arduino (cette pin est reliée en interne à la broche PB4 du µC ATmega328P)
#define pin_SCK       13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)
#define pin_SS_ADC    2     // Le Slave-Select de l'ADC sort sur la pin D2 de la carte Arduino (cette pin est reliée en interne à la broche PD2 du µC ATmega328P)     

// Ligne slave-select de l'ADC (branchée sur sortie D2 de la carte Arduino, soit la pin PD2 du µC, donc le bit 2 sur PORTD[7..0])
#define selectionner_ADC        PORTD &= 0b11111011
#define desactiver_ADC          PORTD |= 0b00000100

// Ligne MISO (branchée sur sortie D12 de la carte Arduino, soit la pin PB4 du µC, donc le bit 4 sur PORTB[7..0])
#define lire_valeur_MISO        PINB & 0b00010000

// Ligne SCK (branchée sur sortie D13 de la carte Arduino, soit la pin PB5 du µC, donc le bit 5 sur PORTB[7..0])
#define mettre_SCK_a_etat_bas   PORTB &= 0b11011111
#define mettre_SCK_a_etat_haut  PORTB |= 0b00100000

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {
  pinMode(pin_SS_ADC, OUTPUT);          // Défini la broche arduino comme sortie, pilotant la broche SS de l'ADC
  digitalWrite(pin_SS_ADC, HIGH);       // ... et la fixe à l'état haut (communication SPI/ADC désactivé)

  pinMode(pin_MISO, INPUT);                                             // On ne fait rien de particulier sur la ligne MISO, mise à part la déclarer en entrée
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)
  
  Serial.begin(9600);                   // Initialise la communication série, avec le PC
  Serial.println(F("================================================================================"));
  Serial.println(F("Test de l'ADC modèle MCP 3201, de chez Microchip"));
  Serial.println(F("- Méthode de communication : bit-banging SPI, purement logiciel"));
  Serial.println(F("================================================================================"));
  Serial.println("");
}


// ===========================================================
// Fonction permettant de lire 1 bit sur le bus SPI, et de l'enregistrer dans un nombre donné, à une position donnée
// Nota : vitesse ralentie par des "NOP" pour fonctionner à 1,6 MHz max (contrainte imposée par l'ADC)
// ===========================================================
void lectureBitSPIvitesseMaxADC(uint16_t &valeurCible, uint8_t positionDuBit) {
 
  executer_NOP;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  mettre_SCK_a_etat_haut;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  if (lire_valeur_MISO)
    valeurCible |= (1 << positionDuBit);
  mettre_SCK_a_etat_bas;

  // Nota : les lignes ci-dessus correspondent grosso modo à 10 cycles d'horloge, ce qui fait, sur la base d'un µC tournant à 16 MHz, une fréquence SPI à 1,6 MHz
  //        (ce qui est la vitesse maxi de fonctionnement théorique du MCP 3201)
  
}
void lectureBitSPIvitesseMaxADC() {         // Fonction alternative, sans paramètres à passer (dans le cas où on attend rien en retour, en fait)
  executer_NOP;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  mettre_SCK_a_etat_haut;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  mettre_SCK_a_etat_bas;
}

// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
void loop() {
  uint16_t valeurADC = 0;

  selectionner_ADC;       // Active la communication avec l'ADC (en abaissant sa ligne /SS)
  
  // Remarque : le modèle d'ADC utilisé ici est un MCP3201 ; une fois la ligne slave-select abaissée, il nous faudra 15 coups d'horloge SCK pour récupérer les données
  //            (les 2 premiers coups d'horloge initialisent la communication, le 3ème coup génère un bit nul, et du 4ème au 15ème, on récupère nos 12 bits de données)

  // Premier coup d'horloge (init 1/2)
  lectureBitSPIvitesseMaxADC();

  // Second coup d'horloge (init 2/2)
  lectureBitSPIvitesseMaxADC();

  // 3ème coup d'horloge (pour faire générer un bit nul, qu'on ne vérifiera pas ici)
  lectureBitSPIvitesseMaxADC();

  // 4ème au 15ème coup d'horloge : on lit et enregistre les 12 bits qui vont se présenter sur le bus SPI
  lectureBitSPIvitesseMaxADC(valeurADC, 11);
  lectureBitSPIvitesseMaxADC(valeurADC, 10);
  lectureBitSPIvitesseMaxADC(valeurADC, 9);
  lectureBitSPIvitesseMaxADC(valeurADC, 8);
  lectureBitSPIvitesseMaxADC(valeurADC, 7);
  lectureBitSPIvitesseMaxADC(valeurADC, 6);
  lectureBitSPIvitesseMaxADC(valeurADC, 5);
  lectureBitSPIvitesseMaxADC(valeurADC, 4);
  lectureBitSPIvitesseMaxADC(valeurADC, 3);
  lectureBitSPIvitesseMaxADC(valeurADC, 2);
  lectureBitSPIvitesseMaxADC(valeurADC, 1);
  lectureBitSPIvitesseMaxADC(valeurADC, 0);
   
  desactiver_ADC;         // Désactive la communication avec l'ADC (en remontant sa ligne /SS)

  Serial.print(F("Valeur de l'ADC = "));    // Affichage du résultat sur le moniteur série de l'IDE Arduino
  Serial.println(valeurADC);
  
  delay(1000);   // Petit délai
}
