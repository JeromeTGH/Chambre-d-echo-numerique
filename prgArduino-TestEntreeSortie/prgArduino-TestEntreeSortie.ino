/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestEntreeSortie.ino

  Description :   Programme permettant renvoyant les données lues en entrées (sur l'ADC)
                  directement en sortie (sur le DAC)

  Remarques :     --> la mémoire SRAM ne sera donc pas utilisée ici
                  --> l'Arduino utilisé ici est un "Nano" (donc un modèle équipé du microcontrôleur ATmega328P)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2023

*/

// ===========================================================
// Définition des raccordements entre périphériques et bus SPI
// ===========================================================
#define pin_MOSI      11    // Le signal MOSI sort sur la pin D11 de la carte Arduino (cette pin est reliée en interne à la broche PB3 du µC ATmega328P)
#define pin_MISO      12    // Le signal MISO entre sur la pin D12 de la carte Arduino (cette pin est reliée en interne à la broche PB4 du µC ATmega328P)
#define pin_SCK       13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)

#define pin_SS_ADC    2     // Le Slave-Select de l'ADC sort sur la pin D2 de la carte Arduino (cette pin est reliée en interne à la broche PD2 du µC ATmega328P)
#define pin_SS_DAC    3     // Le Slave-Select du DAC sort sur la pin D3 de la carte Arduino (cette pin est reliée en interne à la broche PD3 du µC ATmega328P)
#define pin_SS_SRAM   4     // Le Slave-Select de la SRAM sort sur la pin D4 de la carte Arduino (cette pin est reliée en interne à la broche PD4 du µC ATmega328P)

// ===========================================================
// Fonctions raccourcies
// ===========================================================

// Ligne slave-select de l'ADC (branchée sur sortie D2 = pin PORTD.2 du µC)
#define selectionner_ADC        PORTD &= 0b11111011
#define desactiver_ADC          PORTD |= 0b00000100

// Ligne slave-select du DAC (branchée sur sortie D3 = pin PORTD.3 du µC)
#define selectionner_DAC        PORTD &= 0b11110111
#define desactiver_DAC          PORTD |= 0b00001000

// Ligne slave-select de la SRAM (branchée sur sortie D4 = pin PORTD.4 du µC)
#define selectionner_SRAM       PORTD &= 0b11101111
#define desactiver_SRAM         PORTD |= 0b00010000

// Ligne MOSI (branchée sur sortie D11 = pin PORTB.3 du µC)
#define mettre_MOSI_a_etat_bas  PORTB &= 0b11110111
#define mettre_MOSI_a_etat_haut PORTB |= 0b00001000

// Ligne MISO (branchée sur sortie D12 = pin PORTB.4 du µC)
#define lire_valeur_MISO        PORTB & 0b00010000

// Ligne SCK (branchée sur sortie D13 = pin PORTB.5 du µC)
#define mettre_SCK_a_etat_bas   PORTB &= 0b11011111
#define mettre_SCK_a_etat_haut  PORTB |= 0b00100000

// Fonction "NOP" (pour faire faire une pause d'un cycle au µC, soit 62,5 ns, de base)
#define executer_NOP asm volatile ("nop\n\t")

// ===========================================================
// CONSTANTES du programme
// ===========================================================
#define VITESSE_ECHANTILLONNAGE   44100   // De base, on échantillonnera le son à 44100 Hz ("qualité CD", en fait)
#define VALEUR_MAXI_TIMER1        65535   // Le "Timer 1", dont nous allons nous servir, fait 16 bits ; le compteur compte donc de 0 à 65535
#define VALEUR_DEMARRAGE_TIMER1   VALEUR_MAXI_TIMER1 - (F_CPU / VITESSE_ECHANTILLONNAGE)  // En sachant que F_CPU est une valeur déjà définie (valant 16000000, soit 16 MHz)


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {

  // Désactivation de tous les périphériques présents sur le bus SPI, pour commencer
  pinMode(pin_SS_ADC, OUTPUT);    digitalWrite(pin_SS_ADC, HIGH);       // Les lignes "slave select" sont actives à l'état bas, donc désactivées à l'état haut
  pinMode(pin_SS_DAC, OUTPUT);    digitalWrite(pin_SS_DAC, HIGH); 
  pinMode(pin_SS_SRAM, OUTPUT);   digitalWrite(pin_SS_SRAM, HIGH); 

  // Mise au repos du bus SPI
  pinMode(pin_MOSI, OUTPUT);      digitalWrite(pin_MOSI, LOW);          // Mise à l'état bas de la ligne MOSI (données maitre vers esclave)
  pinMode(pin_MISO, INPUT);                                             // On ne fait rien de particulier sur la ligne MISO, qui est une entrée
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)

  // Configuration des registres du "Timer 1", de l'ATmega328P -> page 112 (pour TIMSK1), page 108 (pour TCCR1A), page 110 (pour TCCR1B) du datasheet
  noInterrupts();           // On désactive toutes les interruptions, pour commencer

  bitClear(TCCR1B, WGM13);  // Active le "mode de génération de forme normal", c'est à dire ici, un comptage jusqu'à débordement du timer1 (qui enclenchera le drapeau TOV1)
  bitClear(TCCR1B, WGM12);
  bitClear(TCCR1A, WGM11);
  bitClear(TCCR1A, WGM10);

  bitClear(TCCR1B, CS12);   // Mise à 0-0-1 des bits CS12-CS11-CS10, pour définir une division de vitesse d'horloge par 1 (soit pas de ralentissement d'horloge, ici)
  bitClear(TCCR1B, CS11);
  bitSet(TCCR1B, CS10);

  bitSet(TIMSK1, TOIE1);    // Active le déclenchement d'interruption en cas de débordement (si le compteur "timer1" dépasse 65535 et revient à 0, donc)
                            // Nota : cela enclenchera l'appel de la fonction "ISR(TIMER1_OVF_vect)", écrite plus bas
 
  TCNT1 = VALEUR_DEMARRAGE_TIMER1;    // Définit la valeur de démarrage du Timer 1
  interrupts();                       // Et, on ré-active les interruptions

}


// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
void loop() {
  // Aucun code ici, car mis à part la fonction SETUP, "tout" se passe dans la fonction interruption, tout en bas : ISR(TIMER1_OVF_vect)
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
  if(lire_valeur_MISO != 0)
    valeurCible |= (1 << positionDuBit);
  mettre_SCK_a_etat_bas;

  // Nota : on compte environ 10 cycles d'horloge ici, ce qui fait, sur la base d'un µC tournant à 16 MHz, une fréquence SPI à 1,6 MHz
  //        (ce qui est la vitesse maxi de fonctionnement théorique du MCP 3201)
}
void lectureBitSPIvitesseMaxADC() {
  executer_NOP;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  mettre_SCK_a_etat_haut;
  executer_NOP;
  executer_NOP;
  executer_NOP;
  mettre_SCK_a_etat_bas;
}

// ===========================================================
// Fonction de lecture ADC
// ===========================================================
uint16_t litADC() {

  // Variable qui nous permettra de stocker la valeur lue par l'ADC (à partir des 12 bits qu'on va récupérer, via le bus SPI)
  uint16_t valeurADC = 0;

  // Sélection de l'ADC (en abaissant sa ligne "slave select")
  selectionner_ADC;
  
  // Remarque : l'ADC est assuré par un MCP3201 ; une fois la ligne slave-select abaissée, il nous faudra 15 coups d'horloge SCK pour récupérer les données
  //            (les 2 premiers coups d'horloge initialisent la communication, le 3ème coup génère un bit nul, et du 4ème au 15ème, on récupère nos 12 bits de données)

  // Premier coup d'horloge
  lectureBitSPIvitesseMaxADC();

  // Second coup d'horloge
  lectureBitSPIvitesseMaxADC();

  // 3ème coup d'horloge (pour faire générer un bit nul, qu'on ne vérifiera pas ici)
  lectureBitSPIvitesseMaxADC();

  // 4ème au 15ème coup d'horloge
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

  // Désélection de l'ADC (en remontant sa ligne "slave select")
  desactiver_ADC;

  // Retourne la valeur lue
  return valeurADC;

}


// ===========================================================
// Fonction permettant d'écrire 1 bit sur le bus SPI, à la vitesse maximale du µC
// ===========================================================
void ecritureBitSPIvitesseMaximale(uint8_t valeur) {

  valeur == 0 ? mettre_MOSI_a_etat_bas : mettre_MOSI_a_etat_haut;
  mettre_SCK_a_etat_haut;
  mettre_SCK_a_etat_bas;

  // Nota : à 16 MHz, chaque cycle du µC dure 62,5 ns
  //       (bien qu'on soit à la vitesse maximale, c'est encore suffisamment "lent" pour communiquer avec le DAC ou la SRAM)

}


// ===========================================================
// Fonction d'écriture à destination du DAC
// ===========================================================
void ecritDansDAC(uint16_t valeur) {
  
  // Sélection du DAC (en abaissant sa ligne "slave select")
  selectionner_DAC;

  // Ecriture des 4 bits de configuration du DAC
  ecritureBitSPIvitesseMaximale(0);     // 1er bit de config : toujours à 0
  ecritureBitSPIvitesseMaximale(0);     // 2ème bit de config : 0=unbuffered, 1=buffered
  ecritureBitSPIvitesseMaximale(1);     // 3ème bit de config : 0=gain double, 1=pas de gain
  ecritureBitSPIvitesseMaximale(1);     // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active

  // Ecriture des 12 bits de données
  ecritureBitSPIvitesseMaximale(valeur & (1 << 11));    // Bit 11
  ecritureBitSPIvitesseMaximale(valeur & (1 << 10));    // Bit 10
  ecritureBitSPIvitesseMaximale(valeur & (1 << 9));     // Bit 9
  ecritureBitSPIvitesseMaximale(valeur & (1 << 8));     // Bit 8
  ecritureBitSPIvitesseMaximale(valeur & (1 << 7));     // Bit 7
  ecritureBitSPIvitesseMaximale(valeur & (1 << 6));     // Bit 6
  ecritureBitSPIvitesseMaximale(valeur & (1 << 5));     // Bit 5
  ecritureBitSPIvitesseMaximale(valeur & (1 << 4));     // Bit 4
  ecritureBitSPIvitesseMaximale(valeur & (1 << 3));     // Bit 3
  ecritureBitSPIvitesseMaximale(valeur & (1 << 2));     // Bit 2
  ecritureBitSPIvitesseMaximale(valeur & (1 << 1));     // Bit 1
  ecritureBitSPIvitesseMaximale(valeur & (1 << 0));     // Bit 0

  // Désélection du DAC (en remontant sa ligne "slave select")
  desactiver_DAC;
}



// ===========================================================
// Fonction d'interruption du Timer 1 arduino, en cas de débordement
// ===========================================================
ISR(TIMER1_OVF_vect) {

  // Lit la valeur échantillonnée dans l'ADC
  uint16_t valeurLue = litADC();

  // Ecrit cette valeur dans le DAC
  ecritDansDAC(valeurLue);

  // Et remise du compteur Timer1 à la "bonne valeur", afin d'appeller cette fonction à un rythme de 44100 Hz (valeur par défaut, inscrite tout en haut)
  TCNT1 = VALEUR_DEMARRAGE_TIMER1;
  
}
