/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestEntreeSortie-BitBanging.ino

  Description :   Programme permettant de renvoyer directement les données lues en entrée (via l'ADC)
                  sur la sortie (via le DAC)

  Remarques :     --> la mémoire SRAM ne sera pas utilisée ici
                  --> l'Arduino utilisé pour ce projet est un "Nano" (donc un modèle équipé du microcontrôleur ATmega328P)
                  --> Méthode : bit banging

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       07.04.2023

*/

// ===========================================================
// Définition des raccordements entre périphériques et bus SPI
// ===========================================================
#define pin_MOSI      11    // Le signal MOSI sort sur la pin D11 de la carte Arduino (cette pin est reliée en interne à la broche PB3 du µC ATmega328P)
#define pin_MISO      12    // Le signal MISO entre sur la pin D12 de la carte Arduino (cette pin est reliée en interne à la broche PB4 du µC ATmega328P)
#define pin_SCK       13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)

#define pin_SS_ADC    10    // Le Slave-Select de l'ADC sort sur la pin D10 de la carte Arduino (cette pin est reliée en interne à la broche PB2 du µC ATmega328P)
#define pin_SS_DAC    9     // Le Slave-Select du DAC sort sur la pin D9 de la carte Arduino (cette pin est reliée en interne à la broche PB1 du µC ATmega328P)
#define pin_SS_SRAM   8     // Le Slave-Select de la SRAM sort sur la pin D8 de la carte Arduino (cette pin est reliée en interne à la broche PB0 du µC ATmega328P)

// ===========================================================
// Fonctions raccourcies
// ===========================================================

// Ligne slave-select de l'ADC (branchée sur sortie D10 de la carte Arduino, soit la pin PB2 du µC, donc le bit 2 sur PORTB[7..0])
#define selectionner_ADC        PORTB &= 0b11111011
#define desactiver_ADC          PORTB |= 0b00000100

// Ligne slave-select du DAC (branchée sur sortie D9 de la carte Arduino, soit la pin PB1 du µC, donc le bit 1 sur PORTB[7..0])
#define selectionner_DAC        PORTB &= 0b11111101
#define desactiver_DAC          PORTB |= 0b00000010

// Ligne slave-select de la SRAM (branchée sur sortie D8 de la carte Arduino, soit la pin PB0 du µC, donc le bit 0 sur PORTB[7..0])
#define selectionner_SRAM       PORTB &= 0b11111110
#define desactiver_SRAM         PORTB |= 0b00000001

// Ligne MOSI (branchée sur sortie D11 de la carte Arduino, soit la pin PB3 du µC, donc le bit 3 sur PORTB[7..0])
#define mettre_MOSI_a_etat_bas  PORTB &= 0b11110111
#define mettre_MOSI_a_etat_haut PORTB |= 0b00001000

// Ligne MISO (branchée sur sortie D12 de la carte Arduino, soit la pin PB4 du µC, donc le bit 4 sur PORTB[7..0])
#define lire_valeur_MISO        PINB & 0b00010000

// Ligne SCK (branchée sur sortie D13 de la carte Arduino, soit la pin PB5 du µC, donc le bit 5 sur PORTB[7..0])
#define mettre_SCK_a_etat_bas   PORTB &= 0b11011111
#define mettre_SCK_a_etat_haut  PORTB |= 0b00100000

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")

// ===========================================================
// CONSTANTES du programme
// ===========================================================
#define VITESSE_ECHANTILLONNAGE   40000   // 40000 max, compte tenu des "limites" de l'arduino (pour info, la "qualité CD" est à 44100 Hz)
#define VALEUR_MAXI_TIMER1        65535   // Valeur max que peut atteindre le "Timer 1", dont nous nous servirons ici (pour rappel, c'est un compteur 16 bits ; il compte donc de 0 à 65535)
#define VALEUR_DE_COMPENSATION    56      // Valeur qui s'ajoute au compte Timer1, pour compenser le "temps perdu" au moment de l'appel d'interruption (et retour)

// Calcul de la valeur initiale qu'on donnera au Timer 1, chaque fois qu'il aura dépassé son max (c'est à dire qu'il aura "débordé")
volatile unsigned int valeur_initiale_du_timer1 = VALEUR_MAXI_TIMER1 - (F_CPU / VITESSE_ECHANTILLONNAGE) + VALEUR_DE_COMPENSATION;
// En sachant que F_CPU est déjà une valeur connue ; de base, elle vaut 16.000.000 pour un Arduino Nano (si bien cadencé par son quartz externe à 16 MHz)


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {

  // Désactivation de tous nos périphériques SPI, pour commencer, au démarrage
  pinMode(pin_SS_ADC, OUTPUT);    digitalWrite(pin_SS_ADC, HIGH);       // Les lignes "slave select" sont actives à l'état bas, donc inactives à l'état haut
  pinMode(pin_SS_DAC, OUTPUT);    digitalWrite(pin_SS_DAC, HIGH); 
  pinMode(pin_SS_SRAM, OUTPUT);   digitalWrite(pin_SS_SRAM, HIGH); 

  // Mise au repos du bus SPI
  pinMode(pin_MOSI, OUTPUT);      digitalWrite(pin_MOSI, LOW);          // Mise à l'état bas de la ligne MOSI (ligne de données maitre vers esclave)
  pinMode(pin_MISO, INPUT);                                             // On ne fait rien de particulier sur la ligne MISO, qui est une entrée
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)

  // Configuration des registres du "Timer 1", de l'ATmega328P -> page 112 (pour TIMSK1), page 108 (pour TCCR1A), page 110 (pour TCCR1B) du datasheet
  noInterrupts();           // Avant tout, on désactive toutes les interruptions

  bitClear(TCCR1B, WGM13);  // On active le "mode de génération de forme « normal »", c'est à dire un mode de comptage jusqu'à débordement du timer1
  bitClear(TCCR1B, WGM12);  // (pour cela, on met les bits de configuration WGM13, 12, 11, et 10 à zéro)
  bitClear(TCCR1A, WGM11);
  bitClear(TCCR1A, WGM10);

  bitClear(TCCR1B, CS12);   // Mise à 0-0-1 des bits CS12-CS11-CS10, pour définir une division de vitesse d'horloge par 1, pour le Timer1 (donc pas de ralentissement d'horloge, ici)
  bitClear(TCCR1B, CS11);
  bitSet(TCCR1B, CS10);

  bitSet(TIMSK1, TOIE1);    // On active le déclenchement d'interruption en cas de débordement du Timer1 (si le compteur essaye de dépasser 65535, donc)
                            // Nota : cela enclenchera l'appel de la fonction "ISR(TIMER1_OVF_vect)", écrite plus bas
 
  TCNT1 = valeur_initiale_du_timer1;    // Définit la valeur de démarrage du Timer 1
  interrupts();                         // Et, on ré-active les interruptions

}


// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
void loop() {
  // Aucun code ici … car "tout" se passe dans la fonction SETUP au démarrage, puis dans la fonction d'interruption "ISR(TIMER1_OVF_vect)" après
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
  mettre_SCK_a_etat_bas;
}


// ===========================================================
// Fonction de lecture ADC
// ===========================================================
uint16_t litADC() {

  // Variable qui nous permettra de stocker la valeur lue par l'ADC
  uint16_t valeurADC = 0;

  // Sélection de l'ADC (en abaissant sa ligne "slave select")
  selectionner_ADC;
  
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

  // Désélection de l'ADC (en remontant sa ligne "slave select")
  desactiver_ADC;

  // Retourne la valeur lue (remarque : celle-ci sera en fait une suite de 0 et de 1, du style 110010001101 ; cela permet de faire passer nos 12 bits
  // dans une seule variable uint, codée sur 16 bits, au lieu d'utiliser une chaîne de caractère, qui aurait monopolisé 12 x 8 bits)
  return valeurADC;

}


// ===========================================================
// Fonction permettant d'écrire 1 bit sur le bus SPI, à la vitesse maximale du µC
// ===========================================================
void ecritureBitSPIvitesseMaximale(uint16_t valeur) {

  valeur == 0 ? mettre_MOSI_a_etat_bas : mettre_MOSI_a_etat_haut;
  mettre_SCK_a_etat_haut;
  executer_NOP;
  mettre_SCK_a_etat_bas;

  // Nota : à 16 MHz, chaque cycle du µC dure 62,5 ns
  //       (bien qu'on soit à la vitesse maximale, c'est encore suffisamment "lent" pour communiquer avec le DAC ou la SRAM, à plein régime !)

}


// ===========================================================
// Fonction d'écriture à destination du DAC
// ===========================================================
void ecritDansDAC(uint16_t valeur) {

  // Sélection du DAC (en abaissant sa ligne "slave select")
  selectionner_DAC;

  // Ecriture des 4 bits de configuration du DAC
  ecritureBitSPIvitesseMaximale(0);     // 1er bit de config : toujours à 0
  ecritureBitSPIvitesseMaximale(0);     // 2ème bit de config : 0=unbuffered, 1=buffered (ici choix unbuffured, donc 0)
  ecritureBitSPIvitesseMaximale(1);     // 3ème bit de config : 0=gain double, 1=pas de gain (ici choix "pas de gain", donc 1)
  ecritureBitSPIvitesseMaximale(1);     // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active (ici choix "sortie active", donc 1)

  // Ecriture des 12 bits de données, préalablement lues par l'ADC
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

  // On remet le compteur du Timer1 à la "bonne valeur" de démarrage, car il continu de compter, pendant cette interruption
  TCNT1 = valeur_initiale_du_timer1;

  // Lecture de la valeur échantillonnée par l'ADC
  uint16_t valeurLue = litADC();

  // Écriture de cette valeur dans le DAC
  ecritDansDAC(valeurLue);
  
}
