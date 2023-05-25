/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestEntreeSortie-HardwareSPI.ino

  Description :   Programme permettant de renvoyer directement les données lues en entrée (via l'ADC)
                  sur la sortie (via le DAC)

  Remarques :     --> la mémoire SRAM ne sera pas utilisée ici
                  --> l'Arduino utilisé pour ce projet est un "Nano" (donc un modèle équipé du microcontrôleur ATmega328P)
                  --> Méthode : hardware SPI

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       09.04.2023

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
#define VITESSE_ECHANTILLONNAGE   44100   // De base, on échantillonnera le son reçu à 44100 Hz ("qualité CD", en fait)
#define VALEUR_MAXI_TIMER1        65535   // Valeur max que peut atteindre le "Timer 1", dont nous nous servirons ici (pour rappel, c'est un compteur 16 bits ; il compte donc de 0 à 65535)

// Calcul de la valeur initiale qu'on donnera au Timer 1, chaque fois qu'il aura dépassé son max (c'est à dire qu'il aura "débordé")
#define VALEUR_DEMARRAGE_TIMER1   VALEUR_MAXI_TIMER1 - (F_CPU / VITESSE_ECHANTILLONNAGE)  // En sachant que F_CPU est déjà une valeur définie (valant 16000000L, correspondant à la valeur du quartz 16 MHz, équipant l'Arduino Nano)


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

  // Configuration du registre SPSR ("SPI Status Register") de l'ATmega328P -> page 141/294 du datasheet
  bitSet(SPSR, SPI2X);      // Permet de "doubler" la vitesse d'horloge SCK, de notre Arduino qui sera configuré "maître SPI"
                            // Nota : ce bit, "SPI2X", sera utilisé en combinaison avec les bits SPR1 et SPR0 (cf. ci-dessous)

  // Configuration du registre SPCR ("SPI Control Register") de l'ATmega328P -> page 140/294 du datasheet
  bitClear(SPCR, SPIE);     // Désactive les interruptions SPI
  bitClear(SPCR, DORD);     // Définit l'ordre des données (ici, à 0, c'est le MSB qui sera passé en premier, jusqu'à finir par le LSB)
  bitSet(SPCR, MSTR);       // Définit l'Arduino en "maître SPI" (d'où ce bit à 1)
  bitClear(SPCR, CPOL);     // Définit la polarité d'horloge SCK (ici, avec la valeur 0, on dit que SCK est actif à l'état haut / inactif à l'état bas)
  bitClear(SPCR, CPHA);     // Définit la phase d'horloge SCK (ici, avec la valeur 0, on dit que les données SPI sont échantillonnées sur chaque début de front d'horloge SCK)
  bitSet(SPCR, SPE);        // Autorise les opérations sur le bus SPI

  bitClear(SPCR, SPR1);     // Si SPR1=0 et SPR0=0, et que SPI2X=1, alors la fréquence d'horloge sur SCK sera égale à Fosc/2 (soit 16 MHz/2, soit 8 MHz … soit le max atteignable avec ce µC)
  bitClear(SPCR, SPR0);
  
          /* Relation entre vitesse d'horloge et bits SPI2X, SPR1, et SPR0
             Nota : dans le cas de notre Arduino Nano, cadencé par un quartz à 16 Mhz, alors Fosc = 16 000 000
 
              ------------------------------------------------------
              | SPI2X |  SPR1 |  SPR0 | Fréquence d'horloge SCK    |
              ------------------------------------------------------
              |   0   |   0   |   0   | Fosc / 4           (4 MHz) |
              |   0   |   0   |   1   | Fosc / 16          (1 MHz) |     <--- fréquence de fonctionnement la plus haute, pour communiquer avec notre ADC
              |   0   |   1   |   0   | Fosc / 64        (250 kHz) |
              |   0   |   1   |   1   | Fosc / 128       (125 MHz) |
              |   1   |   0   |   0   | Fosc / 2           (8 MHz) |     <--- fréquence de fonctionnement la plus haute, pour communiquer avec notre DAC et notre SRAM
              |   1   |   0   |   1   | Fosc / 8           (2 MHz) |
              |   1   |   1   |   0   | Fosc / 32        (500 KHz) |
              |   1   |   1   |   1   | Fosc / 64        (250 KHz) |
              ------------------------------------------------------
          */

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
 
  TCNT1 = VALEUR_DEMARRAGE_TIMER1;    // Définit la valeur de démarrage du Timer 1
  interrupts();                       // Et, on ré-active les interruptions

}


// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
void loop() {
  // Aucun code ici … car "tout" se passe dans la fonction SETUP au démarrage, puis dans la fonction d'interruption "ISR(TIMER1_OVF_vect)" après
}



// ===========================================================
// Fonction lectureEcritureSPI (écrit et lit un octet sur le bus SPI)
// ===========================================================
byte lectureEcritureSPI(byte donneesAenvoyer = 0b11111111) {

  // Démarrage de l'envoi/réception simultané de données
  SPDR = donneesAenvoyer;               // On stocke les données à envoyer (8 bits, ici) dans le registre SPDR ("SPI Data Register") ; elles seront émises via la ligne MOSI
                                        // Par défaut, cette valeur sera à 0b11111111, si aucune donnée n'est à envoyer (commande "nulle")
                                        // Nota : le fait de modifier la valeur de ce registre lance automatiquement la transmission SPI

  while(bitRead(SPSR, SPIF) != 1);      // On attend que le bit SPIF ("SPI Interrupt Flag") soit égal à 1 dans le registre SPSR ("SPI Status Register") ; cela arrive une fois le tranfert achevé

  return SPDR;      // On récupère les données lues (8 bits), présentées sur la ligne MISO ; le bit SPIF est automatiquement remis à 0, ainsi.
}


// ===========================================================
// Fonction de lecture ADC
// ===========================================================
uint16_t litADC() {

  // Variable qui nous permettra de stocker la valeur lue par l'ADC
  uint16_t valeurADC = 0;
  uint8_t valHaut;
  uint8_t valBas;

  // On fixe la vitesse de communication SPI à Fosc/16, soit 1 MHz, dans le cas d'un Arduino Nano cadencé par un quartz à 16 MHz
  // (pour ce faire, on met les bits SPI2X à 0, SPR1 à 0, et SPR0 à 1 ; cf. tableau détaillant tout ça, tout en haut)
  bitClear(SPCR, SPI2X);
  bitClear(SPCR, SPR1);
  bitSet(SPCR, SPR0);

  // Sélection de l'ADC (en abaissant sa ligne "slave select")
  selectionner_ADC;
  
  // Remarque : le modèle d'ADC utilisé ici est un MCP3201 ; une fois la ligne slave-select abaissée, il nous faudra 15 coups d'horloge SCK pour récupérer les données
  //            (les 2 premiers coups d'horloge initialisent la communication, le 3ème coup génère un bit nul, et du 4ème au 15ème, on récupère nos 12 bits de données)
  
  // Nous allons lire 2 octets sur le bus SPI (donc 16 bits), pour capturer ces 15 bits
  valHaut = lectureEcritureSPI();         // Lecture/enregistrement des 8 premiers bits
  valBas = lectureEcritureSPI();          // Lecture/enregistrement des 8 bits suivants
  valeurADC = (valHaut << 8) | valBas;    // Concaténation de ces 2x8 bits, soit 16 bits lus

  // On décale la valeur enregistrée d'un cran vers la droite (pour ne conserver que les 15 bits qui nous intéressent, sur les 16 reçus)
  valeurADC = valeurADC >> 1;

  // On conserve uniquement les 12 bits de poids faible, en mettant les autres à zéro
  valeurADC &= 0b0000111111111111;

  // Désélection de l'ADC (en remontant sa ligne "slave select")
  desactiver_ADC;

  // Retourne la valeur lue (remarque : celle-ci sera en fait une suite de 0 et de 1, du style 110010001101 ; cela permet de faire passer nos 12 bits
  // dans une seule variable uint, codée sur 16 bits, au lieu d'utiliser une chaîne de caractère, qui aurait monopolisé 12 x 8 bits)
  return valeurADC;

}


// ===========================================================
// Fonction d'écriture à destination du DAC
// ===========================================================
void ecritDansDAC(uint16_t donneesPourDAC) {
  
  // On fixe la vitesse de communication SPI à Fosc/2, soit 8 MHz, dans le cas d'un Arduino Nano cadencé par un quartz à 16 MHz
  // (pour ce faire, on met les bits SPI2X à 1, SPR1 à 0, et SPR0 à 0 ; cf. tableau détaillant tout ça, tout en haut)
  bitSet(SPCR, SPI2X);
  bitClear(SPCR, SPR1);
  bitClear(SPCR, SPR0);

  // Sélection du DAC (en abaissant sa ligne "slave select")
  selectionner_DAC;

  // Ajout de 4 bits de configuration du DAC, "à gauche" des 12 bits de données
      // 1er bit de config : toujours à 0
      // 2ème bit de config : 0=unbuffered, 1=buffered (ici choix unbuffured, donc 0)
      // 3ème bit de config : 0=gain double, 1=pas de gain (ici choix "pas de gain", donc 1)
      // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active (ici choix "sortie du DAC active", donc 1)
      // ... d'où le masque "0011" qui sera rajouté "devant" les 12 bits de poids faible (conservés avec les "000000000000" qui suivent)
  donneesPourDAC |= 0b0011000000000000;

  // Ecriture de ces 16 bits (pour rappel : 4 bits de configuration, suivis de 12 bits de données)
  lectureEcritureSPI((uint8_t)(donneesPourDAC >> 8));                   // On récupère les 8 bits de poids fort (en les décalant à droite)
  lectureEcritureSPI((uint8_t)(donneesPourDAC & 0b0000000011111111));   // On récupère les 8 bits de poids faible (en ne gardant que ces derniers)

  // Désélection du DAC (en remontant sa ligne "slave select")
  desactiver_DAC;
}


// ===========================================================
// Fonction d'interruption du Timer 1 arduino, en cas de débordement
// ===========================================================
ISR(TIMER1_OVF_vect) {
   
  // Lit la valeur échantillonnée par l'ADC
  uint16_t valeurLue = litADC();

  // Écrit cette valeur dans le DAC
  ecritDansDAC(valeurLue);

  // Et remet le compteur du Timer1 à la "bonne valeur" de démarrage, afin d'appeler cette fonction à un rythme de 44100 Hz (valeur par défaut, inscrite tout en haut)
  TCNT1 = VALEUR_DEMARRAGE_TIMER1;
  
}
