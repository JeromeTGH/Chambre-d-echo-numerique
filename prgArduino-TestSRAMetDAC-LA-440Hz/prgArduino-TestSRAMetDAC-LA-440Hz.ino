/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestSRAMetDAC-LA-440Hz.ino

  Description :   Programme permettant de générer un signal de 440 Hz environ sur la sortie audio,
                  pour tester le bon fonctionnement du DAC. Le signal, de forme sinusoïdal, sera
                  stocké dans la mémoire SRAM (écrite au démarrage de l'arduino, puis lue au fur
                  et à mesure ensuite).

  Remarques :     --> l'ADC ne sera pas utilisé ici
                  --> l'Arduino utilisé est un "Nano" (équipé du microcontrôleur ATmega328P, donc)

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       27.05.2023

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

#define pin_LED_CYCLE 7     // La LED faisant apparaître les "cycles d'écriture du DAC" est reliée à la pin D7 de l'Arduino (soit la broche PD7 de l'ATmega328P)

// ===========================================================
// Fonctions raccourcies
// ===========================================================
// Rappel : PORTx permet de définir l'état de sortie du registre x, tandis que PINx permet de le lire

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

// Ligne LED (branchée sur sortie D7 de la carte Arduino, soit la pin PD7 du µC, donc le bit 7 sur PORTD[7..0])
#define eteindre_la_LED_montrant_les_cycles   PORTD &= 0b01111111
#define allumer_la_LED_montrant_les_cycles    PORTD |= 0b10000000

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")

// ===========================================================
// CONSTANTES du programme
// ===========================================================
#define FREQUENCE_D_ECHANTILLONNAGE   20000   // 40000 max, compte tenu des "limites" de l'arduino (pour info, la "qualité CD" est à 44100 Hz)
#define FREQUENCE_DU_SIGNAL_DE_SORTIE 440     // 440 hertz, par exemple (pour obtenir le "LA", du diapason)

#define VALEUR_MAXI_TIMER1        65535   // Valeur max que peut atteindre le "Timer 1", dont nous nous servirons ici (pour rappel, c'est un compteur 16 bits ; il compte donc de 0 à 65535)
#define VALEUR_DE_COMPENSATION    55      // Valeur qui s'ajoute au compte Timer1, pour compenser le "temps perdu" au moment de l'appel d'interruption (et retour)

// *********************************************************************************************************************************
// Calcul de la valeur initiale qu'on donnera au Timer 1, chaque fois qu'il aura dépassé son max (c'est à dire qu'il aura "débordé")
// *********************************************************************************************************************************
volatile unsigned int valeur_initiale_du_timer1 = VALEUR_MAXI_TIMER1 - (F_CPU / FREQUENCE_D_ECHANTILLONNAGE) + VALEUR_DE_COMPENSATION;
      // En sachant que F_CPU est déjà une valeur connue ; de base, elle vaut 16.000.000
      // pour un Arduino Nano (si bien cadencé par son quartz externe à 16 MHz)

// *****************************************************************************************************************************************
// Tableau qui contiendra la représentation de l'onde sinusoïdale qu'on cherche à reproduire en sortie (pour obtenir un "beau" LA, à 440 Hz)
// *****************************************************************************************************************************************
const unsigned int nombre_de_divisions_onde_sinus = FREQUENCE_D_ECHANTILLONNAGE/FREQUENCE_DU_SIGNAL_DE_SORTIE;
volatile unsigned int position_dans_SRAM = 0;


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

  // Définition de la broche, où est branchée la LED montrant les cycles d'écriture du DAC, en "sortie" (et mise à 0, pour que la LED soit éteinte au démarrage)
  pinMode(pin_LED_CYCLE, OUTPUT);
  digitalWrite(pin_LED_CYCLE, LOW);

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

  // *********************************************************************************************
  // Génération des valeurs représentant une onde sinusoïdale, pour la reproduire en sortie de DAC
  // *********************************************************************************************
  generationSinusEtStockageDansSRAM();

  // *********************************************************************************************
  // Configuration des registres du "Timer 1", de l'ATmega328P -> page 112 (pour TIMSK1), page 108 (pour TCCR1A), page 110 (pour TCCR1B) du datasheet
  // *********************************************************************************************
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
 
  TCNT1 = valeur_initiale_du_timer1;  // Définit la valeur de démarrage du Timer 1
  interrupts();                       // Et, on ré-active les interruptions

  // Délai de stabilisation, avant de lancer le programme
  delay(100);
}


// =============================================
// Fonction LOOP (boucle programme, après setup)
// =============================================
void loop() {
  // Aucun code ici … car "tout" se passe dans la fonction SETUP au démarrage, puis dans la fonction d'interruption "ISR(TIMER1_OVF_vect)" après
}


// ==================================================================
// Fonction lectureEcritureSPI (écrit et lit un octet sur le bus SPI)
// ==================================================================
byte lectureEcritureSPI(byte donneesAenvoyer = 0b11111111) {

  // Démarrage de l'envoi/réception simultané de données
  SPDR = donneesAenvoyer;               // On stocke les données à envoyer (8 bits, ici) dans le registre SPDR ("SPI Data Register") ; elles seront émises via la ligne MOSI
                                        // Par défaut, cette valeur sera à 0b11111111, si aucune donnée n'est à envoyer (commande "nulle")
                                        // Nota : le fait de modifier la valeur de ce registre lance automatiquement la transmission SPI

  while(bitRead(SPSR, SPIF) != 1);      // On attend que le bit SPIF ("SPI Interrupt Flag") soit égal à 1 dans le registre SPSR ("SPI Status Register") ; cela arrive une fois le tranfert achevé

  return SPDR;      // On récupère les données lues (8 bits), présentées sur la ligne MISO ; le bit SPIF est automatiquement remis à 0, ainsi.
}


// ========================================
// Fonction d'écriture à destination du DAC
// ========================================
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


// ===============================================
// Fonction génération sinus et stockage dans SRAM
// ===============================================
void generationSinusEtStockageDansSRAM() {

  uint16_t valeur_a_enregistrer;            // Variable 16 bits, qui contiendra les portions codées sur 12 bits de notre sinusoïde
  
  for(uint16_t i=0 ; i < nombre_de_divisions_onde_sinus ; i++) {
    // Le DAC a besoin de valeurs sur 12 bits (variant donc de 0 à 4095, avec une moyenne à 2047.5), d'où ce choix ici
    valeur_a_enregistrer = 2047.5 + 2047.5 * cos(2*PI*i/nombre_de_divisions_onde_sinus);
                // Nota : un "tour complet" de sinusoïde fait "2xPi radians"
    ecritureEnMemoireSRAM(i*2, valeur_a_enregistrer);
  }
}


// ===================================
// Fonction d'écriture en mémoire SRAM
// ===================================
void ecritureEnMemoireSRAM(uint16_t adresseVisee, uint16_t donnees) {

  // On fixe la vitesse de communication SPI à Fosc/2, soit 8 MHz, dans le cas d'un Arduino Nano cadencé par un quartz à 16 MHz
  // (pour ce faire, on met les bits SPI2X à 1, SPR1 à 0, et SPR0 à 0 ; cf. tableau détaillant tout ça, tout en haut)
  bitSet(SPCR, SPI2X);
  bitClear(SPCR, SPR1);
  bitClear(SPCR, SPR0);

  // Activation de la SRAM (en baissant sa ligne /CS à LOW)
  selectionner_SRAM;
  
  // Envoi d'une instruction signifiant qu'on veut ÉCRIRE dans la SRAM (code 0000 0010)
  lectureEcritureSPI(0b00000010);
  
  // Envoi de 24 bits d'ADRESSE (nécessaire, pour la SRAM)
  lectureEcritureSPI(0b00000000);                                     // Envoi de 8 bits à zéro (puisqu'on adresse que sur 16 bits, dans cet exemple)
  lectureEcritureSPI((uint8_t)(adresseVisee >> 8));                   // Envoi des 8 bits d'ADRESSE de poids fort en premier
  lectureEcritureSPI((uint8_t)(adresseVisee & 0b0000000011111111));   // Envoi des 8 bits d'ADRESSE de poids faible
  
  // Écriture de 2 fois 8 bits de DONNEES
  lectureEcritureSPI((uint8_t)(donnees >> 8));                        // Envoi des 8 bits de DONNEES de poids fort en premier
  lectureEcritureSPI((uint8_t)(donnees & 0b0000000011111111));        // Envoi des 8 bits de DONNEES de poids faible
     
  // Déselection de la SRAM (en remontant sa ligne /CS à HIGH)
  desactiver_SRAM;

}


// ======================================
// Fonction de lecture de la mémoire SRAM
// ======================================
uint16_t lectureEnMemoireSRAM(uint16_t adresseVisee) {

  // On fixe la vitesse de communication SPI à Fosc/2, soit 8 MHz, dans le cas d'un Arduino Nano cadencé par un quartz à 16 MHz
  // (pour ce faire, on met les bits SPI2X à 1, SPR1 à 0, et SPR0 à 0 ; cf. tableau détaillant tout ça, tout en haut)
  bitSet(SPCR, SPI2X);
  bitClear(SPCR, SPR1);
  bitClear(SPCR, SPR0);

  // Variables de stockage temporaire
  uint8_t donnees8bisDePoidsFort;
  uint8_t donnees8bisDePoidsFaible;
  
  // Activation de la SRAM (en baissant sa ligne /CS à LOW)
  selectionner_SRAM;
  
  // Envoi d'une instruction signifiant qu'on veut LIRE dans la SRAM (code 0000 0011)
  lectureEcritureSPI(0b00000011);
  
  // Envoi de 24 bits d'ADRESSE (nécessaire, pour la SRAM)
  lectureEcritureSPI(0b00000000);                                     // Envoi de 8 bits à zéro (puisqu'on adresse que sur 16 bits, dans cet exemple)
  lectureEcritureSPI((uint8_t)(adresseVisee >> 8));                   // Envoi des 8 bits d'ADRESSE de poids fort en premier
  lectureEcritureSPI((uint8_t)(adresseVisee & 0b0000000011111111));   // Envoi des 8 bits d'ADRESSE de poids faible
  
  // Lecture de 2 fois 8 bits de DONNEES
  donnees8bisDePoidsFort = lectureEcritureSPI();                      // Envoi des 8 bits de DONNEES de poids fort en premier
  donnees8bisDePoidsFaible = lectureEcritureSPI();                    // Envoi des 8 bits de DONNEES de poids faible
     
  // Déselection de la SRAM (en remontant sa ligne /CS à HIGH)
  desactiver_SRAM;

  // Et renvoit des données lues, au format 16 bits
  return ((donnees8bisDePoidsFort << 8) || donnees8bisDePoidsFaible);
}


// =================================================================
// Fonction d'interruption du Timer 1 arduino, en cas de débordement
// =================================================================
ISR(TIMER1_OVF_vect) {

  // On remet le compteur du Timer1 à la "bonne valeur" de démarrage, car il continu de compter, pendant cette interruption
  TCNT1 = valeur_initiale_du_timer1;

  // On allume la LED "montrant les cycles" (LED jaune figurant sur la carte PCB)
  allumer_la_LED_montrant_les_cycles;
   
  // On lit la valeur 2 x 8 bits, stockée dans la SRAM
  uint16_t valeurLue = lectureEnMemoireSRAM(position_dans_SRAM);

  // On écrit cette valeur dans le DAC
  ecritDansDAC(valeurLue);

  // On détermine la prochaine position dans la mémoire SRAM (représentant notre onde sinusoïdale à générer en sortie)
  position_dans_SRAM = position_dans_SRAM + 2;                  // Nota : comme la mémoire ne peut stocker que des données 8 bits et que
  if(position_dans_SRAM == (nombre_de_divisions_onde_sinus*2))  //        notre signal est "codé" sur 16 bits, alors on avance de 2 en 2
    position_dans_SRAM = 0;

  // Et on éteint la LED, pour signifier la fin d'un cycle (nota : l'allumage/extinction ne sera pas visible à l'oeil nu, compte tenu de la vitesse ;
  // par contre, cette sortie sert à des mesures sur oscillo, pour comparer la vitesse des cycles, par rapport à la vitesse d'échantillonage souhaitée)
  eteindre_la_LED_montrant_les_cycles;
  
}
