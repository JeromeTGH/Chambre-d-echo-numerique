/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestADC-HardwareSPI.ino

  Description :   Programme permettant de tester l'ADC

  Remarques :     --> la mémoire SRAM et le DAC ne seront pas utilisés ici
                  --> Méthode utilisée : hardware SPI

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       23.04.2023

*/

// Lignes SPI de l'Arduino Nano <--> ADC
#define pin_MOSI      11    // Le signal MOSI sort sur la pin D11 de la carte Arduino (cette pin est reliée en interne à la broche PB3 du µC ATmega328P)
#define pin_MISO      12    // Le signal MISO entre sur la pin D12 de la carte Arduino (cette pin est reliée en interne à la broche PB4 du µC ATmega328P)
#define pin_SCK       13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)
#define pin_SS_ADC    2     // Le Slave-Select de l'ADC sort sur la pin D2 de la carte Arduino (cette pin est reliée en interne à la broche PD2 du µC ATmega328P)     

// Ligne slave-select de l'ADC (branchée sur sortie D2 de la carte Arduino, soit la pin PD2 du µC, donc le bit 2 sur PORTD[7..0])
#define selectionner_ADC        PORTD &= 0b11111011
#define desactiver_ADC          PORTD |= 0b00000100

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {
  pinMode(pin_SS_ADC, OUTPUT);          // Défini la broche arduino comme sortie, pilotant la broche SS de l'ADC
  desactiver_ADC;                       // ... et la fixe à l'état haut (communication SPI/ADC désactivé)

  pinMode(pin_MOSI, OUTPUT);      digitalWrite(pin_MOSI, LOW);          // Mise à l'état bas de la ligne MOSI (ligne de données maitre vers esclave)
  pinMode(pin_MISO, INPUT);                                             // On ne fait rien de particulier sur la ligne MISO, mise à part la déclarer en entrée
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)

  pinMode(10, OUTPUT);    // Pin SS "normale", du port SPI arduino    ==> il faut la déclarer en sortie, même si elle n'est pas utilisée
                          //                                              (sinon, l'arduino ne passera pas en SPI mode master, et restera en slave SPI)

  // Configuration du registre SPCR ("SPI Control Register") de l'ATmega328P -> page 140/294 du datasheet
  bitClear(SPCR, SPIE);     // Désactive les interruptions SPI
  bitClear(SPCR, DORD);     // Définit l'ordre des données (ici, à 0, c'est le MSB qui sera passé en premier, jusqu'à finir par le LSB)
  bitSet(SPCR, MSTR);       // Définit l'Arduino en "maître SPI" (d'où ce bit à 1)
  bitClear(SPCR, CPOL);     // Définit la polarité d'horloge SCK (ici, avec la valeur 0, on dit que SCK est actif à l'état haut / inactif à l'état bas)
  bitClear(SPCR, CPHA);     // Définit la phase d'horloge SCK (ici, avec la valeur 0, on dit que les données SPI sont échantillonnées sur chaque début de front d'horloge SCK)
  bitSet(SPCR, SPE);        // Autorise les opérations sur le bus SPI

  bitClear(SPCR, SPR1);     // Si SPR1=0 et SPR0=1, et que SPI2X=0, alors la fréquence d'horloge sur SCK sera égale à Fosc/16 (soit 16 MHz/16, soit 1 MHz)
  bitSet(SPCR, SPR0);       // Nota : l'ADC peut fonctionner jusqu'à environ 1,6 MHz max (d'où le "bridage" à 1 MHz, ici)
  
  // Configuration du registre SPSR ("SPI Status Register") de l'ATmega328P -> page 141/294 du datasheet
  bitClear(SPSR, SPI2X);    // Permet de "doubler" ou non la vitesse d'horloge SCK, de notre Arduino
                            // Nota : ce bit, "SPI2X", sera utilisé en combinaison avec les bits SPR1 et SPR0 (cf. ci-dessus/dessous)
  

          /* Relation entre vitesse d'horloge et bits SPI2X, SPR1, et SPR0
             Nota : dans le cas de notre Arduino Nano, cadencé par un quartz à 16 Mhz, alors Fosc = 16 000 000
 
              ------------------------------------------------------
              | SPI2X |  SPR1 |  SPR0 | Fréquence d'horloge SCK    |
              ------------------------------------------------------
              |   0   |   0   |   0   | Fosc / 4           (4 MHz) |
              |   0   |   0   |   1   | Fosc / 16          (1 MHz) |     <--- fréquence de fonctionnement la plus haute, pour communiquer avec notre ADC
              |   0   |   1   |   0   | Fosc / 64        (250 kHz) |
              |   0   |   1   |   1   | Fosc / 128       (125 MHz) |
              |   1   |   0   |   0   | Fosc / 2           (8 MHz) |
              |   1   |   0   |   1   | Fosc / 8           (2 MHz) |
              |   1   |   1   |   0   | Fosc / 32        (500 KHz) |
              |   1   |   1   |   1   | Fosc / 64        (250 KHz) |
              ------------------------------------------------------ 
          */


  Serial.begin(9600);                   // Initialise la communication série, avec le PC
  Serial.println(F("================================================================================"));
  Serial.println(F("Test de l'ADC modèle MCP 3201, de chez Microchip"));
  Serial.println(F("- Méthode de communication : harware SPI, purement logiciel"));
  Serial.println(F("================================================================================"));
  Serial.println("");

}


// ===========================================================
// Fonction permettant de lire un octet sur le bus SPI
// ===========================================================
uint8_t lectureOctetSurBusSPI() {

    SPDR = 0b11111111;          // Le fait d'envoyer une valeur démarre l'échange SPI (émission d'une cmde nulle, et réception des données simultanément)
    asm volatile("nop");
    while(!(SPSR & (1<<SPIF))); // Attente de la fin de transmission
    return SPDR;
}


// ===========================================================
// Fonction LOOP (boucle programme, après setup)
// ===========================================================
void loop() {
  uint16_t valeurADC = 0;
  uint8_t valeurHaute = 0;
  uint8_t valeurBasse = 0;

  selectionner_ADC;       // Active la communication avec l'ADC (en abaissant sa ligne /SS)
  
  valeurHaute = lectureOctetSurBusSPI();    // Lecture de 8 premiers bits        (HiZ, HiZ, 0, D11, D10, D9, D8, D7) ; Nota : les 3 premiers bits sont des 'déchets', à éliminer
  valeurBasse = lectureOctetSurBusSPI();    // Lecture de 8 bits supplémentaires (D6, D5, D4, D3, D2, D1, D0, D1)    ; Nota : le dernier bit (D1) est également un 'déchet', à éliminer
  
  valeurADC = (valeurHaute << 8) | valeurBasse;

  valeurADC = valeurADC >> 1;                 // On décale d'un cran vers la droite, pour effacer le "D1" de trop, à la fin (cf. ci-dessus)
  valeurADC = valeurADC & 0b0000111111111111; // On "efface" les 4 bits de gauche, pour ne conserver que nos 12 bits de données
   
  desactiver_ADC;         // Désactive la communication avec l'ADC (en remontant sa ligne /SS)

  Serial.print(F("Valeur de l'ADC = "));    // Affichage du résultat sur le moniteur série de l'IDE Arduino
  Serial.println(valeurADC);
  
  delay(1000);   // Petit délai
}
