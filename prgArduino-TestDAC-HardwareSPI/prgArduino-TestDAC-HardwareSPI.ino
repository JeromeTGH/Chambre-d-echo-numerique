/*
   ______               _                  _///_ _           _                   _
  /   _  \             (_)                |  ___| |         | |                 (_)
  |  [_|  |__  ___  ___ _  ___  _ __      | |__ | | ___  ___| |_ _ __ ___  _ __  _  ___  _   _  ___
  |   ___/ _ \| __|| __| |/ _ \| '_ \_____|  __|| |/ _ \/  _|  _| '__/   \| '_ \| |/   \| | | |/ _ \
  |  |  | ( ) |__ ||__ | | ( ) | | | |____| |__ | |  __/| (_| |_| | | (_) | | | | | (_) | |_| |  __/
  \__|   \__,_|___||___|_|\___/|_| [_|    \____/|_|\___|\____\__\_|  \___/|_| |_|_|\__  |\__,_|\___|
                                                                                      | |
                                                                                      \_|
  Fichier :       prgArduino-TestDAC-HardwareSPI.ino

  Description :   Programme permettant de tester le DAC

  Remarques :     --> la mémoire SRAM et l'ADC ne seront pas utilisés ici
                  --> Méthode utilisée : hardware SPI

  Auteur :        Jérôme TOMSKI (https://passionelectronique.fr/)
  Créé le :       24.04.2023

*/

// Lignes SPI de l'Arduino Nano <--> DAC
#define pin_MOSI          11    // Le signal MOSI sort sur la pin D11 de la carte Arduino (cette pin est reliée en interne à la broche PB3 du µC ATmega328P)
#define pin_SCK           13    // Le signal SCK sort sur la pin D13 de la carte Arduino (cette pin est reliée en interne à la broche PB5 du µC ATmega328P)
#define pin_SS_DAC        3     // Le Slave-Select du DAC sort sur la pin D3 de la carte Arduino (cette pin est reliée en interne à la broche PD3 du µC ATmega328P)
#define pin_SS_spi_base   10    // Broche SS du SPI "de base" de l'Arduino ; elle sera déclarée en "sortie", afin de permettre d'activer le mode "SPI Maître"
#define pin_POTENTIOMETRE A0    // La tension présente sur A0, modifiable par potentiomètre, sera envoyée au DAC, pour "reproduction"

// Ligne slave-select du DAC (branchée sur sortie D3 de la carte Arduino, soit la pin PD3 du µC, donc le bit 3 sur PORTD[7..0])
#define selectionner_DAC        PORTD &= 0b11110111
#define desactiver_DAC          PORTD |= 0b00001000

// Fonction "NOP" (pour faire faire une pause au µC, équivalente à 1 cycle d'horloge, soit 62,5 ns, si prescaler non touché)
#define executer_NOP asm volatile ("nop\n\t")


// ===========================================================
// Fonction SETUP (démarrage programme)
// ===========================================================
void setup() {
  pinMode(pin_SS_DAC, OUTPUT);          // Défini la broche arduino comme sortie, pilotant la broche SS du DAC
  desactiver_DAC;                       // ... et la fixe à l'état haut (communication SPI/DAC désactivé)

  pinMode(pin_MOSI, OUTPUT);      digitalWrite(pin_MOSI, LOW);          // Mise à l'état bas de la ligne MOSI (ligne de données maitre vers esclave)
  pinMode(pin_SCK, OUTPUT);       digitalWrite(pin_SCK, LOW);           // Mise à l'état bas de la ligne SCK (horloge SPI)

  pinMode(pin_SS_spi_base, OUTPUT);     // Pin SS "normale", du port SPI arduino    ==> il faut la déclarer en sortie, même si elle n'est pas utilisée
                                        //                                     (sinon, l'arduino ne passera pas en SPI mode master, et restera en slave SPI)

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
  Serial.println(F("Test du DAC modèle MCP 4921, de chez Microchip"));
  Serial.println(F("- Méthode de communication : harware SPI, purement logiciel"));
  Serial.println(F("================================================================================"));
  Serial.println("");

}


// ===========================================================
// Fonction permettant d'écrire un octet sur le bus SPI
// ===========================================================
void ecritureOctetSurBusSPI(uint8_t octetAenvoyer) {

    SPDR = octetAenvoyer;       // Le fait d'envoyer une valeur démarre l'échange SPI (on envoie qu'un octet à la fois)
    while(!(SPSR & (1<<SPIF))); // Attente de la fin de transmission
    return SPDR;
    
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

  // Ajout de 4 bits de configuration du DAC, "à gauche" des 12 bits de données
      // 1er bit de config : toujours à 0
      // 2ème bit de config : 0=unbuffered, 1=buffered (ici choix unbuffured, donc 0)
      // 3ème bit de config : 0=gain double, 1=pas de gain (ici choix "pas de gain", donc 1)
      // 4ème bit de config : 0=sortie DAC éteinte, 1=sortie DAC active (ici choix "sortie du DAC active", donc 1)
      // ... d'où le masque "0011" qui sera rajouté "devant" les 12 bits de poids faible (conservés avec les "111111111111" qui suivent)
  valeurPOT |= 0b0011000000000000;      // On rajoute donc "0011" devant

  // Ecriture de ces 16 bits (pour rappel : 4 bits de configuration, suivis de 12 bits de données)
  ecritureOctetSurBusSPI((uint8_t)(valeurPOT >> 8));                   // On récupère les 8 bits de poids fort (en les décalant à droite)
  ecritureOctetSurBusSPI((uint8_t)(valeurPOT & 0b0000000011111111));   // On récupère les 8 bits de poids faible (en ne gardant que ces derniers)

  // Désactivation de la communication avec le DAC (en remontant sa ligne /SS)
  desactiver_DAC;
 
  delay(1000);   // Petit délai
}
