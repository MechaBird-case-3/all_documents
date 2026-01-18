/*
 * ═══════════════════════════════════════════════════════════════════════════
 * RC + AUTOLEVEL GIMBAL BESTURING VOOR VLIEGEND PLATFORM
 * ═══════════════════════════════════════════════════════════════════════════
 * 
 * SYSTEEM OVERZICHT:
 * -----------------
 * Dit programma bestuurt een 2-assige gimbal op een vliegtuig/drone die kan 
 * schakelen tussen handmatige RC-besturing en automatische stabilisatie.
 * 
 * TWEE WERKINGSMODI:
 * 1. RC MODE: Directe handmatige besturing via afstandsbediening
 * 2. AUTOLEVEL MODE: IMU-gestuurde automatische horizontale stabilisatie
 * 
 * HARDWARE CONFIGURATIE:
 * ---------------------
 * - Arduino Nano 33 IoT met ingebouwde LSM6DS3 IMU (gyroscoop + accelerometer)
 * - 3x Servo motors:
 *     • Linker vleugel aileron (servo + RC input)
 *     • Rechter vleugel aileron (alleen servo)
 *     • Tail/staart voor pitch controle (alleen servo)
 * - RC ontvanger met minimaal 3 kanalen
 * 
 * BELANGRIJKE NOOT OVER PIN GEBRUIK:
 * -----------------------------------
 * • RC_PIN_AILERON (pin 2)  → Leest RC signaal voor RECHTER vleugel
 * • RC_PIN_ELEVATOR (pin 3) → Leest RC signaal voor LINKER vleugel
 * • RC_PIN_RUDDER (pin 4)   → Leest RC signaal (wordt gelezen maar niet gebruikt)
 * 
 * • SERVO_AILERON_L (pin 11) → Stuurt servo aan voor LINKER vleugel
 * • SERVO_AILERON_R (pin 9)  → Stuurt servo aan voor RECHTER vleugel  
 * • SERVO_ELEVATOR (pin 10)  → Stuurt servo aan voor TAIL/PITCH
 * 
 * De naam "ELEVATOR" in de code verwijst naar de LINKER VLEUGEL aileron input,
 * NIET naar een elevator/hoogteroer. Dit is verwarrend maar zo staat het in de code.
 * 
 * WERKINGSPRINCIPE:
 * ----------------
 * De hoofdloop leest continu de RC-signalen van pin 2 en 3. Als één van de 
 * sticks bewogen wordt (buiten de deadband zone van ±60µs), schakelt het 
 * systeem naar RC-mode voor directe besturing. Als beide sticks neutraal 
 * staan, activeert de autolevel-mode die de IMU gebruikt om de gimbal 
 * perfect horizontaal te houden, ongeacht hoe het vliegtuig beweegt.
 * 
 * COMPLEMENTAIR FILTER (96/4 verhouding):
 * ---------------------------------------
 * In autolevel mode wordt een complementair filter gebruikt dat gyroscoop
 * data (snel, maar drifts) combineert met accelerometer data (traag, maar
 * accuraat op lange termijn). De verhouding is 96% gyroscoop en 4% 
 * accelerometer voor optimale stabiliteit zonder drift.
 */

#include <Servo.h>              // Arduino servo bibliotheek voor PWM servo aansturing
#include <Arduino_LSM6DS3.h>    // Bibliotheek voor de LSM6DS3 6-assige IMU sensor
                                 // (3-assige accelerometer + 3-assige gyroscoop)

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTEN - SERVO MECHANISCHE LIMIETEN
// ═══════════════════════════════════════════════════════════════════════════
// 
// Servo's kunnen fysiek 0-180 graden draaien, maar we beperken het bereik
// tot 45-135 graden om:
// 1. Mechanische schade aan de gimbal te voorkomen
// 2. Overbelasting van de servo's te vermijden
// 3. Een symmetrisch bereik van ±45° rond het centrum te creëren

constexpr int SERVO_CENTER = 90;   // Neutrale/horizontale positie
                                    // Dit is de startpositie en rust-positie
                                    
constexpr int SERVO_MIN    = 45;   // Minimale toegestane hoek (90 - 45 = 45°)
                                    // Dit beperkt de beweging naar één kant
                                    
constexpr int SERVO_MAX    = 135;  // Maximale toegestane hoek (90 + 45 = 135°)
                                    // Dit beperkt de beweging naar andere kant
                                    // Totaal bewegingsbereik = 90° (±45° van centrum)

// ═══════════════════════════════════════════════════════════════════════════
// CONSTANTEN - RC PWM SIGNAAL SPECIFICATIES
// ═══════════════════════════════════════════════════════════════════════════
//
// RC ontvangers communiceren via PWM (Pulse Width Modulation) signalen.
// De puls BREEDTE (in microseconden) bepaalt de gewenste positie/waarde.
//
// STANDAARD RC PWM PROTOCOL:
// • 900µs  = Minimale stick positie (volledig naar links/achter)
// • 1500µs = Centrum stick positie (neutraal/midden)
// • 2100µs = Maximale stick positie (volledig naar rechts/voor)
// • 50Hz herhaalfrequentie (elke 20ms wordt het signaal herhaald)

constexpr int RC_MIN_PULSE = 900;   // Ondergrens van geldig RC signaal (microseconden)
                                     // Signalen korter dan dit zijn ongeldig/noise
                                     
constexpr int RC_MAX_PULSE = 2100;  // Bovengrens van geldig RC signaal (microseconden)
                                     // Signalen langer dan dit zijn ongeldig/noise
                                     
constexpr int RC_CENTER   = 1500;   // Centrum waarde = stick in neutrale positie
                                     // Dit is de referentie voor de deadband berekening
                                     
constexpr int RC_DEADBAND = 60;     // Dode zone: ±60µs rond centrum (totaal 120µs breed)
                                     // WAAROM? Dit voorkomt:
                                     // 1. Jitter/trillingen van goedkope potentiometers
                                     // 2. Ongewenste servo bewegingen bij neutrale stick
                                     // 3. Voortdurend schakelen tussen RC/Auto mode
                                     // Binnen 1440-1560µs wordt gezien als "neutraal"
                                     
constexpr int RC_TIMEOUT  = 25000;  // Timeout voor pulseIn() functie (microseconden)
                                     // Als binnen 25ms geen signaal → return 0
                                     // Dit voorkomt dat de code blijft hangen bij
                                     // verlies van RC signaal (uitgeschakelde zender)

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIES - RC ONTVANGER INPUTS (Digitale pins voor pulseIn)
// ═══════════════════════════════════════════════════════════════════════════
//
// Deze pins lezen PWM signalen van de RC ontvanger.
// BELANGRIJK: De namen in comments komen NIET overeen met de functie!

constexpr int RC_PIN_AILERON  = 2;  // Digitale pin 2: Leest RC kanaal voor RECHTER vleugel
                                     // Comment zegt "Aileron Rechts" - dit klopt
                                     
constexpr int RC_PIN_ELEVATOR = 3;  // Digitale pin 3: Leest RC kanaal voor LINKER vleugel
                                     // Comment zegt "Aileron Links" - verwarrende naam
                                     // De naam "ELEVATOR" klopt NIET - dit is linker aileron!
                                     
constexpr int RC_PIN_RUDDER   = 4;  // Digitale pin 4: Leest rudder kanaal
                                     // Dit signaal wordt WEL gelezen in de loop,
                                     // maar NERGENS gebruikt in de code
                                     // Waarschijnlijk voor toekomstige yaw-controle

// ═══════════════════════════════════════════════════════════════════════════
// PIN DEFINITIES - SERVO MOTOR OUTPUTS (PWM pins voor servo aansturing)
// ═══════════════════════════════════════════════════════════════════════════
//
// Deze pins genereren PWM signalen om servo's aan te sturen.
// Arduino Nano 33 IoT heeft PWM op pins: 2,3,4,5,6,7,8,9,10,11,12,13

constexpr int SERVO_AILERON_L = 11;  // PWM pin 11: Stuurt LINKER vleugel aileron servo
                                      // Beweegt de linker vleugel omhoog/omlaag
                                      
constexpr int SERVO_AILERON_R = 9;   // PWM pin 9: Stuurt RECHTER vleugel aileron servo
                                      // Beweegt de rechter vleugel omhoog/omlaag
                                      // Voor roll: beide ailerons bewegen TEGENGESTELD
                                      
constexpr int SERVO_ELEVATOR  = 10;  // PWM pin 10: Stuurt TAIL/STAART servo (pitch as)
                                      // Deze naam is verwarrend want ook gebruikt voor
                                      // de linker aileron INPUT, maar dit is de TAIL servo

// ═══════════════════════════════════════════════════════════════════════════
// SERVO OBJECTEN - Arduino Servo Library Instanties
// ═══════════════════════════════════════════════════════════════════════════
//
// De Arduino Servo bibliotheek vereist een object per servo.
// Elk object beheert:
// • PWM timing (50Hz, 1-2ms pulsbreedte)
// • Positie opslag (0-180 graden)
// • Hardware timer configuratie

Servo servoAileronLeft;   // Object voor linker vleugel aileron servo
                          // Wordt geattached aan pin 11 in setup()
                          
Servo servoAileronRight;  // Object voor rechter vleugel aileron servo
                          // Wordt geattached aan pin 9 in setup()
                          
Servo servoElevator;      // Object voor tail/pitch servo
                          // Wordt geattached aan pin 10 in setup()

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALE VARIABELEN - RC INPUT PULSLENGTES
// ═══════════════════════════════════════════════════════════════════════════
//
// Deze variabelen slaan de gemeten pulslengtes op van pulseIn().
// Type: unsigned long (32-bit, 0 tot 4,294,967,295)
// Waarom unsigned? Tijd/pulslengtes zijn altijd positief

unsigned long pulseAileron;   // Gemeten pulslengte van pin 2 (rechter vleugel RC input)
                              // Bereik: 0 (geen signaal) of 900-2100µs (geldig signaal)
                              // Update frequentie: ~50Hz (elke 20ms in de loop)
                              
unsigned long pulseElevator;  // Gemeten pulslengte van pin 3 (linker vleugel RC input)
                              // Let op: naam "Elevator" is misleidend - dit is aileron!
                              
unsigned long pulseRudder;    // Gemeten pulslengte van pin 4 (rudder kanaal)
                              // Wordt gelezen maar NIET gebruikt in deze code

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALE VARIABELEN - IMU SENSOR DATA (LSM6DS3 6-DOF sensor)
// ═══════════════════════════════════════════════════════════════════════════
//
// De LSM6DS3 IMU bevat twee sensoren:
// 1. 3-assige ACCELEROMETER: Meet lineaire versnelling (inclusief zwaartekracht)
// 2. 3-assige GYROSCOOP: Meet rotatiesnelheid (graden per seconde)
//
// Samen vormen deze een 6-DOF (Degrees of Freedom) systeem voor oriëntatie tracking

// RAW ACCELEROMETER DATA (in g-krachten, 1g = 9.81 m/s²):
float accelX, accelY, accelZ;  // Versnelling in 3D ruimte
                                // accelX: Links (-) / Rechts (+) versnelling
                                // accelY: Achter (-) / Voor (+) versnelling  
                                // accelZ: Omlaag (-) / Omhoog (+) versnelling
                                // Bij stilstand meet de accelerometer zwaartekracht!
                                // Dit wordt gebruikt om pitch en roll te berekenen

// RAW GYROSCOOP DATA (in graden per seconde, °/s):
float gyroX, gyroY, gyroZ;     // Rotatiesnelheid om 3 assen
                                // gyroX: Pitch snelheid (voorover/achterover kantelen)
                                // gyroY: Roll snelheid (links/rechts kantelen)
                                // gyroZ: Yaw snelheid (draaien om verticale as)
                                // Gyroscoop drifts over tijd maar is heel nauwkeurig
                                // op korte termijn (snel, geen lag)

// GEFILTERDE ORIËNTATIE (berekend met complementair filter):
float pitch = 0.0f;            // Huidige pitch hoek in graden
                                // Positief = neus omhoog, Negatief = neus omlaag
                                // Bereik theoretisch -180° tot +180°
                                // Praktisch gebruik: ongeveer -45° tot +45°
                                
float roll  = 0.0f;            // Huidige roll hoek in graden
                                // Positief = rechter vleugel omlaag (naar rechts rollen)
                                // Negatief = linker vleugel omlaag (naar links rollen)
                                // Bereik theoretisch -180° tot +180°
                                // Praktisch gebruik: ongeveer -45° tot +45°

// KALIBRATIE OFFSETS (bepaald tijdens setup):
float pitchOffset = 0.0f;      // Pitch correctie waarde (graden)
                                // Compenseert voor:
                                // - Niet perfect horizontale montage van IMU
                                // - Sensor bias/offset
                                // - Structurele helling van het vliegtuig
                                // Wordt afgetrokken van berekende pitch
                                
float rollOffset  = 0.0f;      // Roll correctie waarde (graden)
                                // Zelfde doel als pitchOffset maar voor roll as
                                // Zorgt dat "horizontaal" echt horizontaal is

// COMPLEMENTAIR FILTER PARAMETERS:
constexpr float FILTER_ALPHA = 0.96;  // Filter gewicht: 96% gyroscoop, 4% accelerometer
                                       // WAAROM 0.96?
                                       // - Gyroscoop is nauwkeurig op korte termijn (snel)
                                       // - Accelerometer is nauwkeurig op lange termijn (geen drift)
                                       // - 96/4 verhouding geeft optimale balans:
                                       //   * Snelle respons (gyro)
                                       //   * Geen drift (accel correctie)
                                       // Formule: nieuweWaarde = α×(oud+gyro×dt) + (1-α)×accel
                                       
unsigned long lastIMUTime = 0;         // Timestamp van vorige IMU update (milliseconden)
                                       // Gebruikt om delta-tijd (dt) te berekenen
                                       // dt is nodig voor gyroscoop integratie:
                                       // hoek += rotatiesnelheid × tijd

// ═══════════════════════════════════════════════════════════════════════════
// GLOBALE VARIABELEN - DEBUG SYSTEEM
// ═══════════════════════════════════════════════════════════════════════════
//
// Debug output via Serial monitor helpt bij:
// - Troubleshooting (waarom werkt het niet?)
// - Tuning (zijn de waardes correct?)
// - Monitoring (wat doet het systeem nu?)

bool debugMode = true;                    // Debug output AAN/UIT schakelaar
                                          // true = print debug info naar Serial
                                          // false = stille werking (sneller)
                                          // Kan runtime geschakeld met 'd' command
                                          
unsigned long lastDebugTime = 0;          // Timestamp van laatste debug print (ms)
                                          // Gebruikt om print frequentie te beperken
                                          
constexpr unsigned long DEBUG_INTERVAL = 200;  // Minimale tijd tussen debug prints (ms)
                                               // 200ms = 5 prints per seconde
                                               // WAAROM beperken?
                                               // - Te veel Serial output vertraagt code
                                               // - Serial.print() duurt ~1ms per regel
                                               // - Leesbaardere output in Serial Monitor

// ═══════════════════════════════════════════════════════════════════════════
// HULPFUNCTIES - RC SIGNAAL VALIDATIE EN VERWERKING
// ═══════════════════════════════════════════════════════════════════════════

// ───────────────────────────────────────────────────────────────────────────
// FUNCTIE: rcValid
// ───────────────────────────────────────────────────────────────────────────
// Controleert of een RC puls binnen het geldige bereik ligt.
//
// PARAMETERS:
//   p: Gemeten pulslengte in microseconden
//
// RETURN:
//   true  = Puls is geldig (tussen 900-2100µs)
//   false = Puls is ongeldig (0, noise, of out-of-range)
//
// GEBRUIK:
//   Voorkomt dat we rare waardes (0, 5000, etc) als input gebruiken
//
inline bool rcValid(int p) {
  return (p > RC_MIN_PULSE && p < RC_MAX_PULSE);
  // Vergelijkt: 900 < p < 2100
  // Let op: NIET >= en <=, dus exacte grenzen zijn ongeldig
}

// ───────────────────────────────────────────────────────────────────────────
// FUNCTIE: rcOutsideDeadband
// ───────────────────────────────────────────────────────────────────────────
// Controleert of RC stick buiten de dode zone (deadband) staat.
// Dit bepaalt of we RC-mode moeten activeren.
//
// PARAMETERS:
//   p: Gemeten pulslengte in microseconden
//
// RETURN:
//   true  = Stick is bewogen (buiten centrum ±60µs) EN signaal is geldig
//   false = Stick is neutraal OF signaal is ongeldig
//
// LOGICA:
//   Voor 1500µs centrum met 60µs deadband:
//   - 1440µs tot 1560µs = BINNEN deadband (false)
//   - <1440µs of >1560µs = BUITEN deadband (true, als geldig)
//
inline bool rcOutsideDeadband(int p) {
  return rcValid(p) && abs(p - RC_CENTER) > RC_DEADBAND;
  //     ^^^^^^^^^^    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //     Eerst: is signaal geldig?
  //                  Dan: is afstand tot centrum > 60?
  //
  // abs() = absolute waarde (maakt negatief positief)
  // Voorbeeld: abs(1400 - 1500) = abs(-100) = 100 > 60 ✓
  //            abs(1520 - 1500) = abs(20) = 20 > 60 ✗
}

// ───────────────────────────────────────────────────────────────────────────
// FUNCTIE: rcApplyDeadband
// ───────────────────────────────────────────────────────────────────────────
// Past deadband toe: pulsen binnen de dode zone worden geforceerd naar centrum.
// Dit elimineert jitter en zorgt voor een echte neutrale positie.
//
// PARAMETERS:
//   p: Gemeten pulslengte in microseconden
//
// RETURN:
//   RC_CENTER (1500) als binnen deadband of ongeldig
//   Originele waarde (p) als buiten deadband
//
// VOORBEELDEN:
//   rcApplyDeadband(0)    → 1500 (ongeldig signaal)
//   rcApplyDeadband(1450) → 1500 (binnen deadband: |1450-1500|=50 < 60)
//   rcApplyDeadband(1510) → 1500 (binnen deadband: |1510-1500|=10 < 60)
//   rcApplyDeadband(1650) → 1650 (buiten deadband: |1650-1500|=150 > 60)
//   rcApplyDeadband(900)  → 900  (buiten deadband, geldige minimale waarde)
//
inline int rcApplyDeadband(int p) {
  if (!rcValid(p)) return RC_CENTER;                    // Ongeldig → centrum
  if (abs(p - RC_CENTER) <= RC_DEADBAND) return RC_CENTER;  // Binnen zone → centrum
  return p;                                             // Buiten zone → echte waarde
}

// ───────────────────────────────────────────────────────────────────────────
// FUNCTIE: servoLimit
// ───────────────────────────────────────────────────────────────────────────
// Beperkt servo waarden tot veilig mechanisch bereik (45°-135°).
// Dit voorkomt schade aan servo's en gimbal mechaniek.
//
// PARAMETERS:
//   v: Gewenste servo positie in graden
//
// RETURN:
//   Beperkte waarde tussen SERVO_MIN (45) en SERVO_MAX (135)
//
// VOORBEELDEN:
//   servoLimit(30)  → 45  (te laag, afgekapt naar minimum)
//   servoLimit(90)  → 90  (binnen bereik, ongewijzigd)
//   servoLimit(150) → 135 (te hoog, afgekapt naar maximum)
//
// WAAROM NODIG?
//   - Berekeningen kunnen waardes buiten 45-135 opleveren
//   - Zonder limiet kunnen servo's vastlopen of stuk gaan
//   - Gimbal mechaniek heeft fysieke stops
//
inline int servoLimit(int v) {
  return constrain(v, SERVO_MIN, SERVO_MAX);
  //     ^^^^^^^^^^^  ^^^^^^^^^^^  ^^^^^^^^^
  //     Arduino functie  min=45    max=135
  //
  // constrain(waarde, min, max) = clamp functie
  // Als waarde < min → return min
  // Als waarde > max → return max
  // Anders → return waarde
}

// ═══════════════════════════════════════════════════════════════════════════
// SETUP FUNCTIE - EENMALIGE INITIALISATIE BIJ OPSTARTEN
// ═══════════════════════════════════════════════════════════════════════════
//
// Deze functie wordt EENMAAL uitgevoerd wanneer Arduino opstart of reset wordt.
// Hier initialiseren we alle hardware en kalibreren we de IMU.
//
void setup() {
  // ─────────────────────────────────────────────────────────────────────────
  // SERIAL COMMUNICATIE SETUP
  // ─────────────────────────────────────────────────────────────────────────
  Serial.begin(9600);              // Start Serial communicatie op 9600 baud
                                   // 9600 baud = 9600 bits per seconde
                                   // = ongeveer 960 karakters per seconde
                                   // Standaard snelheid voor Arduino debugging
                                   
  delay(1000);                     // Wacht 1 seconde (1000 milliseconden)
                                   // WAAROM? Serial monitor heeft tijd nodig om
                                   // te starten, anders missen we eerste berichten

  Serial.println("\n=== RC + AUTOLEVEL GIMBAL ===");
  // Print header naar Serial Monitor
  // \n = newline (lege regel ervoor voor overzicht)

  // ─────────────────────────────────────────────────────────────────────────
  // IMU SENSOR INITIALISATIE
  // ─────────────────────────────────────────────────────────────────────────
  if (!IMU.begin()) {              // Probeer IMU te starten
    //  ^^^^^^^^^^^
    //  Returns: true = succes, false = fout
    //  ! = NOT operator, dus if(NOT succes) = if(fout)
    
    Serial.println("IMU FOUT");   // Print foutmelding
    
    while (true);                  // ONEINDIGE LOOP = STOP PROGRAMMA
    //    ^^^^                     // Dit is een FAILSAFE
    //    Conditie is altijd waar  // Als IMU niet werkt, kunnen we niet autolevel
                                   // Beter stoppen dan fout gedrag
                                   // Rode LED op Arduino blijft branden
                                   // Gebruiker weet: er is een probleem
  }
  // Als we hier komen, is IMU succesvol gestart ✓

  // ─────────────────────────────────────────────────────────────────────────
  // RC ONTVANGER PINNEN CONFIGUREREN
  // ─────────────────────────────────────────────────────────────────────────
  pinMode(RC_PIN_AILERON, INPUT);   // Pin 2: INPUT mode (rechter vleugel RC)
  pinMode(RC_PIN_ELEVATOR, INPUT);  // Pin 3: INPUT mode (linker vleugel RC)
  pinMode(RC_PIN_RUDDER, INPUT);    // Pin 4: INPUT mode (rudder, niet gebruikt)
  
  // pinMode(pin, INPUT) configureert pin als:
  // - Hoge impedantie (leest signaal zonder stroom te trekken)
  // - Geen pull-up/pull-down weerstand
  // - Geschikt voor PWM signalen van RC ontvanger

  // ─────────────────────────────────────────────────────────────────────────
  // SERVO'S KOPPELEN AAN PINNEN EN CENTREREN
  // ─────────────────────────────────────────────────────────────────────────
  servoAileronLeft.attach(SERVO_AILERON_L);   // Koppel servo object aan pin 11
  servoAileronRight.attach(SERVO_AILERON_R);  // Koppel servo object aan pin 9
  servoElevator.attach(SERVO_ELEVATOR);       // Koppel servo object aan pin 10
  
  // attach() doet:
  // 1. Reserveert hardware timer voor PWM generatie
  // 2. Configureert pin als OUTPUT
  // 3. Start 50Hz PWM signaal (20ms periode)
  // 4. Maakt servo klaar om commando's te ontvangen

  servoAileronLeft.write(SERVO_CENTER);    // Zet linker aileron naar 90°
  servoAileronRight.write(SERVO_CENTER);   // Zet rechter aileron naar 90°
  servoElevator.write(SERVO_CENTER);       // Zet tail naar 90°
  
  // write() stuurt servo naar opgegeven positie (0-180°)
  // SERVO_CENTER = 90° = neutrale/horizontale positie
  // Dit zorgt voor een veilige startpositie bij opstarten
  // Gimbal staat nu horizontaal, niet scheef

  // ─────────────────────────────────────────────────────────────────────────
  // IMU KALIBRATIE UITVOEREN
  // ─────────────────────────────────────────────────────────────────────────
  calibrateIMU();                  // Roep kalibratie functie aan (zie verderop)
                                   // Dit neemt 100 metingen over ~2 seconden
                                   // Berekent pitchOffset en rollOffset
                                   // KRITIEK: Gimbal moet STIL en HORIZONTAAL zijn!
                                   
  lastIMUTime = millis();          // Zet starttijd voor delta-tijd berekening
  // millis() = aantal milliseconden sinds Arduino opstart
  // Deze waarde wordt gebruikt in loop() om dt te berekenen

  Serial.println("Systeem klaar");  // Bevestiging: setup compleet
  // Nu begint de loop() te draaien
}

// ═══════════════════════════════════════════════════════════════════════════
// MAIN LOOP - CONTINUES UITVOERING (50-100x per seconde)
// ═══════════════════════════════════════════════════════════════════════════
//
// Deze functie wordt ONEINDIG herhaald na setup().
// Hier vindt de hoofdlogica plaats: RC lezen → Mode kiezen → Servo's aansturen
//
void loop() {
  // ─────────────────────────────────────────────────────────────────────────
  // CHECK DEBUG TOGGLE COMMANDO
  // ─────────────────────────────────────────────────────────────────────────
  handleDebugToggle();             // Controleer of gebruiker 'd' heeft getypt
                                   // Zo ja: schakel debugMode aan/uit
                                   // Dit blokkeert niet (non-blocking check)

  // ─────────────────────────────────────────────────────────────────────────
  // LEES ALLE RC KANALEN
  // ─────────────────────────────────────────────────────────────────────────
  pulseAileron  = pulseIn(RC_PIN_AILERON, HIGH, RC_TIMEOUT);
  // pulseIn(pin, HIGH, timeout):
  // - Wacht tot pin HIGH wordt
  // - Meet hoe lang pin HIGH blijft
  // - Return: pulslengte in microseconden
  // - Als timeout (25ms) bereikt: return 0
  // - Dit BLOKKEERT de code tijdens meting!
  // - Typische RC puls: