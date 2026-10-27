/**
 * @file main.c
 * @brief TOF-Distanzmessung mit DAVE 4, OLED, PWM und CAN-Ausgabe.
 *
 * @details Nach dem Einschalten richtet main() die Hardware ein und startet
 * den VL53L0X und CAN0. In einer Endlosschleife werden Messung, CAN, Anzeige, LED und PWM
 * zu unterschiedlichen Zeitpunkten bearbeitet. Ein Timer zaehlt dazu die
 * vergangenen Millisekunden. Die Vorlage stammt aus TOFTest/Src/main.c
 * des STM32-Cube-Projekts; hier werden DAVE-APPs verwendet.
 *
 * @par PWM0 als abstandsabhaengiger Sollwert fuer die Geblaeseansteuerung
 * Die DAVE-APP PWM_0 gibt das PWM-Signal an P0.5 aus. Je kleiner der geglaettete
 * Abstand distance_mean_mm, desto groesser der berechnete Tastgrad. Ein Tastgrad
 * von 50 Prozent bedeutet, dass das Signal innerhalb jeder PWM-Periode halb so
 * lange aktiv wie die gesamte Periode ist; er ist keine gemessene Geblaesedrehzahl.
 * Die Anwendung berechnet den Sollwert nach jeweils mindestens 200 ms neu.
 * Dieses Aktualisierungsintervall ist nicht die Periodendauer des PWM-Signals;
 * die Hardware erzeugt die PWM zwischen den Aktualisierungen selbststaendig.
 *
 * Fuer einen Abstand d bis MAX_HEIGHT_mm gilt mit ganzzahliger Division:
 * PWM_Value = 100 * (MAX_HEIGHT_mm - d) / MAX_HEIGHT_mm.
 * Bei MAX_HEIGHT_mm = 300 ergeben sich beispielsweise:
 * - 0 mm: 100 Prozent, DAVE-Wert 10000.
 * - 150 mm: 50 Prozent, DAVE-Wert 5000.
 * - 300 mm und groessere Abstaende: 0 Prozent, DAVE-Wert 0.
 *
 * @par Start- und Fehlerverhalten des PWM-Ausgangs
 * Die aktuelle DAVE-Konfiguration startet PWM_0 bereits bei DAVE_Init() mit
 * 50 Prozent Tastgrad. Der erste abstandsabhaengige Sollwert wird erst nach
 * der Initialisierung in der Hauptschleife gesetzt. Der Ersatzwert
 * TOF_VL53L0X_OUT_OF_RANGE (8190) liegt oberhalb der aktuellen Hoehengrenze
 * und ergibt beim naechsten PWM-Update 0 Prozent. Bei einem I2C-Lesefehler
 * bleibt dagegen der letzte Mittelwert erhalten: Der PWM-Zweig prueft
 * initedTOF nicht und berechnet weiter den Sollwert aus diesem alten Wert.
 * Ein Initialisierungs-Halt vor der Hauptschleife setzt keinen neuen Tastgrad.
 *
 * @par Lesereihenfolge fuer Einsteiger
 * Zuerst main() und TOFSensor_t lesen, dann TOF_init() und
 * TOF_read_continuous_distance(). Die Einzelheiten des I2C-Protokolls
 * stehen in tof_dave_port.inc.
 *
 * @note Der Timer muss auch bei I2C-Zugriffen weiterlaufen, damit Zeitlimits
 * erkannt werden. Deshalb findet im Timer-Callback keine Buskommunikation statt.
 */
#include "DAVE.h"
#include "Libraries/CAN/CANlib.h"
#include "Libraries/TOF/i2cTOF.h"
#include "Libraries/OLED/oled.h"
#include "Libraries/OLED/gui.h"
#include "Libraries/OLED/test.h"
#include <stdio.h>

/** @brief Port/Pin-Paar der LED fuer das Lebenszeichen (P0.7). */
#define LED1 XMC_GPIO_PORT0, 7
/** @brief Port/Pin-Paar der Fehler-LED (P0.6). */
#define LED2 XMC_GPIO_PORT0, 6
/** @brief Standardprofil mit 30 ms Messbudget. */
#define TOF_PROFILE TOF_DEFAULT_MODE_D

/** @brief Aktualisierung des PWM-Ausgabewertes in Millisekunden. */
#define PWMset_Periode_ms 100U

/** @brief Mindestabstand der CAN-Sendeversuche in Millisekunden. */
#define TOF_CAN_PERIOD_MS 50U
/** @brief Ersatzwert bei fehlender/ungueltiger Messung; keine Entfernung. */
#define TOF_CAN_INVALID_MM UINT16_MAX

/** @brief Unter diesem Rohabstand in mm wird das Oszilloskop angezeigt. */
#define TOF_SCOPE_LIMIT_MM 300U
/** @brief Zustand des laufenden OLED-Plots; keine gespeicherte Messwertliste. */
static GUI_Scope_t tof_scope;
/** @brief Sperrt Textausgaben, solange die Grafik den Bildschirm nutzt. */
static bool scope_active = false;

/** @brief Anzahl gueltiger Messwerte im gleitenden Mittelwert. */
#define TOF_MEAN_WINDOW_SIZE 4U

/* Die Summe passt bei maximal 65535 Werten vom Typ uint16_t in uint32_t. */
#if (TOF_MEAN_WINDOW_SIZE < 1U) || (TOF_MEAN_WINDOW_SIZE > 65535U)
#error "TOF_MEAN_WINDOW_SIZE muss zwischen 1 und 65535 liegen"
#endif
//#define TOF_PROFILE TOF_HIGH_ACCURACY_MODE_A


/**
 * @brief Zeitabstand der Distanzanzeige in ms: vier nominelle Messintervalle.
 * @details Im Standardprofil sind das 4 * (30 + 5) = 140 ms. Es werden keine
 * Messwerte gezaehlt; geprueft wird die verstrichene Zeit. Die PWM-Prozentanzeige
 * wird unabhaengig davon im PWM-Zweig nach jeweils mindestens 200 ms erneuert.
 */
#define DisplayRate 4*(TOF_Sensor_1.Ranging_Profile_time + 5U)

/**
 * @brief Obere Abstandsgrenze in mm fuer einen positiven PWM-Sollwert.
 * @details Unterhalb der Grenze nimmt der Tastgrad bei kleinerem Abstand linear
 * zu. Genau an der Grenze und darueber ist er 0 Prozent. Die Bezeichnung Hoehe
 * meint hier unmittelbar den vom TOF-Sensor gemessenen, geglaetteten Abstand;
 * eine Umrechnung auf eine andere geometrische Bezugshoehe findet nicht statt.
 * @pre Der Wert muss groesser als 0 sein, da durch ihn dividiert wird.
 * Fuer die bestehende Behandlung ungueltiger Messungen muss er ausserdem
 * kleiner als TOF_VL53L0X_OUT_OF_RANGE bleiben.
 */
#define MAX_HEIGHT_mm 300U

/**
 * @brief Seit Timerstart vergangene Millisekunden.
 * @details volatile zeigt dem Compiler, dass ein Interrupt den Wert aendern
 * kann. Es ersetzt keine Sperre fuer komplexe gemeinsame Datenzugriffe.
 */
volatile uint32_t SysTickCounter = 0U;
/** @brief Bus, Einstellungen und letzter Messwert des einzigen Sensors. */
TOFSensor_t TOF_Sensor_1;
/** @brief true, solange Initialisierung und Messbetrieb erfolgreich sind. */
volatile bool initedTOF = false;
/** @brief Anzahl erkannter TOF-Fehler; im Debugger beobachtbar. */
volatile uint32_t tof_error_count = 0U;

/** @brief Anzahl angenommener CAN-Auftraege, keine Anzahl bestaetigter Frames. */
volatile uint32_t can_tx_accepted_count = 0U;
/** @brief Anzahl wegen belegtem Sendeobjekt ausgelassener Sendeintervalle. */
volatile uint32_t can_tx_busy_count = 0U;
/** @brief Anzahl abgelehnter Auftraege (z.B. Bus-off). */
volatile uint32_t can_tx_error_count = 0U;

/**
 * @brief Packt den TOF-Wert in eine 8-Byte-Botschaft und startet den Versand.
 * @param[in] distance_mm Geglaettete Entfernung in mm; 65535 bedeutet ungueltig.
 * @return Ergebnis der Auftragsuebergabe an CANLIB_Send().
 * @details Byte 0 Bit 0..3: Botschaftszaehler, Bit 4..7: reserviert = 0.
 * Byte 1: niederwertiges Byte der Entfernung; Byte 2: hoeherwertiges Byte.
 * Byte 3..7: reserviert = 0. Der uint16_t-Wert ist somit Little Endian.
 * Beispiel: Zaehler 5, Abstand 300 mm -> 05 2C 01 00 00 00 00 00.
 * Der erste Auftrag nutzt Zaehler 0. Nur bei angenommenem Auftrag wird
 * modulo 16 weitergezaehlt. Hardware-Wiederholungen behalten dieselben Daten.
 * Bei BUSY wird dieses Intervall ausgelassen; beim naechsten Versuch wird
 * der aktuelle Messwert verwendet. Kein blockierendes Warten auf CAN.
 */
static CANLIB_TxResult_t TOF_CAN_SendDistance(uint16_t distance_mm)
{
    static uint8_t message_counter = 0U;
    uint8_t data[8] = {0};
    CANLIB_TxResult_t result;
    data[0] = message_counter & 0x0FU;
    data[1] = (uint8_t)(distance_mm & 0xFFU);
    data[2] = (uint8_t)(distance_mm >> 8U);
    result = CANLIB_Send(data, sizeof(data));
    if (result == CANLIB_TX_ACCEPTED) {
        message_counter = (uint8_t)((message_counter + 1U) & 0x0FU);
        ++can_tx_accepted_count;
    } else if (result == CANLIB_TX_BUSY) {
        ++can_tx_busy_count;
    } else {
        ++can_tx_error_count;
    }
    return result;
}

/**
 * @brief Berechnet den gleitenden Mittelwert der letzten gueltigen TOF-Messwerte.
 * @param[in] distance_mm Neuer Messwert in mm oder TOF_VL53L0X_OUT_OF_RANGE.
 * @return Auf ganze Millimeter gerundeter Mittelwert; bei ungueltigem Eingang
 *         TOF_VL53L0X_OUT_OF_RANGE, ohne den bisherigen Verlauf zu veraendern.
 *
 * @details Ein Ringpuffer speichert bis zu TOF_MEAN_WINDOW_SIZE Werte. Ist er
 * voll, ersetzt jeder neue Messwert den aeltesten. Die Summe wird dabei durch
 * Abziehen des alten und Addieren des neuen Wertes aktualisiert. Dadurch muss
 * nicht bei jedem Aufruf der gesamte Puffer erneut addiert werden.
 *
 * Beim Start wird nur durch die Anzahl bereits vorhandener Werte geteilt:
 * Der erste Mittelwert entspricht daher genau dem ersten Messwert. Fehlende
 * Werte werden nicht als Null mitgezaehlt. Ungueltige Werte werden uebersprungen;
 * nach einer Messluecke bleiben die letzten gueltigen Werte im Fenster erhalten.
 *
 * @note Genau einmal je erfolgreich gelesener Messung im Hauptprogramm aufrufen,
 * nicht bei jeder Displayaktualisierung. Die statischen Variablen behalten ihren
 * Inhalt zwischen Aufrufen. Der Filter gehoert zu einem einzigen Sensor und wird
 * beim Programmstart automatisch mit Null initialisiert.
 */
static uint16_t TOF_MovingAverage(uint16_t distance_mm)
{
    static uint16_t samples[TOF_MEAN_WINDOW_SIZE] = {0};
    static uint32_t sum = 0U;
    static uint16_t next_index = 0U;
    static uint16_t sample_count = 0U;

    /* Reichweitenfehler sind keine Entfernungen und duerfen die Summe nicht aendern. */
    if (distance_mm == TOF_VL53L0X_OUT_OF_RANGE) {
        return TOF_VL53L0X_OUT_OF_RANGE;
    }

    if (sample_count < TOF_MEAN_WINDOW_SIZE) {
        ++sample_count;
    } else {
        /* Am Schreibplatz steht bei vollem Puffer der aelteste Messwert. */
        sum -= samples[next_index];
    }
    samples[next_index] = distance_mm;
    sum += distance_mm;

    /* Nach dem letzten Feld wieder beim ersten beginnen: ein Ringpuffer. */
    ++next_index;
    if (next_index >= TOF_MEAN_WINDOW_SIZE) {
        next_index = 0U;
    }

    /* Halben Divisor addieren: Ganzzahldivision rundet so zum naechsten mm.
     * sample_count ist hier mindestens 1; eine Division durch 0 ist ausgeschlossen. */
    return (uint16_t)((sum + sample_count / 2U) / sample_count);
}

/**
 * @brief Timer-Callback: erhoeht jede Millisekunde den Zeitzaehler.
 * @param[in] args Von SYSTIMER uebergebene Benutzerdaten; hier unbenutzt.
 * @details Ein Callback ist eine Funktion, die von anderer Software aufgerufen
 * wird. Hier uebernimmt das die SYSTIMER-APP im Interruptkontext.
 */
static void milliSec_Handler(void *args)
{
    /* Bewusst unbenutzt: verhindert eine entsprechende Compilerwarnung. */
    (void)args;
    ++SysTickCounter;
}

/**
 * @brief Schaltet bei einem vom DAVE-Handler gemeldeten NACK die Fehler-LED ein.
 * @details NACK bedeutet, dass der angesprochene Teilnehmer nicht bestaetigt.
 * Der Funktionsname ist in der generierten DAVE-Konfiguration hinterlegt.
 * Der TOF-Port prueft seine Fehler selbst und maskiert diesen Interrupt
 * waehrend seiner eigenen Transfers.
 */
void NACK_Handler(void)
{
    XMC_GPIO_SetOutputHigh(LED2);
}

/**
 * @brief Schreibt eine Statuszeile oder Entfernung auf das OLED.
 * @param[in] text Nullterminierte Zeichenkette; passend zur Displaybreite.
 * @details y=2 bezeichnet die dritte 8-Pixel-Seite des Displays.
 * Die Schrift ist 16 Pixel hoch. Leerzeichen am Zeilenende loeschen
 * Zeichen eines vorherigen, laengeren Textes.
 */
static void show_status(char *text)
{
    GUI_ShowString(0, 2, text, 16, 1);
}


/**
 * @brief Schaltet die Ansicht um und zeichnet genau eine neue TOF-Messung.
 * @param[in] raw_mm Ungefilterter Abstand in mm; Ersatzwert beendet die Grafik.
 * @details Unter 300 mm: Rohwert auf einer Skala 0..300 mm auftragen.
 * Ab 300 mm oder bei Fehlern: normale Textanzeige wiederherstellen.
 * Ein Pixel pro Messung, nominal 35 ms im Standardprofil (keine Echtzeitgarantie).
 * Ein kompletter Umlauf umfasst 128 Messungen, nominal etwa 4,48 Sekunden.
 * Der Schwellwert hat bewusst keine Hysterese. Nur bei neuen Messungen aufrufen.
 */
static void TOF_UpdateScope(uint16_t raw_mm)
{
    bool want_scope = raw_mm < TOF_SCOPE_LIMIT_MM;
    if (want_scope != scope_active) {
        /* Ein Vollbildloeschen ist nur beim Ansichtswechsel erforderlich. */
        OLED_Clear(0U);
        if (want_scope) {
            scope_active = GUI_ScopeInit(&tof_scope, 0U, 1U,
                                         OLED_WIDTH, 6U, TOF_SCOPE_LIMIT_MM);
            GUI_ShowString(0, 0, "TOF roh: 0..300mm", 8, 1);
            GUI_ShowString(0, 7, "t -> 1px/Messung", 8, 1);
        } else {
            scope_active = false;
            GUI_ShowString(0, 0, "TOF + CAN", 16, 1);
        }
    }
    if (scope_active) { GUI_ScopePlot(&tof_scope, raw_mm); }
}

/**
 * @brief Initialisiert das Board und bearbeitet die regelmaessigen Aufgaben.
 * @return Kein regulaerer Ruecksprung: die Anwendung bleibt in der Endlosschleife.
 * @details Der Startcode des Controllers ruft diese Funktion auf.
 * Ein fehlgeschlagener Hardware- oder Timerstart fuehrt zum Halt mit Fehler-LED.
 * Bei einem TOF-Fehler bleibt dagegen die Hauptschleife mit Fehleranzeige aktiv.
 */
int main(void)
{
    /* const: Diese gemeinsam fuer beide LEDs verwendeten Einstellungen bleiben gleich. */
    const XMC_GPIO_CONFIG_t led_config = {
        .mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
        .output_level = XMC_GPIO_OUTPUT_LEVEL_LOW
    };
    /* DAVE_Init() richtet die im Projekt konfigurierten APPs ein. */
    DAVE_STATUS_t status = DAVE_Init();
    uint32_t timer_id;
    /* Zeitstempel der letzten Ausfuehrung, keine herunterzaehlenden Timer. */
    uint32_t last_measurement = 0U;
    uint32_t last_display = 0U;
    uint32_t last_led = 0U;
    /* Zeitstempel des letzten PWM-Sollwertupdates; Mindestabstand 200 ms. */
    uint32_t last_pwm = 0U;
    /* CAN bekommt einen eigenen Zeitstempel, unabhaengig von der Anzeige. */
    uint32_t last_can = 0U;
    bool can_ready;
    CANLIB_TxResult_t can_result = CANLIB_TX_ERROR;
    /* Gemeinsamer Mittelwert fuer Distanzanzeige und PWM-Sollwert.
     * distanceFromTOF bleibt der ungefilterte Sensorwert. */
    uint16_t distance_mean_mm = TOF_VL53L0X_OUT_OF_RANGE;
    /* Tastgrad in ganzen Prozent (0 bis 100), zugleich Wert der Prozentanzeige.
     * Erst beim DAVE-Aufruf wird mit 100 auf den Bereich 0 bis 10000 skaliert. */
    uint16_t PWM_Value;
    /* Platz fuer bis zu 21 Zeichen und das abschliessende Nullzeichen. */
    char line[22];

    XMC_GPIO_Init(LED1, &led_config);
    XMC_GPIO_Init(LED2, &led_config);
    if (status != DAVE_STATUS_SUCCESS) {
        XMC_GPIO_SetOutputHigh(LED2);
        /* Ohne funktionierende Hardware oder Zeitbasis hier bewusst anhalten. */
        while (1U) { }
    }
    can_ready = CANLIB_Init();
    if (!can_ready) {
        XMC_GPIO_SetOutputHigh(LED2);
    }
    /* SYSTIMER erwartet Mikrosekunden: 1000 us entsprechen 1 ms.
     * Die zurueckgegebene ID identifiziert den angelegten Softwaretimer. */
    timer_id = SYSTIMER_CreateTimer(1000U, SYSTIMER_MODE_PERIODIC,
                                   milliSec_Handler, NULL);
    if ((timer_id == 0U) || (SYSTIMER_StartTimer(timer_id) != SYSTIMER_STATUS_SUCCESS)) {
        XMC_GPIO_SetOutputHigh(LED2);
        /* Ohne funktionierende Hardware oder Zeitbasis hier bewusst anhalten. */
        while (1U) { }
    }
    /* Erst warten, wenn der Zeitzaehler laeuft; Sensor und Display starten mit. */
    delay_ms(500U);
    OLED_Init();
    OLED_Clear(0);
    GUI_ShowString(0, 0, "TOF + CAN", 16, 1);
    show_status("Initialisierung ");

    /* & liefert die Adresse einer Variablen. So kann die Bibliothek die
     * Originalstruktur beschreiben und das vorhandene DAVE-Handle verwenden. */
    initTOFSensorData(&TOF_Sensor_1, &I2C_MASTER_0,
                      TOF_ADDR_VL53LOX, TOF_PROFILE, 0U);
    /* Reihenfolge: Sensor pruefen/kalibrieren, Profil setzen, Dauermessung starten.
     * Jeder folgende Schritt wird nur nach Erfolg des vorherigen ausgefuehrt. */
    initedTOF = TOF_init(&TOF_Sensor_1);
    if (initedTOF) {
        initedTOF = TOF_set_ranging_profile(&TOF_Sensor_1);
    }
    if (initedTOF) {
        TOF_Sensor_1.enableTOFSensor = true;
        initedTOF = TOF_start_continuous(&TOF_Sensor_1);
    }
    if (!initedTOF) {
        TOF_Sensor_1.enableTOFSensor = false;
        ++tof_error_count;
        XMC_GPIO_SetOutputHigh(LED2);
        show_status("TOF Init-Fehler  ");
    }

    /* 1U ist immer wahr: Ein Mikrocontroller arbeitet normalerweise dauerhaft. */
    while (1U) {
        /* Einmal lesen: Alle Aufgaben dieses Durchlaufs nutzen denselben Zeitwert.
         * Unsigned-Subtraktion funktioniert auch ueber den Zaehlerueberlauf,
         * solange die betrachteten Zeitabstaende kleiner als ein Umlauf sind. */
        uint32_t now = SysTickCounter;
        /* && verlangt beide Bedingungen: Sensor bereit UND genug Zeit vergangen.
         * Die 5 ms Zugabe sind ein kleiner Abstand zum vorgesehenen Messbudget. */
        if (initedTOF && (uint32_t)(now - last_measurement) >=
                         TOF_Sensor_1.Ranging_Profile_time + 5U) {
            last_measurement = now;
            /* ! kehrt den Wahrheitswert um: Dieser Zweig behandelt einen Fehler. */
            if (!TOF_read_continuous_distance(&TOF_Sensor_1)) {
                ++tof_error_count;
                initedTOF = false;
                TOF_Sensor_1.enableTOFSensor = false;
                XMC_GPIO_SetOutputHigh(LED2);
                TOF_UpdateScope(TOF_VL53L0X_OUT_OF_RANGE);
                show_status("TOF Lesefehler   ");
            } else {
                /* Jede neue Messung genau einmal in die Mittelwertbildung geben. */
                distance_mean_mm = TOF_MovingAverage(TOF_Sensor_1.distanceFromTOF);
                TOF_UpdateScope(TOF_Sensor_1.distanceFromTOF);
            }
        }
        /* CAN sendet auch bei Sensorfehler einen eindeutig ungueltigen Wert.
         * Alte Entfernungen werden damit nicht als aktuelle Messung ausgegeben. */
        if ((uint32_t)(now - last_can) >= TOF_CAN_PERIOD_MS) {
            uint16_t can_distance = (initedTOF &&
                distance_mean_mm != TOF_VL53L0X_OUT_OF_RANGE)
                ? distance_mean_mm : TOF_CAN_INVALID_MM;
            last_can = now;
            if (can_ready) {
                can_result = TOF_CAN_SendDistance(can_distance);
            }
        }
        /* Langsamere Anzeige spart Buszeit; die Messung darf haeufiger erfolgen. */
        if (!scope_active && (uint32_t)(now - last_display) >= DisplayRate) {
            last_display = now;
            if (initedTOF) {
                if (distance_mean_mm == TOF_VL53L0X_OUT_OF_RANGE) {
                    show_status("Ausser Reichw.   ");
                } else {
                    /* snprintf begrenzt die Ausgabe auf die Puffergroesse.
                     * %5u formatiert eine positive Ganzzahl mit mindestens 5 Stellen. */
                    snprintf(line, sizeof(line), "%5u mm        ",
                             (unsigned int)distance_mean_mm);
                    show_status(line);
                }
            }
            /* Auftrag angenommen bedeutet noch nicht, dass ein Empfaenger ACK gab. */
            GUI_ShowString(0, 6, !can_ready ? "CAN Init-Fehler       " :
                CANLIB_GetState() == CANLIB_STATE_BUS_OFF ? "CAN Bus-off          " :
                can_result == CANLIB_TX_BUSY ? "CAN TX busy        " :
                can_result == CANLIB_TX_ACCEPTED ? "CAN TX sent    " :
                "CAN TX Fehler        ", 8, 1);
            snprintf(line, sizeof(line), "Zeit: %6lu s", (unsigned long)(now / 1000U));
            GUI_ShowString(0, OLED_HEIGHT / 8 - 1, line, 8, 1);
        }
        if ((uint32_t)(now - last_led) >= 1000U) {
            last_led = now;
            XMC_GPIO_ToggleOutput(LED1);
        }
        /* Abstandsabhaengigen PWM-Sollwert nach mindestens PWMset_Periode_ms aktualisieren.
         * Blockierende Sensor- oder Displayzugriffe koennen das Intervall verlaengern.
         * Dieser Zweig laeuft auch bei initedTOF == false. Bei einem Lesefehler
         * bleibt deshalb der zuvor gespeicherte Mittelwert fuer PWM massgeblich. */
        if ((uint32_t)(now - last_pwm) >= PWMset_Periode_ms)
        {
            last_pwm = now;
            /* Die Bereichspruefung verhindert eine negative Differenz bei
             * vorzeichenlosen Zahlen. Am Grenzwert selbst ist die Differenz 0. */
            if (distance_mean_mm <= MAX_HEIGHT_mm)
            {
                /* Zuerst multiplizieren, dann dividieren, um vorzeitigen
                 * Genauigkeitsverlust zu vermeiden. Nachkommastellen entfallen:
                 * z.B. 100 mm bei 300 mm Grenze ergeben 66 statt 66,67 Prozent. */
            	PWM_Value = 100U*(MAX_HEIGHT_mm-distance_mean_mm)/MAX_HEIGHT_mm;

            }
            else
            {
                /* Abstand oberhalb der Grenze, einschliesslich Ersatzwert 8190. */
            	PWM_Value = 0U;
            }
             /* DAVE erwartet Hundertstel-Prozent: 1 Prozent entspricht 100.
              * Der Aufruf aendert den Tastgrad, nicht die konfigurierte PWM-Frequenz. */
             PWM_SetDutyCycle(&PWM_0, 100*PWM_Value);
             /* %3u: Prozentzahl mit mindestens drei Stellen; %%: ein Prozentzeichen.
              * Angezeigt wird der berechnete Sollwert, keine Rueckmessung am Ausgang.
              * Die Ausgabe steht auf OLED-Seite 5 (bei 64 Pixel Displayhoehe). */
             if (!scope_active) {
                 snprintf(line, sizeof(line), "%3u %% ", PWM_Value);
                 GUI_ShowString(0, OLED_HEIGHT / 8 - 3, line, 8, 1);
             }
        }

    }
}
