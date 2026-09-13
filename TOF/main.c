/**
 * @file main.c
 * @brief Einstieg in den TOF-Distanztest mit DAVE 4 und OLED.
 *
 * @details Nach dem Einschalten richtet main() die Hardware ein und startet
 * den VL53L0X. In einer Endlosschleife werden Messung, Anzeige, LED und PWM
 * zu unterschiedlichen Zeitpunkten bearbeitet. Ein Timer zaehlt dazu die
 * vergangenen Millisekunden. Die Vorlage stammt aus TOFTest/Src/main.c
 * des STM32-Cube-Projekts; hier werden DAVE-APPs verwendet.
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

/** @brief Anzahl gueltiger Messwerte im gleitenden Mittelwert. */
#define TOF_MEAN_WINDOW_SIZE 10U

/* Die Summe passt bei maximal 65535 Werten vom Typ uint16_t in uint32_t. */
#if (TOF_MEAN_WINDOW_SIZE < 1U) || (TOF_MEAN_WINDOW_SIZE > 65535U)
#error "TOF_MEAN_WINDOW_SIZE muss zwischen 1 und 65535 liegen"
#endif
//#define TOF_PROFILE TOF_HIGH_ACCURACY_MODE_A

#define DisplayRate (TOF_Sensor_1.Ranging_Profile_time + 5U)*4

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
    uint32_t last_pwm = 0U;
    /* Separater Anzeigewert: distanceFromTOF bleibt der ungefilterte Sensorwert. */
    uint16_t distance_mean_mm = TOF_VL53L0X_OUT_OF_RANGE;
    /* Platz fuer bis zu 21 Zeichen und das abschliessende Nullzeichen. */
    char line[22];

    XMC_GPIO_Init(LED1, &led_config);
    XMC_GPIO_Init(LED2, &led_config);
    if (status != DAVE_STATUS_SUCCESS) {
        XMC_GPIO_SetOutputHigh(LED2);
        /* Ohne funktionierende Hardware oder Zeitbasis hier bewusst anhalten. */
        while (1U) { }
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
    GUI_ShowString(0, 0, "VL53L0X TOF", 16, 1);
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
                show_status("TOF Lesefehler   ");
            } else {
                /* Jede neue Messung genau einmal in die Mittelwertbildung geben. */
                distance_mean_mm = TOF_MovingAverage(TOF_Sensor_1.distanceFromTOF);
            }
        }
        /* Langsamere Anzeige spart Buszeit; die Messung darf haeufiger erfolgen. */
        if ((uint32_t)(now - last_display) >= DisplayRate) {
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
            snprintf(line, sizeof(line), "Zeit: %6lu s", (unsigned long)(now / 1000U));
            GUI_ShowString(0, OLED_HEIGHT / 8 - 1, line, 8, 1);
        }
        if ((uint32_t)(now - last_led) >= 1000U) {
            last_led = now;
            XMC_GPIO_ToggleOutput(LED1);
        }
        /* PWM veraendert das Verhaeltnis von Ein- zu Ausschaltzeit am Ausgang.
         * % 1000 liefert den Rest der Division, also Werte von 0 bis 999.
         * DAVE verwendet fuer den Tastgrad 0 bis 10000 (= 0 bis 100 Prozent). */
        if ((uint32_t)(now - last_pwm) >= 10U) {
            last_pwm = now;
            PWM_SetDutyCycle(&PWM_0, 10U * (now % 1000U));
        }
    }
}
