/** @file main.c
 * @brief TestP05: P0.5 und LED2 alle fuenf Sekunden gemeinsam umschalten.
 * @details KIT_XMC14_2Go mit XMC1404. Beide Pins starten LOW. Nach 5 s
 * werden sie HIGH, nach weiteren 5 s wieder LOW. Ein kompletter Zyklus
 * dauert 10 s. Keine PWM-Erzeugung, keine Sensor-, CAN- oder UART-Ansteuerung.
 * P0.5 ist ein Push-Pull-GPIO. HIGH bedeutet T13-Ansteuerung EIN, LOW AUS.
 */
#include "DAVE.h"
#include <stdbool.h>
#include <stdint.h>
/** @brief P0.5 ist der zu pruefende Ausgang. */
#define TEST_PIN 5U
/** @brief LED2 des Kits liegt an P0.6 und leuchtet bei HIGH. */
#define LED2_PIN 6U
/** @brief Portmaske beider gleichzeitig geschalteter Ausgaenge. */
#define OUTPUT_MASK ((1UL<<TEST_PIN)|(1UL<<LED2_PIN))
/** @brief SYSTIMER erwartet Mikrosekunden: 5.000.000 us = 5 s. */
#define SWITCH_INTERVAL_US 5000000UL
/** @brief Aktueller Ausgangszustand; im Debugger sichtbar. */
volatile bool TestP05_On=false;
/** @brief Anzahl der ausgefuehrten Umschaltungen; im Debugger sichtbar. */
volatile uint32_t TestP05_SwitchCount=0U;
/** @brief Initialisierungsergebnis zur Diagnose, ohne zusaetzlichen Ausgang. */
volatile DAVE_STATUS_t TestP05_InitStatus=DAVE_STATUS_FAILURE;

/** @brief Initialisiert nur Takt und SYSTIMER fuer den GPIO-Test.
 * @return SUCCESS bei erfolgreicher Initialisierung, sonst Fehlerstatus.
 * @details Diese starke Definition ersetzt die __WEAK DAVE_Init-Version
 * in Dave/Generated/DAVE.c. Die geerbte PWM-APP wird NICHT initialisiert,
 * damit P0.5 keinen ungewollten 50-Prozent-Startimpuls erzeugt.
 * Auch I2C bleibt unbenutzt. Der eigene Init bleibt bei DAVE-Codegenerierung
 * in main.c erhalten; die generierten Dateien werden nicht manuell geaendert.
 */
DAVE_STATUS_t DAVE_Init(void)
{
    DAVE_STATUS_t status=(DAVE_STATUS_t)CLOCK_XMC1_Init(&CLOCK_XMC1_0);
    if (status==DAVE_STATUS_SUCCESS) {
        status=(DAVE_STATUS_t)SYSTIMER_Init(&SYSTIMER_0);
    }
    return status;
}
/** @brief Geerbter I2C-Callback; I2C wird in diesem Test nicht gestartet.
 * @details Nur vorhanden, damit die unveraenderte DAVE-Vorlage vollstaendig
 * bleibt. Diese Funktion schreibt bewusst nicht auf LED2.
 */
void NACK_Handler(void) { }

/** @brief Schaltet Ausgang und LED mit EINEM Registerzugriff.
 * @param[in] on true: beide HIGH; false: beide LOW.
 * @details OMR-Bits 0..15 setzen Pins, Bits 16..31 loeschen sie. Der gemeinsame
 * Schreibzugriff vermeidet den Zeitversatz zweier GPIO-Funktionsaufrufe.
 * Andere Portpins bleiben unveraendert. Keine alternierende PWM-Funktion.
 */
static void TestP05_SetOutputs(bool on)
{
    XMC_GPIO_PORT0->OMR=on ? OUTPUT_MASK : OUTPUT_MASK<<16U;
    TestP05_On=on;
}
/** @brief Periodischer 5-s-Callback; nur kurze GPIO-Zugriffe.
 * @param[in] args Unbenutztes Timerargument.
 * @details SYSTIMER ruft im Interrupt auf. Keine Wartefunktion, keine Buszugriffe.
 */
static void TestP05_Switch(void *args)
{
    (void)args;
    TestP05_SetOutputs(!TestP05_On);
    ++TestP05_SwitchCount;
}
/** @brief Startet den Test mit LOW und schaltet danach dauerhaft alle 5 s.
 * @return Kein regulaeres Programmende.
 */
int main(void)
{
    const XMC_GPIO_CONFIG_t output={
        .mode=XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
        .output_level=XMC_GPIO_OUTPUT_LEVEL_LOW
    };
    uint32_t timer;
    /* So frueh wie moeglich einen definierten LOW-Pegel herstellen. */
    XMC_GPIO_Init(XMC_GPIO_PORT0,TEST_PIN,&output);
    XMC_GPIO_Init(XMC_GPIO_PORT0,LED2_PIN,&output);
    TestP05_SetOutputs(false);
    TestP05_InitStatus=DAVE_Init();
    if (TestP05_InitStatus!=DAVE_STATUS_SUCCESS) { while (1U) { } }
    timer=SYSTIMER_CreateTimer(SWITCH_INTERVAL_US,SYSTIMER_MODE_PERIODIC,TestP05_Switch,NULL);
    if (timer==0U) { TestP05_InitStatus=DAVE_STATUS_FAILURE; while (1U) { } }
    if (SYSTIMER_StartTimer(timer)!=SYSTIMER_STATUS_SUCCESS) {
        TestP05_InitStatus=DAVE_STATUS_FAILURE;
        while (1U) { }
    }
    /* Der Interrupt schaltet die Pins; die CPU darf dazwischen schlafen. */
    while (1U) { __WFI(); }
}
