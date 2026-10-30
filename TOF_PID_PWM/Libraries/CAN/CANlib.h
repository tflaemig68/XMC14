/**
 * @file CANlib.h
 * @brief Parametrierbarer Classic-CAN-Sender und -Empfaenger fuer KIT_XMC14_2Go.
 * @details Die ID identifiziert eine Botschaft, nicht die Adresse eines Geraetes.
 * CANlib benutzt CAN-Knoten 0 und Message Objects 0 (RX) und 1 (TX) exklusiv. I2C und OLED werden
 * weiterhin durch DAVE initialisiert. Es ist keine zusaetzliche CAN-DAVE-APP noetig.
 */
#ifndef CANLIB_H
#define CANLIB_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Laenge des Identifiers: 11 (Standard) oder 29 (Extended) Bit. */
#ifndef CANLIB_ID_BITS
#define CANLIB_ID_BITS 11U
#endif

/** @brief Exakt zu empfangende ID, ohne Verschiebung oder zusaetzliche Flagbits. */
#ifndef CANLIB_RX_ID
#define CANLIB_RX_ID 0x100UL
#endif

/** @brief Standard-Sende-ID; die Anwendung sendet gruppenabhaengig mit SendWithId. */
#ifndef CANLIB_TX_ID
#define CANLIB_TX_ID 0x101UL
#endif

/** @brief Busgeschwindigkeit in Bit/s; Sender und Empfaenger muessen uebereinstimmen. */
#ifndef CANLIB_BITRATE
#define CANLIB_BITRATE 500000UL
#endif

#if CANLIB_ID_BITS == 11U
#define CANLIB_ID_MASK 0x7FFUL
#elif CANLIB_ID_BITS == 29U
#define CANLIB_ID_MASK 0x1FFFFFFFUL
#else
#error "CANLIB_ID_BITS muss 11 oder 29 sein"
#endif
#if (CANLIB_RX_ID < 0) || (CANLIB_RX_ID > CANLIB_ID_MASK)
#error "CANLIB_RX_ID passt nicht zur eingestellten ID-Laenge"
#endif
#if (CANLIB_TX_ID < 0) || (CANLIB_TX_ID > CANLIB_ID_MASK)
#error "CANLIB_TX_ID passt nicht zur eingestellten ID-Laenge"
#endif
#if (CANLIB_BITRATE == 0U) || (CANLIB_BITRATE > 1000000UL)
#error "CANLIB_BITRATE muss zwischen 1 und 1000000 Bit/s liegen"
#endif

/** @brief Zeit ohne neue passende Botschaft bis zur Anzeige ALT, in Millisekunden. */
#define CANLIB_RX_TIMEOUT_MS 1000UL

/**
 * @brief Kopie einer vollstaendig empfangenen Datenbotschaft.
 * @details Der XMC1404 empfaengt hier Classic CAN, kein CAN FD. DLC 9 bis 15
 * bedeutet bei Classic CAN ebenfalls acht Nutzdatenbytes. Bytes oberhalb von
 * length werden auf 0 gesetzt, damit keine alten Nutzdaten angezeigt werden.
 */
typedef struct {
    uint32_t id;      /**< Unverschobener Identifier. */
    uint8_t id_bits;  /**< Identifierformat: 11 oder 29 Bit. */
    uint8_t dlc;      /**< Empfangener Data Length Code (0 bis 15). */
    uint8_t length;   /**< Tatsaechliche Nutzdatenlaenge (0 bis 8 Bytes). */
    uint8_t data[8];  /**< Daten in Busreihenfolge; data[0] ist das erste Byte. */
} CANLIB_Message_t;

/** @brief Betriebszustand fuer die Statusanzeige. */
typedef enum {
    CANLIB_STATE_NOT_READY, /**< Noch nicht oder nicht erfolgreich initialisiert. */
    CANLIB_STATE_ACTIVE,    /**< CAN aktiv, keine Fehlerwarnschwelle erreicht. */
    CANLIB_STATE_WARNING,   /**< CAN-Fehlerwarnschwelle erreicht. */
    CANLIB_STATE_BUS_OFF    /**< Knoten vom Bus abgeschaltet; Board neu starten. */
} CANLIB_State_t;

/**
 * @brief Initialisiert CAN0, Pins, Bitzeit, Empfangsfilter und Sendeobjekt.
 * @pre DAVE_Init() muss erfolgreich ausgefuehrt worden sein.
 * @return true bei Erfolg, false falls die Bitzeit nicht einstellbar ist.
 * @note Einmal beim Start aufrufen. RX=P1.1, TX=P1.0/ALT9 passen zum
 * eingebauten Transceiver des KIT_XMC14_2Go. Der Knoten bestaetigt empfangene
 * Frames mit ACK und sendet Daten erst nach CANLIB_Send(). Kein Silent Mode.
 */
bool CANLIB_Init(void);

/**
 * @brief Fragt ohne Warten auf eine neue Botschaft das Empfangsobjekt ab.
 * @param[out] message Gueltiger Ergebnispuffer; wird nur bei true beschrieben.
 * @return true fuer eine neue passende Datenbotschaft, andernfalls false.
 * @details Hoechstens drei Leseversuche verhindern eine Endlosschleife bei
 * gleichzeitigem Hardwareempfang. Im Hauptprogramm moeglichst oft aufrufen.
 * Ein einzelnes Hardwareobjekt speichert den neuesten Frame; waehrend langsamer
 * Displayzugriffe koennen aeltere Frames ueberschrieben werden. Dies ist eine
 * Anzeige der letzten Botschaft, kein verlustfreier CAN-Logger.
 */
bool CANLIB_Receive(CANLIB_Message_t *message);

/** @brief Ergebnis der Uebergabe an das einzelne Sendeobjekt. */
typedef enum {
    CANLIB_TX_ACCEPTED, /**< Auftrag angenommen; noch keine Empfangsbestaetigung! */
    CANLIB_TX_BUSY,     /**< Voriger Auftrag laeuft; Daten unveraendert lassen. */
    CANLIB_TX_ERROR     /**< Ungueltige Parameter, nicht bereit oder Bus-off. */
} CANLIB_TxResult_t;

/**
 * @brief Startet eine Classic-CAN-Datenbotschaft mit CANLIB_TX_ID.
 * @param[in] data Zeiger auf mindestens length Bytes; darf nicht NULL sein.
 * @param[in] length Anzahl Nutzdatenbytes von 0 bis 8 (gleich DLC).
 * @return ACCEPTED bei angenommenem Auftrag, BUSY bei belegtem Objekt,
 *         ERROR bei ungueltigen Parametern oder nicht betriebsbereitem CAN.
 * @details Kopiert die Daten sofort; der Aufrufer darf seinen Puffer danach
 * wiederverwenden. Wartet nicht auf Bus oder ACK und hat keine Warteschlange.
 * Bei BUSY/ERROR bleibt der bisherige Auftrag unveraendert. Die Hardware
 * wiederholt fehlgeschlagene Uebertragungen automatisch. Ohne ACK kann das
 * Objekt dauerhaft belegt bleiben. Bus-off erfordert einen Board-Neustart.
 * @note Nur aus der Hauptschleife aufrufen, nicht gleichzeitig aus Interrupts.
 * ACCEPTED bestaetigt nur die Uebergabe, nicht die erfolgreiche Busuebertragung.
 */
CANLIB_TxResult_t CANLIB_Send(const uint8_t *data, uint8_t length);

/** @brief Startet einen Sendeauftrag mit einer zur Laufzeit gewaehlten ID.
 * @param[in] id Unverschobener Identifier innerhalb CANLIB_ID_MASK.
 * @param[in] data Datenpuffer mit mindestens length Bytes.
 * @param[in] length Nutzdatenlaenge 0..8.
 * @return ACCEPTED, BUSY oder ERROR wie bei CANLIB_Send().
 * @details Prueft BUSY vor Aenderung von ID oder Daten. Ein laufender Auftrag
 * behaelt auch bei Gruppenwechsel seine bisherige ID bis zum Abschluss.
 */
CANLIB_TxResult_t CANLIB_SendWithId(uint32_t id, const uint8_t *data, uint8_t length);

/** @brief Liefert den CAN-Knotenzustand. @return Aktueller Betriebszustand. */
CANLIB_State_t CANLIB_GetState(void);

/**
 * @brief Liefert die Anzahl erkannter Hardware-Ueberlaufmeldungen.
 * @return Seit Initialisierung beobachtete MSGLST-Ereignisse.
 * @note Ein Ereignis kann mehrere ueberschriebene Botschaften bedeuten;
 * der Wert ist keine genaue Anzahl aller verlorenen Frames.
 */
uint32_t CANLIB_GetOverrunCount(void);

#endif /* CANLIB_H */
