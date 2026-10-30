# CAN-Spezifikation TOF_PID_PWM

Stand 29.10.2026. Classic CAN, 500 kbit/s, voreingestellt 11-Bit-IDs.
Alle genannten IDs sind hexadezimal. Der XMC1404 verwendet maximal acht
Datenbytes je Frame; es gibt keine 64-Byte-Botschaft.

## Board sendet Telemetrie

Nach einem gueltigen UART-Parametersatz mit Gruppe 1..32 sendet das Board auf
**0x100 + Gruppe**: Gruppe 1 auf 0x101, Gruppe 32 auf 0x120.
Vor der ersten gueltigen UART-Gruppennummer ist Gruppe 0 aktiv und es werden
keine CAN-Sendeauftraege gestartet. Empfang und Hardware-ACK bleiben aktiv.
Die bisherige ID 0x123 wird nicht mehr fuer die Telemetrie verwendet.

Intervall: mindestens 50 ms zwischen Sendeversuchen, kein Nachholen. Nutzdaten:

| Offset | Inhalt |
| --- | --- |
| 0 | Botschaftszaehler 0..15 in Bits 0..3, Bits 4..7 null |
| 1 | PWM in Prozent, int8_t, aktuell 0..100 |
| 2..3 | Gemittelte Hoehe in mm, uint16_t Little Endian; 0xFFFF ungueltig |
| 4..5 | Wirksamer Sollwert, int16 Little Endian; -1 solange CAN-Vorgabe fehlt |
| 6 | Formatkennung 2: Telemetrie mit Sollwert und Quellenauswahl |
| 7 | Bit 0: UART-Vorgabe 0 / CAN-Sollwertbetrieb; Bits 1..7 null |

DLC ist 8. Nur angenommene Sendeauftraege erhoehen den Zaehler.
BUSY laesst den laufenden Auftrag unveraendert. Auch beim Gruppenwechsel
behaelt ein bereits laufender Auftrag seine alte ID; erst neue Auftraege
verwenden die neue Gruppe. Hardware-Wiederholungen bleiben moeglich.

## Zentraler Sender liefert Sollwerte auf ID 0x100

Pro Gruppe wird ein eigener Classic-CAN-Datenframe mit **DLC 3** gesendet:

| Offset | Inhalt |
| --- | --- |
| 0 | Gruppe, uint8_t, 1..32 |
| 1 | Hoehensollwert Low-Byte |
| 2 | Hoehensollwert High-Byte |

Die Sollhoehe ist int16_t Little Endian. Die Anwendung akzeptiert 0..300 mm;
negative Werte, Werte ueber 300, andere Gruppen, andere ID-/Identifierformate
und andere Datenlaengen werden ignoriert. Keine zusaetzliche Anwendungs-CRC.
Die CAN-Hardware prueft den CAN-Frame.

Beispiele auf ID 0x100:

- Gruppe 1, 150 mm: `01 96 00`.
- Gruppe 2, 200 mm: `02 C8 00`.
- Gruppe 32, 300 mm: `20 2C 01`.
- Gruppe 1, 0 mm: `01 00 00` (wirksamer CAN-Sollwert 0 mm).

## Auswahl der Sollwertquelle

Der UART-Parameterbefehl behaelt sein 13-Byte-Format. Sein Sollwertfeld bedeutet:

- **1..300 mm:** direkte UART-Vorgabe fuer den PID. CAN-Sollwerte werden ignoriert.
- **0:** CAN-Betrieb. Nur Frames fuer die per UART eingestellte eigene Gruppe
  werden als Sollwert uebernommen. Die UART-Null bleibt als Quellenauswahl
  gespeichert, auch wenn CAN danach einen positiven Sollwert liefert.

UART_Setpoint_mm enthaelt die UART-Vorgabe/Quellenauswahl; TOF_Setpoint_mm den
wirksamen Regelsollwert. UART-Telemetrie ch2 und OLED zeigen den wirksamen Wert.
Beim erstmaligen Eintritt in CAN-Betrieb oder Gruppenwechsel wird der alte
CAN-Sollwert verworfen; bis zum naechsten passenden gueltigen Frame bleibt
PWM 0. Ein identischer UART-Parametersatz verwirft keinen vorhandenen CAN-Wert.
Aenderungen an den Verstaerkungen alleine behalten die gewaehlte Quelle und
setzen wie bisher das PID-Gedaechtnis zurueck.

Ein uebernommener CAN-Sollwert bleibt bis zum naechsten gueltigen Wert oder
Quellen-/Gruppenwechsel aktiv. Es gibt keinen CAN-Sollwert-Timeout.
Der Sensorfehler-/Messwertalter-Schutz der Regelung bleibt aktiv.
Ein CAN-Sollwertwechsel behaelt den I-Anteil und setzt nur das D-Gedaechtnis zurueck. Nach Reset gilt
wie bisher UART-Vorgabe 300 mm und Gruppe 0.

## Bibliothek, Diagnose und Pruefung

CANLIB_RX_ID ist auf 0x100 vorbelegt. CANLIB_SendWithId() erlaubt eine
Sende-ID je Auftrag und prueft BUSY vor Aenderung von ID/Daten.
CANLIB_Send() bleibt als Wrapper mit CANLIB_TX_ID (Standard 0x101) verfuegbar;
die Anwendung verwendet fuer ihre Telemetrie ausschliesslich SendWithId().
CAN_RX auf einem anderen Board empfaengt jetzt 0x101..0x120 parallel.

can_setpoint_count zaehlt uebernommene CAN-Sollwerte. CAN-RX verwendet ein
Hardwareobjekt und speichert den neuesten Frame; mehrere schnelle Gruppen-
Frames koennen sich waehrend OLED-/I2C-Zugriffen ueberschreiben. Der zentrale
Sender sollte die Gruppenwerte zyklisch wiederholen. Keine verlustfreie Queue.

Tests/validate_can_group.py fuehrt die echten Anwendungshelfer als ARM-Code
mit simuliertem Sendeaufruf aus. Hardware, ACK und Buslast sind damit nicht
geprueft. Ein Boardtest steht aus.

CAN_RX verwendet jetzt alle 32 Gruppen-IDs und zeigt Soll, Ist und 60-s-ISE.
Beide Firmwares muessen fuer die Sollwertanzeige mit diesem Stand gebaut werden.

CAN_RX erzeugt fuer Gruppen im CAN-Sollwertbetrieb alle 5 s einen gemeinsamen
Pseudozufallssollwert 100..200 mm und sendet je Gruppe einen Frame auf 0x100.
Byte 7 Bit 0 bleibt waehrenddessen gesetzt, bis UART wieder 1..300 mm vorgibt.

Bei angenommenem Telemetrie-Sendeauftrag zeigt die Textansicht
`CAN TX sent RX active`, sobald im CAN-Sollwertbetrieb ein gueltiger Wert
fuer die eigene Gruppe empfangen wurde. Ohne solchen Wert steht dort nur
`CAN TX sent`. Gruppen-/Quellenwechsel loeschen den Empfangsstatus.
Die vorhandene Oszilloskopansicht und CAN-Fehleranzeigen behalten Vorrang.

Hinweis zur Reglerkorrektur vom 30.10.2026: Reine Sollwertaenderungen
ueber UART oder CAN loeschen den I-Anteil nicht mehr. Ein vollstaendiger Reset
erfolgt weiterhin bei Gruppen-/Quellen-/Verstaerkungsaenderungen sowie
Messfehlern und den vorhandenen Zeitueberschreitungen.
