# UART-Spezifikation CAN_RX

Stand: 29.10.2026. Revision 4.0, implementierte gruppenweise Binaerausgabe.
Diese Revision ersetzt die bisherige Beschreibung ohne UART-Schnittstelle.

## Verbindung

KIT_XMC14_2Go, eingebauter J-Link-USB-COM-Port. Die Portnummer wird vom PC
vergeben. USIC0_CH0 sendet ueber P2.1 ALT6 zur USB-Bruecke.
128000 Baud (UARTLIB_BAUDRATE), 8 Datenbits, keine Paritaet, ein Stopbit (8N1).
Keine Hardware- oder Software-Flusssteuerung. Keine ASCII-Zahlen, kein
Zeilenende. UARTLIB_ActualBaudrate zeigt die Generatorbaudrate im Debugger.
Die Rate ist wie bei TOF_PID_PWM voreingestellt; die fehlerfreie physische
Uebertragung durch die konkrete Bruecke ist noch am Board zu pruefen.
Bei Aenderung muessen PC und Firmware dieselbe Baudrate verwenden.

Diese Erweiterung implementiert nur Ausgabe. CAN_RX nimmt keine UART-Befehle
an. P2.2 wird nicht als UART-Eingang konfiguriert. Kein eigener UART-Interrupt.
TX-FIFO: 16 Worte, RAM-Adressen 48..63; I2C behaelt den Bereich 0..31.

## Paket CAN_RX -> PC

Genau acht Bytes je Gruppe, Offsets ab 0:

```text
5A Gruppe Ist_L Ist_H Soll_L Soll_H ISE_L ISE_H
```

| Offset | Feld | Typ und Bedeutung |
| --- | --- | --- |
| 0 | Synchronisation | Konstant 0x5A |
| 1 | Gruppennummer | uint8_t, 1..32 |
| 2..3 | Isthoehe | int16_t Little Endian, mm; -1 ungueltig |
| 4..5 | Wirksame Sollhoehe | int16_t Little Endian, mm; -1 ungueltig/unbekannt |
| 6..7 | ISE60 | uint16_t Little Endian, ganze mm^2*s, maximal 65535 |

Isthoehe und Sollhoehe sind int16 im Zweierkomplement, Low-Byte zuerst.
CAN_RX verwendet **5A + Gruppe** anstelle des TOF_PID_PWM-Headers **55 AA**.
Das bisherige PWM-Feld wurde durch eine zwei Byte breite ISE ersetzt.
Es gibt keine CRC, keinen Zaehler, kein Laengenfeld und kein Strukturpadding.

Isthoehe und Sollhoehe stammen aus dem zuletzt abgeholten CAN-Gruppenframe.
Istwerte bis 32767 werden als int16 uebertragen, groessere uint16-Werte
(einschliesslich 0xFFFF) als -1. Dies ist unabhaengig vom engeren
0..300-mm-ISE-Auswertebereich. Ohne gueltigen Sollwert in Telemetrieversion
1/2 wird -1 gesendet.

ISE entspricht dem Integral der letzten 60 abgeschlossenen Sekunden,
abgerundet auf ganze mm^2*s (interne Summe / 1000), anschliessend auf
**65535 begrenzt**. Ein groesserer Wert wird als 0xFFFF uebertragen, niemals
modulo 65536 umgebrochen. 0xFFFF bedeutet hier 65535 oder groesser;
ein eigener Ueberlaufstatus wird nicht uebertragen.
Die interne ISE-Berechnung und die Displayanzeige behalten ihren bisherigen
Wertebereich. Bei fehlenden gueltigen Soll-/Istwerten wird die ISE wie bisher
nicht weiter integriert; noch vorhandene Historie bleibt bis zum Herausfallen
aus dem Zeitfenster erhalten. Es gibt kein eigenes
ISE-Gueltigkeitsbit, siehe auch die ISE-Definition in README_CAN_RX.md.

Beispiele:

| Gruppe / Ist / Soll / ISE | Hexadezimale Bytefolge |
| --- | --- |
| 1 / 150 mm / 200 mm / 123 mm^2*s | `5A 01 96 00 C8 00 7B 00` |
| 32 / ungueltig / ungueltig / 0 mm^2*s | `5A 20 FF FF FF FF 00 00` |

## Zeitverhalten und Gruppenwahl

Ein Gruppenumlauf startet fruehestens alle 50 ms (UARTGROUP_PERIOD_MS).
Aktuell vorhandene Gruppen werden aufsteigend von 1 bis 32 ausgegeben,
unabhaengig von der gerade sichtbaren Displayseite. Vor jedem Paket wird die
Aktualitaet geprueft: weniger als 1000 ms seit letztem Gruppenframe.
Ohne vorhandene Gruppen werden keine Pakete gesendet.

Pro Hauptschleife wird hoechstens ein Paket uebergeben. Der 16-Wort-TX-FIFO
muss mindestens acht freie Plaetze haben. Bei BUSY wird kein Teilpaket gesendet;
die gleiche Gruppenposition wird spaeter mit dann aktuellen Daten erneut
versucht. Ein laufender Umlauf wird abgeschlossen, bevor der naechste startet.
Kein Nachholen ausgefallener Zeitintervalle und keine blockierende Warteschleife.

Die Werte verschiedener Gruppen sind kein zeitgleich aufgenommener Snapshot.
CAN-/OLED-Zugriffe und USB-Pufferung koennen das Intervall verlaengern oder
Pakete in einem PC-Lesezugriff zusammenfassen. Paketgrenzen entsprechen nicht
USB-Lesegrenzen. Bei 32 Gruppen und nominal 20 Umlaeufen/s entstehen
5120 Bytes/s, mit 8N1 51200 Bit/s. Die reine UART-Zeit fuer einen Umlauf
betraegt bei 128000 Baud 20 ms (0,625 ms je Paket), ohne Software-/USB-Verzoegerung.

## Dekodierung auf dem PC

1. Nach 0x5A suchen, Folgebyte als Gruppennummer 1..32 pruefen.
2. Insgesamt acht Bytes sammeln, auch ueber mehrere Lesezugriffe hinweg.
3. Zwei int16-Werte aus Bytes 2..5 und einen uint16-Wert aus Bytes 6..7
   mit Little Endian dekodieren.
4. Innerhalb eines ausgerichteten Pakets ist ein weiteres 0x5A normales Datenbyte.

Einzelbyte-Synchronisation und Gruppenpruefung garantieren nach Byteverlust
keine eindeutige Erkennung. Ohne CRC und Escaping koennen falsche Ausrichtung
und unerkannte Datenfehler auftreten. Es gibt keine Empfangsquittierung.
Eine Sendepause ist nicht Teil des Protokolls.

## Diagnose und Pruefung

UARTGROUP_Sent zaehlt vollstaendige FIFO-Uebergaben, keine bestaetigten
PC-Empfaenge. UARTGROUP_Busy zaehlt zurueckgestellte Versuche, keine verlorenen
Pakete. UART-Initialisierungsfehler werden ueber die Fehler-LED angezeigt;
CAN und Display laufen weiter.

Implementierung: Libraries/UART/UARTlib.c/.h und UARTgroups.c/.h;
ISE-Berechnung in Libraries/CAN/GroupMonitor.c/.h.
Tests/validate_uart.py prueft echtes ARM-Packen und den Umlauf mit simuliertem
FIFO: Bytefolge, Vorzeichen, Gruppensortierung, Zeitabstand, BUSY und Timeout.
Tests/validate_config.py kompiliert die betroffenen Dateien mit Warnungen als Fehler.
Hardwaretest und Flashen sind nicht Bestandteil dieser Softwarepruefung.
