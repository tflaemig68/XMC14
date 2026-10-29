# CAN_RX - CAN-Empfang mit KIT_XMC14_2Go und DAVE 4

Eigenstaendiges Projekt in D:/GITHUB/XMC14/CAN_RX, abgeleitet von TOF.
Zielcontroller: XMC1404-Q040x0200. Das Quellprojekt TOF bleibt unveraendert.
TOF-Treiber, Mittelwertbildung und abstandsabhaengige PWM-Ansteuerung werden
in diesem Projekt nicht verwendet. Das vorhandene OLED zeigt eine CAN-Botschaft.

## Einstellungen in Libraries/CAN/CANlib.h

| Parameter | Voreinstellung | Bedeutung |
| --- | --- | --- |
| CANLIB_ID_BITS | 11U | 11-Bit-Standard oder 29-Bit-Extended |
| CANLIB_RX_ID | 0x123UL | Exakt zu empfangende Botschafts-ID |
| CANLIB_BITRATE | 500000UL | Bestaetigte Busgeschwindigkeit 500 kbit/s |
| CANLIB_RX_TIMEOUT_MS | 1000UL | Danach ohne neuen passenden Frame Anzeige ALT |

Nach Aenderungen neu bauen und flashen. Fuer Extended CAN beispielsweise
CANLIB_ID_BITS auf 29U und CANLIB_RX_ID auf 0x18FF0123UL setzen. ID-Bereich:
11 Bit = 0 bis 0x7FF, 29 Bit = 0 bis 0x1FFFFFFF. Ungueltige ID-Laengen oder zu
grosse/negative IDs erzeugen beim Uebersetzen einen Fehler. Das Format wird
mitgefiltert: Extended-ID 0x123 passt nicht zum Standardfilter 0x123.
Die ID-Laenge ist unabhaengig von der Nutzdatenlaenge (DLC).

## Board und Verkabelung

- CAN0 RX: P1.1, Eingang RXDCH.
- CAN0 TX: P1.0, Alternativfunktion ALT9.
- Diese Pins sind auf dem KIT_XMC14_2Go bereits mit dem eingebauten
  TLE9351-Transceiver verbunden. Die externen Anschluesse sind CAN-H und CAN-L.
- Gemeinsamen CAN-GND anschliessen; am Kit steht GND zur Verfuegung.
- OLED wie in TOF: SDA P2.10, SCL P2.11, 400 kHz, 7-Bit-Adresse 0x3C.
- P0.7 blinkt als Lebenszeichen, P0.6 zeigt Initialisierungs-/Buswarnungen oder
  einen vom DAVE-Handler gemeldeten OLED-I2C-Fehler an.

Laut Kit-Handbuch ist ein Abschlusswiderstand R3 auf dem Board vorhanden.
An beiden Enden des CAN-Busses je 120 Ohm zwischen H und L vorsehen; vorhandene
Widerstaende mitzaehlen. An einem mittleren Busknoten soll kein weiterer
Abschluss verbleiben. CAN_H/CAN_L sind Buspegel, nicht die MCU-Pins RX/TX.

| Signal | VN7610 direkt | CAN-Abzweig FR/CANcable 2Y | Kit |
| --- | --- | --- | --- |
| CAN_L | Pin 1 | Pin 2 | CAN-L |
| CAN_H | Pin 9 | Pin 7 | CAN-H |
| CAN_GND | Pin 6 | Pin 3 | GND |

Quellen fuer die Hardwarebelegung:
- [Infineon KIT_XMC14_2Go User Guide, Abschnitt 2.6](https://www.infineon.com/assets/row/public/documents/30/44/infineon-kit-xmc14-2go-usermanual-en.pdf)
- [Infineon XMC1400 Pin-Funktionen](https://documentation.infineon.com/xmc1000/docs/zir1764741941297)
- [Vector VN7610 Manual, Seite 12](https://www.vector.com/dam/jcr%3A06a72e28-bcf0-459b-a0ea-81d05d409d7a/VN7610_Manual_EN.2026-06-29-19-40-43.pdf)

## Softwareaufbau

main.c initialisiert die vorhandenen DAVE-APPs, Timer und OLED. Anschliessend
richtet CANLIB_Init() den CAN-Knoten ueber die bereits enthaltene XMCLib ein.
Es wird keine CAN-APP im DAVE-Modell hinzugefuegt. Die CAN-Pins P1.0/P1.1
muessen bei spaeteren DAVE-Aenderungen fuer CANlib frei bleiben.
CANlib belegt CAN0 und Message Object 0 exklusiv. Sie verwendet MCLK als Quelle,
einen angeforderten CAN-Modultakt von 8 MHz und berechnet die Bitzeit aus dem
von XMC_CAN_InitEx zurueckgelieferten tatsaechlichen Takt. Die sample_point-
Vorgabe ist 80 Prozent; die realisierbare Position haengt vom Taktteiler ab.

Die geerbte PWM-APP bleibt im DAVE-Modell erhalten. Unmittelbar nach erfolgreichem
DAVE_Init() setzt main() den Tastgrad auf 0, stoppt die PWM und legt P0.5 auf Low.
Die APP startet gemaess Vorlage kurzzeitig waehrend DAVE_Init() mit 50 Prozent.
CAN_RX ist kein Geblaeseregler und verarbeitet keine Entfernung aus CAN-Nutzdaten.

CANLIB_Receive() prueft im Hauptprogramm auf neue Daten und erstellt eine
konsistente Kopie von ID, DLC und Daten. Aendert die Hardware waehrend der Kopie
den Empfangspuffer, wird die Kopie verworfen und bis zu dreimal neu versucht.
Die Methode wartet nicht auf eine neue Botschaft. Der Knoten arbeitet aktiv und
sendet CAN-Bestaetigungen (ACK), aber keine eigenen Datenframes. Remote-Requests
werden nicht als Nutzdaten angezeigt; nur Datenframes aktualisieren das RX-Objekt.

Classic CAN mit 0 bis 8 Datenbytes wird unterstuetzt, kein CAN FD. Ein DLC von
9 bis 15 bedeutet bei Classic CAN acht Datenbytes und wird so dargestellt.
Die Rohdaten werden in Busreihenfolge angezeigt. Zusaetzlich dekodiert die
Anwendung das unten beschriebene TOF_CAN-Format fuer die Oszilloskopansicht.

## OLED-Anzeige

| Zeile | Inhalt |
| --- | --- |
| 0 | CAN RX 500 kbit/s |
| 1 | Empfangs-ID (vor dem ersten Empfang die konfigurierte ID) |
| 2 | ID-Laenge, originaler DLC und tatsaechliche Byteanzahl N |
| 3 | D0: Datenbytes 0 bis 3 |
| 4 | D4: Datenbytes 4 bis 7 |
| 5 | RX: abgeholte Frames, L: beobachtete Ueberlaufmeldungen |
| 6 | WARTE AUF ID, AKTIV, ALT, CAN WARNUNG oder BUS OFF |
| 7 | PWM in Prozent und Botschaftszaehler; ohne vollstaendiges Paket Laufzeit |

Nicht vorhandene Bytes erscheinen als --. DLC 0 loescht daher auch die Anzeige
vorheriger Nutzdaten. Die Anzeige wird etwa alle 100 ms aktualisiert. Ohne neue
Botschaft bleiben die letzten Daten sichtbar und werden nach einer Sekunde
als ALT markiert. Bei Bus-off ist ein Board-Neustart vorgesehen; die Bibliothek
fuehrt keine automatische Wiederanmeldung durch.

CAN_LastMessage und CAN_ReceivedCount koennen im Debugger betrachtet werden.
CANLIB_GetOverrunCount() liefert beobachtete MSGLST-Meldungen. Ein einzelnes
Hardwareobjekt haelt nur den neuesten Frame: Bei hohem Busaufkommen bzw.
waehrend blockierender OLED-Ausgaben koennen Frames ueberschrieben werden.
RX zaehlt abgeholte Frames und L die Ueberlaufmeldungen, nicht die genaue Anzahl
aller gesendeten bzw. verlorenen Frames. Beide Anzeigezaehler sind aus Platzgruenden
modulo 1000000; intern bleiben sie uint32_t. Die Anwendung ist kein CAN-Logger.
Die geerbte OLED-Bibliothek besitzt weiterhin unbeschraenkte I2C-Warteschleifen;
ein elektrisch blockierter OLED-Bus kann das Hauptprogramm anhalten.

## In DAVE 4 importieren und bauen

1. File -> Import -> General -> Existing Projects into Workspace.
2. D:/GITHUB/XMC14/CAN_RX als Root Directory auswaehlen, Projekt CAN_RX importieren.
3. Debug-Konfiguration bauen. Header und Bibliothek werden automatisch gefunden.
4. Neue Debug-Konfiguration fuer CAN_RX/Debug/CAN_RX.elf anlegen und das Kit waehlen.

.project, .cproject, DAVE-Modell, Startup und Linkerskript sind enthalten.
Die Build-Rezepte in Debug erlauben auch den direkten Build mit der vorhandenen
DAVE-ARM-GCC-4.9-Toolchain. DAVE darf diese Rezepte beim Build neu generieren.
Generierte DAVE-APP-Quellen wurden nicht manuell veraendert. Anwendungsquellen
liegen in main.c und Libraries/CAN/CANlib.c/.h.

## Empfangstest mit CANalyzer 12 und VN7610

1. CAN-Kanal zuordnen und 500 kbit/s, Classic CAN, normalen aktiven Betrieb einstellen.
2. Standard-Datenframe mit ID 0x123, DLC 8 und Daten 01 23 45 67 89 AB CD EF
   zunaechst alle 100 ms senden. Erwartet: D0: 01 23 45 67, D4: 89 AB CD EF.
3. Andere ID (z.B. 0x124) und Extended-ID 0x123 senden: RX darf sich dadurch
   nicht erhoehen und die angezeigten Nutzdaten duerfen sich nicht aendern.
4. Passende Frames mit DLC 0, 1, 4 und 8 pruefen; unbenutzte Bytes muessen -- zeigen.
5. Senden beenden: nach einer Sekunde ALT. Ohne jemals empfangenen Frame: WARTE AUF ID.
6. Fuer einen Extended-Test CANlib.h auf 29U/0x18FF0123UL aendern, neu bauen/flashen
   und einen passenden Extended-Datenframe senden.

Kein Flashen und kein Test am angeschlossenen Board durchgefuehrt. Der Build
prueft die Softwareintegration, nicht die elektrische Funktion des CAN-Busses.
Die vorhandenen Warnungen der OLED-Fonttabellen stammen aus der Vorlage.

## Durchgefuehrte Softwarepruefungen

- Vollstaendiger Debug-Build mit DAVE ARM-GCC 4.9: ELF und HEX erzeugt.
- 18 Compilerpruefungen fuer main.c und CANlib.c mit -Wall -Wextra -Werror:
  Standard-/Extended-ID, jeweiliger maximaler ID-Wert sowie erwartete Ablehnung
  von falscher ID-Laenge, ID-Ueberlauf, negativer ID und Bitrate 0.
- Wiederholbar mit `python Tests/validate_config.py`.
- Doxygen ohne Warnungen erzeugt.
- Dateivergleich bestaetigt: Das urspruengliche TOF-Projekt wurde nicht geaendert.

Die Pruefungen ersetzen weder die Inbetriebnahme auf dem Board noch einen
Empfangstest mit CANalyzer. DAVE-Import und Flashen wurden nicht ausgefuehrt.

## Dokumentation

Im Projektverzeichnis doxygen Doxyfile ausfuehren. Einstieg:
Debug/doxygen/html/index.html. Die Anwendung und CANlib besitzen deutsche
Doxygen-Kommentare fuer Einsteiger.


## TOF_CAN-Datenformat und Oszilloskop

CAN_RX verwendet jetzt dasselbe acht Byte lange Paket wie TOF_CAN:

| Byte (ab 1) | C-Index | Inhalt |
| --- | --- | --- |
| 1 | data[0] | Botschaftszaehler in Bit 0..3, 0..15 |
| 2 | data[1] | PWM als int8_t, ganze Prozent |
| 3–4 | data[2..3] | Hoehe als uint16_t, Little Endian, mm |
| 5–8 | data[4..7] | Reserviert |

Beispiel `05 32 96 00 00 00 00 00`: Zaehler 5, PWM 50 %, Hoehe 150 mm.
Der Wert 0xFFFF kennzeichnet eine ungueltige Hoehe. Nur passende ID/ID-Laenge
und DLC exakt 8 werden als TOF-Paket angenommen. Reservierte Bits werden ignoriert.
PWM wird vorzeichenrichtig dekodiert (0xFF = -1, 0x80 = -128); der Sender nutzt
aktuell 0..100 %. CAN_PWMPercent und CAN_TOFCounter sind im Debugger sichtbar.
Die Textansicht zeigt PWM und Zaehler in der letzten Zeile. Alte Werte bleiben
bei Empfangsausfall sichtbar und werden durch die Statuszeile ALT gekennzeichnet.

Eine gueltige empfangene Hoehe unter 300 mm aktiviert das Oszilloskop.
Ein neu abgeholter Frame erzeugt genau einen Punkt; ohne neue Botschaft wird
nicht weitergezeichnet. Ab 300 mm, bei ungueltigem Paket/0xFFFF, CAN-Warnung,
Bus-off, OLED-Fehler oder nach CANLIB_RX_TIMEOUT_MS (1000 ms) ohne Empfang
erscheint wieder die Textansicht. Es gibt keine Hysterese. CAN_TOF_SCOPE_LIMIT_MM
in main.c legt die Umschaltgrenze und Y-Skala fest.

Die GUI aus TOF_CAN schreibt pro Punkt nur zwei Spalten (zwoelf Grafikbytes).
Ein Durchlauf hat 128 Punkte; bei 100 ms Sendeintervall nominal 12,8 Sekunden.
Empfangsluecken werden nicht als Leerstellen eingefuegt. Die Zeitachse zaehlt
abgeholte Frames und ist keine exakte Echtzeitachse. CAN_RX zeichnet den vom
Sender uebertragenen geglaetteten Wert, nicht dessen lokalen Sensorrohwert.
Die PWM-Hardware in CAN_RX bleibt deaktiviert; der empfangene PWM-Wert wird
nur ausgewertet und angezeigt, nicht auf einen Ausgang uebertragen.

Tests/receive_test.c prueft die neue Bytebelegung, signed PWM, 299/300-mm-Grenze,
ungueltige DLC/IDs, Fehlerwert und Timeout. Tests/validate_receive.py benoetigt
unicorn und pyelftools zur Ausfuehrung des ARM-Codes mit simuliertem Speicher.
Firmware: Release/CAN_RX.elf und Release/CAN_RX.hex. Ein Hardwaretest steht aus.
