> Aktuelles CAN-Protokoll: [CAN_Spezifikation.md](CAN_Spezifikation.md).
> TX: 0x100 + Gruppe; ohne Gruppe kein Senden. UART-Sollwert 0 waehlt CAN als Quelle.

> Projekt TOF_PID_PWM: Die hier beschriebene lineare PWM-Kennlinie ist durch
> den PID-Hoehenregler ersetzt. Aktuelle Regelung und Startverhalten: [README_PID.md](README_PID.md).

# TOF_CAN ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â€šÂ¬Ã…â€œ TOF-Messwert auf Classic CAN

Eigenstaendiges DAVE4-Projekt fuer KIT_XMC14_2Go / XMC1404-Q040x0200,
kopiert aus TOF. Die CANlib stammt aus CAN_RX und wurde nur in diesem neuen
Projekt um die Sendefunktion erweitert. TOF und CAN_RX bleiben unveraendert.
OLED, gleitender Mittelwert (vier Werte), LEDs und PWM aus TOF bleiben aktiv.
Die Details zu Sensor, I2C und PWM stehen in README_TOF.md.

## Projekt in DAVE4 verwenden

1. File ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ Import ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ General ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ Existing Projects into Workspace.
2. D:/GITHUB/XMC14/TOF_CAN als Root Directory auswaehlen.
3. TOF_CAN importieren; bei bestehendem Verzeichnis keine weitere Kopie anlegen.
4. Als aktives Projekt setzen und Build Project ausfuehren.
5. Eine J-Link-Konfiguration fuer TOF_CAN und Debug/TOF_CAN.elf verwenden.

Die CANlib verwendet die enthaltene XMCLib direkt; eine neue CAN-DAVE-APP
ist nicht erforderlich. CAN0 sowie Message Object 0 (RX) und 1 (TX) sind
reserviert. P1.0 und P1.1 bei spaeteren DAVE-Aenderungen freihalten.
Die vorhandenen Debug-Buildrezepte erlauben auch einen Build ausserhalb der IDE;
DAVE darf diese beim naechsten Build neu erzeugen. Tests ist von den
Firmware-Quellen in beiden Buildkonfigurationen ausgeschlossen.

## Einstellungen

In Libraries/CAN/CANlib.h:

| Parameter | Vorgabe | Bedeutung |
| --- | --- | --- |
| CANLIB_ID_BITS | 11U | Gemeinsames ID-Format: 11 oder 29 Bit |
| CANLIB_TX_ID | 0x123UL | ID der gesendeten TOF-Botschaft |
| CANLIB_RX_ID | 0x123UL | Empfangsfilter der weiterhin nutzbaren Empfangsfunktion |
| CANLIB_BITRATE | 500000UL | 500 kbit/s |

Der Standard passt zum vorhandenen CAN_RX-Empfaenger. Fuer Extended CAN
zum Beispiel CANLIB_ID_BITS = 29U und CANLIB_TX_ID = 0x18FF0123UL einstellen;
der Empfaenger muss dasselbe Format und dieselbe Empfangs-ID verwenden.
Die ID-Laenge hat keinen Einfluss auf die acht Nutzdatenbytes.
Die Beispielanwendung ruft CANLIB_Receive() nicht auf.

In main.c legt TOF_CAN_PERIOD_MS den Mindestabstand zwischen Sendeversuchen
fest (100 ms). I2C- und OLED-Zugriffe koennen diesen Abstand verlaengern;
es handelt sich nicht um einen exakt periodischen Interruptsender.
Gesendet wird der zuletzt berechnete gleitende Mittelwert in Millimetern.
Die CAN-Uebertragung beginnt nach Abschluss der Sensorinitialisierung.

## Datenformat: DLC = 8

Die Nummerierung unten beginnt bei 1; C-Arrayindizes beginnen bei 0.
Der bisherige 4-Bit-Zaehler (0..15) bleibt im ersten Byte erhalten.

| Byte | C-Index | Inhalt |
| --- | --- | --- |
| 1 | data[0] | Botschaftszaehler 0..15, obere vier Bits 0 |
| 2 | data[1] | PWM als int8_t, ganze Prozent (aktuell 0..100) |
| 3 | data[2] | Hoehe uint16_t, niederwertiges Byte |
| 4 | data[3] | Hoehe uint16_t, hoeherwertiges Byte |
| 5ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“8 | data[4..7] | Reserviert, 0 |

Hoehe bezeichnet den geglaetteten TOF-Abstand in mm. Little Endian / Intel,
Startbit 16, Laenge 16, unsigned, Faktor 1, Offset 0. PWM: Startbit 8,
Laenge 8, signed, Faktor 1, Offset 0, Einheit Prozent.
Zaehler: Startbit 0, Laenge 4. Er erhoeht sich nur bei angenommenem Auftrag.

Beispiel: Zaehler 5, PWM 50 Prozent und Hoehe 150 mm:
`05 32 96 00 00 00 00 00`.
0xFFFF bleibt der Ersatzwert fuer eine ungueltige Hoehe.
Gesendet wird nach einem eventuell faelligen PWM-Update, sodass der PWM-Wert
dem zuletzt gesetzten Sollwert entspricht. Vor dem ersten Update gilt der
DAVE-Startwert von 50 Prozent. Bei Sensorfehlern bleibt das bestehende
PWM-Verhalten erhalten; ein ungueltiger Abstand erzwingt keinen neuen PWM-Wert.

```c
uint8_t counter = data[0] & 0x0FU;
int8_t pwm_percent = (int8_t)data[1];
uint16_t distance_mm = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
/* distance_mm == UINT16_MAX bedeutet ungueltig. */
```

Achtung: CAN_RX verwendet noch das bisherige Format mit Hoehe in data[1..2].
Vor gemeinsamem Betrieb muss dessen Decoder auf data[2..3] angepasst werden.
Diese Aenderung betrifft ausschliesslich TOF_CAN.

## Sendefunktion und Fehlerverhalten

CANLIB_Send(data, length) kopiert 0 bis 8 Bytes in MO1 und startet einen
Sendeauftrag mit CANLIB_TX_ID. Der Puffer muss gueltig sein und darf nach dem
Aufruf wiederverwendet werden. Die Funktion darf nur aus der Hauptschleife,
nicht gleichzeitig aus mehreren Ausfuehrungskontexten aufgerufen werden.

- CANLIB_TX_ACCEPTED: Auftrag uebernommen, noch keine Bestaetigung vom Bus.
- CANLIB_TX_BUSY: Ein vorheriger Auftrag laeuft. Keine Nutzdaten werden geaendert.
- CANLIB_TX_ERROR: Ungueltiger Aufruf, CAN nicht initialisiert oder Bus-off.

TOF_CAN_SendDistance() erhoeht den Zaehler nur bei ACCEPTED, modulo 16.
Bei BUSY/ERROR entfaellt dieses Sendeintervall. Der naechste Versuch verwendet
den aktuellen Messwert und denselben noch nicht vergebenen Zaehler.
Die Hardware wiederholt einen fehlgeschlagenen Auftrag mit unveraendertem
Zaehler und Messwert automatisch; ohne aktiven ACK-Teilnehmer kann dieser
Auftrag dauerhaft ausstehen. Eine spaetere Zustellung kann deshalb einen alten
Messwert enthalten. Es gibt keine Sendewarteschlange, keine Ablaufzeit fuer
Auftraege und keine automatische Bus-off-Wiederanmeldung (Board neu starten).

Im Debugger sichtbar: can_tx_accepted_count, can_tx_busy_count,
can_tx_error_count. Diese zaehlen API-Ergebnisse, keine bestaetigten Frames.
Die OLED-Statuszeile zeigt Initialisierungsfehler, Bus-off, belegtes Sendeobjekt
oder angenommenen Auftrag. CAN-Fehler verhindern die TOF/PWM-Verarbeitung nicht.
Das geerbte PWM-Verhalten einschliesslich Starttastgrad und Verhalten bei
Sensorfehlern ist in README_TOF.md beschrieben.

## Anschluss und Test mit CANalyzer / CAN_RX

CAN nutzt den Onboard-Transceiver mit TX=P1.0/ALT9 und RX=P1.1.
CAN-H, CAN-L und CAN-GND mit dem Gegenknoten verbinden. Beide Busenden mit
je 120 Ohm abschliessen; den vorhandenen Abschluss auf dem Kit beruecksichtigen.
Der Gegenknoten muss aktiv ACK senden (kein reiner Listen-only-Betrieb).
Verdrahtungsdetails zum Vector VN7610 stehen in ../CAN_RX/README_CAN_RX.md.
Sensor und OLED bleiben an SDA=P2.10 und SCL=P2.11 angeschlossen.

1. Gegenknoten auf Classic CAN, 500 kbit/s, Standard-ID 0x123 einstellen.
2. Firmware laden und starten. DLC muss 8 sein, Byte 0 muss von 00 bis 0F laufen.
3. Abstand veraendern; Byte 3/4 (C-Indizes 2/3) als uint16 Little Endian dekodieren und mit dem
   geglaetteten OLED-Wert vergleichen. Wegen unterschiedlicher Aktualisierung
   koennen Momentaufnahmen voneinander abweichen.
4. Ungueltige Entfernung pruefen: Byte 3/4 muessen FF FF enthalten.
5. Ohne ACK pruefen, dass Anzeige und Messschleife weiterarbeiten und der
   offene CAN-Auftrag nicht ueberschrieben wird. Bus-off erfordert Neustart.

## Pruefstand

Vollstaendiger Debug-Build mit der installierten DAVE-ARM-GCC-4.9-Toolchain:
erfolgreich; Debug/TOF_CAN.elf und Debug/TOF_CAN.hex erzeugt.
24 Compilerpruefungen mit Tests/validate_config.py: gueltige und ungueltige
RX-/TX-IDs, Standard/Extended und Bitrate; main.c und CANlib.c mit
-Wall -Wextra -Werror geprueft. Die geerbten OLED-Fontdateien erzeugen beim
Gesamtbuild bereits vorhandene Initialisierungswarnungen.
Doxygen: Doxyfile erzeugt Debug/doxygen/html/index.html.
Die Firmware wurde hier nicht auf das Board geflasht; Busuebertragung,
Zaehlerfolge und Timing muessen noch an der Hardware geprueft werden.


## Laufendes OLED-Oszilloskop

Die GUI in Libraries/OLED/gui.c und gui.h stellt GUI_ScopeInit() und
GUI_ScopePlot() bereit. Die Vorlage wurde lokal als Libraries/OLED/graphics.c
gefunden (OSZIinit/OSZIrun, Prof. Flaemig); eine Datei graphics.c.demo lag nicht
vor. Das Prinzip der wandernden, spaltenweise geloeschten Schreibposition wurde
fuer das monochrome SSD1306 neu umgesetzt. Die unveraenderte STM32/TFT-Vorlage
ist in Debug und Release vom DAVE-Build ausgeschlossen.

Bei einem erfolgreich gelesenen **Rohabstand unter 300 mm** schaltet main.c
auf die Grafik. Jeder neue Rohwert erzeugt genau einen Punkt und rueckt die
Schreibposition um ein Pixel vor. Ab 300 mm, bei Reichweitenfehler oder bei
Lesefehler kehrt die Anzeige zur Textansicht zurueck. Ein Lesefehler zeigt
wieder die Fehlermeldung. Es gibt keine Hysterese: Schwankungen um 300 mm
koennen zwischen den Ansichten wechseln. TOF_SCOPE_LIMIT_MM legt die Grenze
und die obere Y-Skala fest. CAN und PWM nutzen weiterhin den gleitenden
Mittelwert; die Grafik zeigt bewusst den ungefilterten Verlauf.

- Y: 0 mm unten, 300 mm oben; Werte der allgemeinen GUI-Funktion oberhalb
  ihrer Skala werden begrenzt. Das Hauptprogramm zeichnet keine Fehlerwerte.
- X: ein Aufruf/eine neue Messung pro Spalte. Im Standardprofil nominal
  mindestens 35 ms pro Punkt, 128 Punkte pro Umlauf (etwa 4,48 s).
  Die Zeitbasis ist aufrufbezogen, keine unabhaengig gemessene Echtzeitachse;
  blockierende I2C-Transfers und Ansichtswechsel verlaengern die Intervalle.
- OLED-Seiten 2 bis 6 bilden die 128 x 40 Pixel grosse Grafik; Seiten 0/1 zeigen
  dauerhaft TOF_PID_PWM mit zweistelliger Gruppe (00 bis zum ersten Empfang),
  Seite 7 die Bedeutung der Zeitachse. Es gibt drei Rasterlinien.
- Die naechste Spalte ist gestrichelt markiert und bereits geloescht.
  Alte Spalten rechts davon bleiben bis zum Ueberschreiben erhalten; dies
  ist ein wiederholter Durchlauf, kein nach links geschobener Bildpuffer.
- Die normale Distanz-, Zeit-, CAN- und PWM-Textausgabe pausiert im Oszimodus,
  damit sie die Grafik nicht ueberschreibt. Messung, CAN und PWM laufen weiter.

Beispiel fuer die allgemeine GUI:

```c
GUI_Scope_t scope;
GUI_ScopeInit(&scope, 0, 1, 128, 6, 300);
/* Bei jeder neuen Messung, nicht bei jedem Hauptschleifendurchlauf: */
GUI_ScopePlot(&scope, distance_mm);
```

GUI_ScopeInit() prueft den Bereich und zeichnet ihn einmal mit einem
128-Byte-Seitenpuffer. GUI_ScopePlot() braucht keinen Bildpuffer und nur zwei
Datenbytes als Arbeitspuffer. Pro Punkt werden zwei Spalten auf jeder der
fuenf Seiten geschrieben: 10 Datenbytes plus I2C-Adressierungs-/Steuerbytes.
Benachbarte Spalten werden gemeinsam uebertragen; nur beim Umlauf sind zwei
getrennte Spaltenzugriffe pro Seite erforderlich. Rechenaufwand O(Seiten),
keine Gleitkommarechnung, kein Vollbildloeschen pro Messung. Der vorhandene
OLED-Treiber bleibt synchron und wartet auf I2C; die GUI fuegt keine eigene
Timeoutbehandlung hinzu. Ein Vollbildloeschen erfolgt beim Ansichtswechsel.

Validierung der Erweiterung:
- Release-Build erfolgreich: Release/TOF_CAN.elf und Release/TOF_CAN.hex.
- Tests/validate_scope.py fuehrt den echten ARM-GUI-Code mit einem simulierten
  OLED-RAM aus: Bereichspruefung, Y-Skalierung, Clipping, Cursor-Loeschung,
  Randumlauf, verschobener Plotbereich und konstantes Datenvolumen bestanden.
  Python-Abhaengigkeiten: unicorn und pyelftools; alternativ deren Verzeichnis
  als erstes Skriptargument uebergeben. Kein I2C-Hardwaretest.
- Die CAN-Konfigurationspruefungen laufen weiterhin durch.
- Doxygen umfasst nun auch gui.c/gui.h; HTML unter Debug/doxygen/html/index.html.
- Umschaltung und tatsaechliche I2C-Laufzeiten sind noch am Board zu pruefen.


## UART Ein-/Ausgabe

USB-COM ueber P2.1/P2.2, binaere Hoehe/Sollwert/PWM-Telemetrie und
einstellbarer Hoehensollwert: siehe [README_UART.md](README_UART.md).
