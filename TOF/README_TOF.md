# TOF-Test fuer DAVE 4 / XMC1404

Die Anwendung in main.c basiert auf D:/GITHUB/STM32/EDS_2024/TOFTest/Src/main.c.
Eine separate Datei namens TOFTest.c war dort nicht vorhanden.
Die STM32-Menuebedienung mit Drehgeber und ST7735 wird auf dieser DAVE-Hardware
als automatisch startender Distanztest auf dem vorhandenen SSD1306-OLED umgesetzt.

## Anschluss und Ablauf

- I2C_MASTER_0: SDA P2.10, SCL P2.11, 400 kHz (vorhandene DAVE-Konfiguration).
- VL53L0X: 7-Bit-Adresse 0x29; im DAVE-Transfer wird daraus 0x52.
- SSD1306: 7-Bit-Adresse 0x3C, bestehende OLED-Bibliothek verwendet 0x78.
- Sensor und OLED teilen den Bus; passende Pull-ups und gemeinsame Masse vorausgesetzt.
- LED P0.7: Lebenszeichen im Sekundentakt. LED P0.6: Fehler.
- PWM_0 an P0.5: abstandsabhaengiger Sollwert fuer die Geblaeseansteuerung;
  Aktualisierung nach jeweils mindestens 200 ms.

Nach 500 ms Startzeit: OLED initialisieren, Sensor-ID pruefen, Sensor initialisieren
und kalibrieren, Standardprofil (30 ms Messbudget) setzen, kontinuierlich messen.
Messwerte werden im Standardprofil etwa alle 35 ms gelesen. Der gleitende
Mittelwert verwendet aktuell vier gueltige Messwerte (TOF_MEAN_WINDOW_SIZE).
Die Distanzanzeige wird nach jeweils mindestens 140 ms aktualisiert (DisplayRate). Messwerte
8190/8191 werden als ausserhalb der Reichweite angezeigt. Bei TOF-Fehlern bleibt
die Fehleranzeige stehen; nach Beheben des Fehlers das Board neu starten.
TOF_Sensor_1, initedTOF und tof_error_count sind im Debugger sichtbar.

## PWM0 fuer die Geblaeseansteuerung

PWM_0 gibt an P0.5 einen Tastgrad aus, der vom geglaetteten TOF-Abstand abhaengt.
Der Abstand wird direkt als Hoehe verwendet, ohne geometrische Umrechnung.
Die Grenze MAX_HEIGHT_mm betraegt aktuell 300 mm. Bis einschliesslich dieser
Grenze wird in ganzen Prozent gerechnet:

`PWM_Value = 100 * (MAX_HEIGHT_mm - distance_mean_mm) / MAX_HEIGHT_mm`

Oberhalb der Grenze ist PWM_Value gleich 0. Die Ganzzahldivision schneidet
Nachkommastellen ab. Der DAVE-Aufruf erhaelt `100 * PWM_Value`, weil die APP
Hundertstel-Prozent im Wertebereich 0 bis 10000 erwartet.

| Geglaetteter Abstand | PWM_Value | Wert fuer DAVE |
| --- | --- | --- |
| 0 mm | 100 % | 10000 |
| 100 mm | 66 % | 6600 |
| 150 mm | 50 % | 5000 |
| 300 mm oder mehr | 0 % | 0 |

Der Sollwert und seine Prozentanzeige werden nach jeweils mindestens 200 ms
aktualisiert. Sensor- und Displayzugriffe koennen dieses Intervall verlaengern.
Die Hardware erzeugt das PWM-Signal unabhaengig davon fortlaufend; seine
Frequenz wird durch die DAVE-Konfiguration bestimmt. Die Prozentanzeige steht
auf OLED-Seite 5 und zeigt den berechneten Tastgrad, keine gemessene Drehzahl
oder Rueckmeldung des Geblaeses.

Die aktuelle DAVE-Konfiguration startet PWM_0 bereits bei DAVE_Init() mit 50 %.
Erst nach der Initialisierung setzt die Hauptschleife den abstandsabhaengigen
Wert. Der Ersatzwert 8190 fuer eine ungueltige Distanz ergibt mit der aktuellen
Hoehengrenze beim naechsten PWM-Update 0 %. Bei einem I2C-Lesefehler bleibt
hingegen der letzte Mittelwert gespeichert; der PWM-Zweig laeuft ohne Pruefung
von initedTOF weiter. Ein Halt vor der Hauptschleife setzt keinen neuen Tastgrad.
Dies beschreibt das vorhandene Verhalten; die Dokumentation aendert es nicht.
MAX_HEIGHT_mm muss groesser als 0 und fuer die bestehende Fehlerwertbehandlung
kleiner als TOF_VL53L0X_OUT_OF_RANGE sein.

## Portierung

Libraries/TOF/i2cTOF.h verwendet I2C_MASTER_t statt I2C_TypeDef.
Libraries/TOF/tof_dave_port.inc wird direkt von i2cTOF.c eingebunden und nicht
separat kompiliert. Die bisherigen Register-Helfer verwenden DAVE-Bytefunktionen:
START, Registeradresse, bei Lesen REPEATED START, ACK fuer Zwischenbytes,
NACK fuer das letzte Empfangsbyte und STOP. 16-Bit-Register werden MSB zuerst
uebertragen; die byteweise Tuning-Tabelle bleibt byteweise erhalten.

Die TOF-Transfers haben 20 ms Timeout pro Wartephase; Sensor-Ready-Schleifen
brechen nach 1000 ms ab. Der NACK-Interrupt wird waehrend eines TOF-Transfers
voruebergehend maskiert, damit der generierte Handler das Fehlerflag nicht vor
der Polling-Pruefung loescht. SYSTIMER bleibt aktiv. Alle I2C-Zugriffe laufen im
Hauptprogramm, der Timer-Callback erhoeht ausschliesslich den Millisekundenzaehler.
Die Portierung setzt die vorhandene DIRECT/FIFO-Konfiguration voraus.

Zusatzkorrekturen in der Sensorbibliothek: Fehlercodes mit OR verknuepfen,
Rueckgaben zuvor ungepruefter Initialisierungsschritte beachten, Zeitregister als
16 Bit lesen/schreiben, korrekten final-range-Wert initialisieren und numerische
Rohdistanz statt Pufferadresse speichern. Verwendet wird das Standardprofil;
die weiteren urspruenglichen Profile sind nicht auf Hardware verifiziert.

Die generierten DAVE-APP-Quellen wurden nicht geaendert. Projekt-Includepfade und
Build-Verzeichnisse verweisen auf TOF statt auf das benachbarte I2C-Projekt.

## Verifikation und Grenzen

Vollstaendiger Debug-Neuaufbau mit der installierten DAVE ARM-GCC-4.9-Toolchain:
Uebersetzen, Linken und HEX-Erzeugung erfolgreich. Bestehende Warnungen in
Libraries/OLED/oledfont.h betreffen Font-Initialisierungen. Keine Warnungen in
main.c oder der portierten TOF-Bibliothek. Ausgabe: Debug/TOF.elf und Debug/TOF.hex.

Kein Flashen und kein Sensor-/Logikanalysator-Test durchgefuehrt. Fuer die
Inbetriebnahme: plausible Distanz bei mehreren Abstaenden, Reichweitenanzeige,
fehlenden TOF-Sensor beim Start und Unterbrechung der Sensorverbindung pruefen.
Die bestehende OLED-Bibliothek verwendet weiterhin den generierten DIRECT-Treiber
mit unbeschraenkten Wartezeiten. Ein fehlendes OLED oder elektrisch blockierter
Gesamtbus kann deshalb weiterhin in der Displayausgabe blockieren; die TOF-
Timeouts ersetzen keine elektrische Bus-Recovery.

## Kommentare und Doxygen

Die Anwendungsdatei main.c, i2cTOF.c, i2cTOF.h und der eingebundene I2C-Port
enthalten deutsche Erklaerungen fuer Einsteiger. Die Funktionskommentare
beschreiben Zweck, Parameter, Einheiten und Rueckgaben. Inline-Kommentare
zeigen unter anderem Zeigerzugriff, Zeitstempel, Bitmasken und Byte-Reihenfolge.

Im Projektverzeichnis `doxygen Doxyfile` ausfuehren (Doxygen muss im Suchpfad
liegen). Alternativ die mit DAVE gelieferte doxygen.exe mit Doxyfile aufrufen.
Die HTML-Dokumentation beginnt bei Debug/doxygen/html/index.html. Der gepruefte
Lauf mit Doxygen 1.8.3.1 erzeugt keine Warnungen; Meldungen stehen gegebenenfalls
in Debug/doxygen-warnings.log. Generierte DAVE- und fremde Bibliotheksdateien
sind nicht Teil dieser Anwendungsdokumentation.
