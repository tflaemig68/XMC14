# CAN_RX: Gruppenmonitor fuer 32 Hoehenregler

Empfang: Classic CAN, 500 kbit/s, 11-Bit-IDs 0x101..0x120. Die Rand-IDs 0x101 und 0x120 haben einzelne Empfangsobjekte; dazwischen
teilen jeweils zwei benachbarte IDs einen Filter (0x102/103 bis 0x11E/11F).
Alle Gruppen bleiben gleichzeitig empfangsbereit;
CANLIB_Receive fragt sie reihum fair ab. MO0..MO16 dienen dem Empfang, MO17 sendet Sollwerte auf 0x100.
Die CAN-Hardware bestaetigt passende Frames weiterhin mit ACK.
Die alte Einzel-ID-/Oszilloskopansicht ist durch diese Uebersicht ersetzt.

## Telemetrieformat (TOF_PID_PWM ebenfalls aktualisieren und flashen)

DLC 8; Gruppe = ID - 0x100:

| Byteoffset | Inhalt |
| --- | --- |
| 0 | Zaehler in unteren 4 Bit |
| 1 | PWM als int8, Prozent |
| 2..3 | Isthoehe uint16 Little Endian, 0xFFFF ungueltig |
| 4..5 | Wirksame Sollhoehe int16 Little Endian, -1 ungueltig |
| 6 | Formatkennung 2 (Version 1 weiterhin empfangbar) |
| 7 | Bei Version 2 Bit 0: UART-Vorgabe 0, CAN-Sollwertbetrieb; Rest 0 |

Alte Sender mit Byte 6 = 0 erscheinen als Gruppe, aber mit Sollwert `---`;
ISE wird ohne gueltigen Sollwert nicht integriert. CAN_RX leitet den Sollwert
nicht aus zentralen Befehlen ab, sondern verwendet den tatsaechlich im Regler
wirksamen Wert. Dadurch funktioniert die Anzeige auch bei direkter UART-Vorgabe.

## Anzeige und Seiten

Eine Gruppe ist vorhanden, wenn ihr letzter vollstaendiger Frame weniger als
1000 ms alt ist (CANLIB_RX_TIMEOUT_MS). Jede vorhandene Gruppe belegt eine
8-Pixel-Zeile; die Sortierung erfolgt aufsteigend nach Gruppennummer.
Die 128x64-Anzeige hat acht Zeilen ohne eigene Kopfzeile:

```text
01 150 145     125
02 200 198      80
```

Spalten: **Gruppe, Sollhoehe [mm], Isthoehe [mm], ISE60 [mm^2*s]**.
ISE wird ganzzahlig rechtsbuendig in sechs Stellen ohne fuehrende Nullen dargestellt.
Werte groesser als 999999 mm^2*s erscheinen als `XXXXXX`,
schwarze Schrift auf weissem Grund ausschliesslich im ISE-Feld. Ungueltige Hoehe 0xFFFF oder fehlender
Sollwert wird als `---` angezeigt. Istwerte ueber 300 mm werden angezeigt,
aber nicht in die ISE aufgenommen (Auswertebereich 0..300 mm).

Maximal acht Gruppen pro Seite; bei mehr Gruppen zyklischer Wechsel alle 5 s.
Bei geaenderter Seitenzahl beginnt die Anzeige wieder auf Seite 1. Jede Zeile
wird alle 25 ms aktualisiert (komplette Seite nominell 200 ms), sodass CAN
zwischen den Zeilen abgefragt werden kann. Eine Seitenumstellung ist daher
innerhalb eines Bilddurchlaufs sichtbar. Leere Zeilen werden geloescht.
Ohne aktive Gruppen erscheint `Warte auf Gruppen`. Bei Bus-off Reset-Hinweis.
Fehler-LED zeigt CAN-/OLED-Fehler, die andere LED blinkt weiter im Sekundentakt.

## 60-Sekunden-ISE

ISE = Integral (Sollhoehe - Isthoehe)^2 dt, Einheit mm^2*s, ohne Normierung.
Der letzte gueltige Regelfehler gilt zeitlich konstant bis zum naechsten Frame
(Zero Order Hold), maximal jedoch bis 1000 ms nach dessen Empfang.
Anschliessend oder bei ungueltigen Daten wird kein Fehler integriert.
Fehlende Daten sind damit keine Nullfehler-Messung; die Kennzahl ist bei
Messluecken nur ueber die beobachtete gueltige Zeit aussagekraeftig.

Pro Gruppe speichert ein Ringpuffer 60 Einsekundenintegrale in mm^2*ms.
Angezeigt wird die Summe der letzten **60 abgeschlossenen Sekunden**; die
laufende Teilsekunde folgt beim naechsten Abschluss. Damit ist das Fenster
60 s breit und der Wert um weniger als eine Sekunde verzoegert. Die Sekunden
beginnen relativ zum ersten Gruppenempfang. In den ersten 60 s wird die bisher
vorhandene Zeit summiert; es wird nicht auf 60 s hochgerechnet.
Bei Wiederkehr einer Gruppe bleibt noch im Fenster liegende Historie erhalten.
Es gibt keinen Reset bei Sollwertwechsel. Nach Reset beginnt die ISE bei 0.

32 * 60 uint32-Speicherstellen benoetigen 7680 Bytes fuer die Sekundenwerte;
64-Bit-Summen verhindern Ueberlauf. Bei 300 mm Fehler ergibt sich maximal
5.400.000 mm^2*s pro 60-s-Fenster. Die Implementierung verwendet keinen Heap.

## Grenzen und Tests

Je Empfangsobjekt wird nur der neueste Frame gespeichert; die Gruppenpaare
teilen diesen Speicher. Aeltere
Frames koennen ueberschrieben werden, die Bibliothek zaehlt erkannte MSGLST-
Ereignisse. Der Monitor ist kein verlustfreier Logger; die ISE wird aus den
abgeholten Werten und deren Empfangszeit im Hauptprogramm angenaehert.
Die uebrigen geerbten Bibliotheken bleiben im Projekt, die Anzeige verwendet
kein Oszilloskop mehr. PWM wird nach DAVE_Init auf 0 gesetzt und gestoppt.

Tests/validate_receive.py prueft echten ARM-Code im Emulator: Gruppenfilter,
Sortierung, Timeout, 60-s-Fenster, ungueltige Daten, Uhrueberlauf und Seiten.
Tests/validate_config.py prueft die betroffenen C-Dateien mit Warnungen als Fehler.
Ein Test mit 32 realen Sendern und der OLED-Hardware steht aus.

## Automatische Zufallssollwerte

Aktive Gruppen mit UART-Vorgabe 0 erhalten alle 5 s einen Pseudozufallswert
im inklusiven Bereich 100..200 mm. In jedem Zyklus wird genau ein Wert gezogen
und auf ID 0x100 je Gruppe ein DLC-3-Frame `Gruppe Hoehe_L Hoehe_H` gesendet.
Die erste Aussendung erfolgt 5 s nach Erkennen der ersten anfordernden Gruppe.
Spaeter hinzugekommene Gruppen nehmen am naechsten Zyklus teil.
Die Pseudozufallsfolge ist nicht kryptographisch und kann Werte wiederholen.

Version 2 der TOF_PID_PWM-Telemetrie meldet die UART-Quellenauswahl explizit
in Byte 7 Bit 0. Dadurch bleiben weitere Vorgaben aktiv, obwohl die wirksame
Sollhoehe nach der ersten CAN-Vorgabe nicht mehr 0 ist. Auch eine noch fehlende
CAN-Vorgabe (Sollwert -1) wird bei gesetztem Quellenbit versorgt.
Direkte UART-Vorgabe (Bit 0 geloescht) oder Empfangstimeout beendet die Teilnahme.
Version-1-Sender fordern nur an, solange ihre gemeldete Sollhoehe tatsaechlich 0 ist.

Pro Hauptschleife wird maximal ein Auftrag versucht. BUSY behaelt den Auftrag
bis zum naechsten Versuch. Bei einem neuen 5-s-Zyklus ersetzen die aktuellen
Auftraege eine noch nicht abgearbeitete Liste. Ausgefallene Gruppen werden
vor der Uebergabe entfernt. Bereits an die Hardware uebergebene Auftraege koennen
noch abgeschlossen oder automatisch wiederholt werden. Kein blockierendes Warten.
Beide Projekte TOF_PID_PWM und CAN_RX fuer dauerhaften Automatikbetrieb flashen.

## TX-Anzeige

Hinter dem sechsstelligen ISE-Feld erscheint gruppenbezogen fuer eine Sekunde
`TX`, sobald der Sollwert-Sendeauftrag von der CAN-Bibliothek angenommen wurde.
Bei BUSY oder ERROR wird keine neue TX-Anzeige ausgeloest. Die Anzeige bestaetigt
die Uebergabe an die CAN-Hardware, nicht den Empfang oder die Sollwertuebernahme
im entfernten Regler. Die inverse ISE-Ueberlaufanzeige bleibt auf das ISE-Feld
begrenzt. Format: `01 150 145    125 TX`.

## UART-Ausgabe

USB-COM, 128000 Baud 8N1: zyklisch je vorhandener Gruppe acht Bytes
`5A Gruppe Ist_L Ist_H Soll_L Soll_H ISE_L ISE_H`.
Ist und Soll sind int16; ISE ist uint16 in ganzen mm^2*s (auf 65535 begrenzt), alles Little Endian.
Aufsteigender Gruppenumlauf fruehestens alle 50 ms, ohne UART-Eingabebefehle.
Details: [UART_Spezifikation.md](UART_Spezifikation.md).
