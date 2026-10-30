# TestP05 - P0.5 und LED2 im 5-Sekunden-Takt

DAVE4-Projekt fuer KIT_XMC14_2Go / XMC1404-Q040x0200, abgeleitet aus der
bewaehrten DAVE4-Projektvorlage. Keine externe Beschaltung fuer die LED erforderlich.

| Zeit ab Timerstart | P0.5 | LED2 (P0.6) |
| --- | --- | --- |
| 0 bis 5 s | LOW / AUS | aus |
| 5 bis 10 s | HIGH / EIN | ein |
| 10 bis 15 s | LOW / AUS | aus |
| fortlaufend | Umschaltung alle 5 s | synchron |

P0.5 ist ein normaler Push-Pull-Ausgang, kein PWM-Alternativausgang.
Beide Ausgaenge werden mit einem gemeinsamen OMR-Portzugriff gesetzt.
Die Testperiode ist 10 s mit jeweils 5 s HIGH und 5 s LOW.
Die Quarz-/DCO-Toleranz und SYSTIMER-Aufloesung gelten fuer die Zeitangaben.

main.c definiert eine eigene DAVE_Init-Funktion und ueberschreibt damit die
schwache generierte Variante. Nur CLOCK_XMC1 und SYSTIMER werden gestartet.
Die in der geerbten DAVE-Modellvorlage noch enthaltenen PWM-/I2C-APPs werden
nicht initialisiert. Dadurch startet die vorhandene 50-Prozent-PWM nicht auf
P0.5. Generierte Dateien bleiben unveraendert; auch erneutes Generieren
ueberschreibt die eigene Init in main.c nicht.

Im Debugger: TestP05_On, TestP05_SwitchCount und TestP05_InitStatus.
Bei Init-/Timerfehler bleiben beide Ausgaenge LOW und die Anwendung haelt an.
Am Reset bis zum Eintritt in main gelten die Hardware-Resetzustaende;
die Software ersetzt keinen externen Gate-Pull-down.

## In DAVE4 oeffnen

File -> Import -> General -> Existing Projects into Workspace.
Ordner D:/GITHUB/XMC14/TestP05 auswaehlen, Projekt TestP05 importieren.
Release oder Debug bauen und fuer TestP05 eine Debug-/Run-Konfiguration erstellen.
Build-Artefakte nach dem Release-Build: Release/TestP05.elf und TestP05.hex.
Vorhandene TOF-Projekte und deren Debug-Konfigurationen bleiben erhalten.

## T13 pruefen

LED2 aus: P0.5 gegen GND soll nahe 0 V liegen. LED2 ein: nahe 3.3 V.
Gate gegen Source von T13 mitmessen. Der Test zeigt den Ausgangszustand,
nicht den tatsaechlichen Drainstrom. Bleibt die Last trotz 0 V Gate-Source
leitend, Beschaltung/Bestueckung und die moegliche GND-Bruecke im Schaltplan
pruefen. Die Platine wird durch diesen Softwaretest nicht veraendert.

Zusaetzlich liegen gepruefte Firmwaredateien unter Firmware/TestP05.elf und
Firmware/TestP05.hex. Release/Debug werden von DAVE bei Bedarf neu erzeugt.
