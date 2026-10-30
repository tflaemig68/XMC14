# PID-Laufzeitoptimierung, 30.10.2026

## Umsetzung

Die Positionsform bleibt bestehen. Vorberechnet werden KI*TA, KD/TA und
TA/(filter_time+TA). Ein Cache erkennt Aenderungen an KI, KD, TA und Filterzeit.
Bei konstanten Parametern hat der Regelschritt keine Division; der erste
PID-Aufruf bei Initialisierung berechnet die Koeffizienten. Bei KD=0 werden
weder D-Koeffizienten dividiert noch D-Schritte ausgefuehrt. Bei veraenderter
realer Abtastzeit wird der Cache neu berechnet, keine Jitterzeit unterschlagen.
Auch main rechnet Millisekunden mit *0.001f statt /1000.0f in Sekunden um.

Der I-Anteil lautet jetzt I_k=I_alt+(KI*TA)*e. ISUM speichert bereits den
gewichteten I-Stellwert in Prozent, nicht mehr Integral(e dt) in mm*s.
Der gefilterte D-Zustand speichert ebenso bereits den D-Stellwert in Prozent.
Diese Einheiten sind in PID.h dokumentiert. Im Debugger muss ISUM deshalb
nicht nochmals mit KI multipliziert werden. Bei KI=0 wird ISUM auf 0 gesetzt.

Bei Sollwertwechseln bleibt I erhalten, D wird wie zuvor neu gestartet.
Verstaerkungswechsel erfordern PIDclear, wie in TOF_UARTApply umgesetzt:
Ein gespeicherter gewichteter Zustand darf sonst nicht mit einem neuen
KI/KD verwechselt werden. Gruppen-/Quellen-/Fehlerresets bleiben erhalten.

Anti-Windup ist weiterhin bedingte Integration: Ein neuer I-Schritt, der den
Gesamtausgang weiter ueber 100 oder unter 0 treiben wuerde, wird verworfen.
Entgegengesetzte Schritte koennen den Integrator wieder abbauen. Der Ausgang
wird anschliessend begrenzt. Endlichkeitspruefungen sichern Koeffizienten,
Integrator und Ausgang gegen NaN/Unendlich ab. Es wird kein zusaetzlicher,
willkuerlicher I-Grenzwert eingefuehrt, der das bisherige Verhalten aendern wuerde.
Die IEEE-754-Endlichkeitspruefung und Cachevergleiche verwenden Integer-Bitmuster
statt vieler Software-Float-Vergleiche. Dies setzt den im XMC-ARM-GCC verwendeten
32-Bit-IEEE-754-float voraus. Es wird kein -ffast-math eingeschaltet.

Die alternative Rekursion y_k=y_alt+q0*e_k+q1*e_alt+q2*e_alt2 wird nicht
verwendet. Sie waere fuer den ungefilterten PID bei konstantem TA mit
q0=KP+KI*TA+KD/TA, q1=-KP-2*KD/TA, q2=KD/TA herleitbar. Variable Zeit,
D-Tiefpass und Anti-Windup wuerden eine angepasste Herleitung erfordern.
Die vorgesehene Vorberechnung wurde stattdessen in die vorhandene Positionsform
uebernommen. Fixed Point wird nicht eingefuehrt.

## Vergleich mit vorherigem Code

Tests/pid_reference.c bewahrt die Rechenfassung unmittelbar vor Optimierung
als Testreferenz. Sie wird durch den Tests-Quellausschluss nicht in die Firmware
gelinkt. Tests/validate_pid_compare.py vergleicht 8000 Schritte: P/PI/PID,
wechselnde Zeiten, positive/negative Verstaerkungen, Ausgangssaettigung,
Tiefpass und Reset. Ausgang sowie entsprechend gewichtete I-/D-Zustaende
werden mit einer absoluten Toleranz von 0.02 Prozentpunkten verglichen.
Diese ist kleiner als die 1-Prozent-Ausgabequantisierung. Die Rechnung ist
wegen geaenderter float-Operationsreihenfolge nicht bitidentisch; unmittelbar
an einer Rundungsgrenze kann auch die gerundete PWM um 1 Prozentpunkt abweichen.

Die bestehenden PID-Rechentests und UART-/CAN-Sollwerttests bestehen ebenfalls,
insbesondere der zuvor korrigierte Integratorerhalt bei laufendem Profil.

## Instruktionsvergleich

Tests/benchmark_pid.py baut mit DAVE GCC 4.9, Cortex-M0, -Os wie im Projekt.
Pro Fall werden Initialisierung plus 100 Regelschritte im Unicorn-Emulator
gezaehlt, einschliesslich der eingebundenen Software-Float-Routinen. Die
Vergleichssignale und Parameter sind in Tests/pid_benchmark.c festgelegt.

| Fall | Vorher: Instruktionen | Optimiert | Reduktion | Float-Divisionen vorher / optimiert |
| --- | ---: | ---: | ---: | ---: |
| PI, konstant 50 ms | 335222 | 84404 | 74.8 % | 198 / 0 |
| PID, konstant 50 ms | 341977 | 137975 | 59.7 % | 198 / 2 |
| PI, wechselnd 50/51/52 ms | 357395 | 131293 | 63.3 % | 198 / 0 |
| PID, wechselnd 50/51/52 ms | 364068 | 270313 | 25.8 % | 198 / 200 |

Bei konstantem TA sind die zwei PID-Divisionen nur in der Initialisierung
enthalten. Mit wechselndem TA und aktivem D-Anteil muessen die Koeffizienten
jedes Mal aktualisiert werden; die Reduktion stammt dann vor allem aus weniger
Multiplikationen, Vergleichen und der gewichteten Zustandsdarstellung.

**Diese Zahlen sind keine Taktzyklen, keine Mikrosekunden und kein gemessener
Hardware-Beschleunigungsfaktor.** Instruktionslaufzeiten, Flash-Wartezeiten und
Interrupts werden durch diese Zaehler nicht abgebildet. Fuer echte Laufzeitmessung
auf dem XMC1404 PIDrun mit einem GPIO-Puls einklammern und am Oszilloskop messen.
Referenz und Optimierung dabei mit gleichen Eingaben und Compileroptionen messen.
Die Rohdaten stehen in Tests/pid_benchmark_results.json.

## Code und Verwendung

Libraries/PID.c dokumentiert die Gleichungen und Gruende direkt am Code.
Libraries/PID.h beschreibt alle Zustands- und Cachefelder. Nur PIDinit/PIDrun
und die Resetfunktionen verwenden; Cachefelder nicht von aussen aendern.
Der Cache vergroessert den Reglerzustand um einige Worte; die vorher benoetigten
Software-Float-Bibliotheken koennen fuer andere Funktionen weiterhin im ELF bleiben.

Firmware neu bauen und flashen; die Abstimmung an der realen Regelstrecke bleibt
zu pruefen. Der Softwarevergleich ersetzt diese Messung nicht.
