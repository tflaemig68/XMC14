# Hoehenregelung TOF_PID_PWM

PID.c/PID.h basieren auf Libraries/regler.c.demo von Prof Flaemig,
V2.1 (Aug. 2025), Lizenz CC BY-NC-SA 4.0. Die unbenutzten Filter und
Funktionszeiger der Vorlage wurden nicht uebernommen.

Hoehe bezeichnet den geglaetteten TOF-Abstand in mm. Laut Aufbau nimmt dieser
mit hoeherer PWM zu. Deshalb gilt Fehler = Sollwert - Istwert.
TOF_Setpoint_mm in main.c ist der Sollwert; Startwert 300 mm, ueber das
UART-Parameterprotokoll im Bereich 0..300 mm einstellbar.
Die UART- und CAN-Datenformate bleiben erhalten. Die Textansicht zeigt
Sollwert und PWM. Unter 300 mm bleibt die vorhandene Oszilloskopansicht aktiv.

In main.c: HEIGHT_PID_KP=0.333333 Prozent/mm, KI=0.1 Prozent/(mm*s),
KD=0.02 Prozent*s/mm. Dies sind ungetestete Startparameter, keine am Aufbau
abgestimmte Einstellung. Der Regler ersetzt die bisherige lineare Kennlinie.
Bei der Inbetriebnahme zuerst mit KI=KD=0 den P-Anteil abstimmen, dann I
und bei Bedarf D schrittweise ergaenzen. Ein dauerhaft erreichbarer Sollwert
und die benoetigte Halteleistung haengen vom realen Geblaeseaufbau ab.

Regelschritte erfolgen fruehestens alle 50 ms und nur mit neuer Messung.
Die tatsaechlich verstrichene Zeit wird als TA in Sekunden verwendet.
ISUM speichert im Unterschied zur Vorlage bereits das zeitgewichtete Integral.
Der D-Anteil wird mit einer Zeitkonstante von 0.1 s gefiltert.
Ausgangsbegrenzung 0..100 Prozent, bedingte Integration gegen Windup.
Bei Sollwertwechsel wird der Regler zurueckgesetzt (auch der I-Anteil).
CAN und UART melden den auf ganze Prozent gerundeten PWM-Wert.

Bei Sensorfehler, Reichweitenfehler, Messwertalter ab 250 ms oder einer
Regelpause ueber 500 ms: PWM=0 und Reglerreset. Blockierende Buszugriffe
koennen die Reaktionszeit verlaengern; dies ist keine Hardwareabschaltung.
DAVE ist weiterhin auf 50 Prozent Start-PWM konfiguriert. main setzt direkt
nach erfolgreichem DAVE_Init 0 Prozent. Ein kurzer Startimpuls ist moeglich;
fuer impulsfreien Start in der DAVE PWM_0-APP den Anfangstastgrad auf 0 setzen
und den Code neu generieren. Bei fehlgeschlagenem DAVE_Init bleibt der
bestehende Fehlerhalt aktiv; der Ausgangszustand ist dann nicht garantiert.

Softwaretests pruefen PID-Rechnung, Zeitbezug, Saettigung/Anti-Windup,
D-Filter, Reset und ungueltige Eingaben mit dem kompilierten ARM-Code.
Ein Test am Board und die Abstimmung der Regelstrecke stehen aus.

UART kann ausserdem KP/KI/KD mit Aufloesung 0,01 gemeinsam mit Gruppe und
Sollwert setzen. Gruppe ist nur Kennzeichnung. Siehe UART_Spezifikation.md.
