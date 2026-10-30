> Nachfolgende Laufzeitoptimierung: [PID_Optimierung.md](PID_Optimierung.md).
> ISUM speichert im optimierten Code direkt den gewichteten I-Anteil in Prozent.

# Pruefung des I-Anteils, 30.10.2026

## Befund

Die PID-Grundrechnung ist zeitlich korrekt: ISUM += Fehler * TA;
I = KI * ISUM. TA wird in main aus dem tatsaechlichen Regelintervall in Sekunden
gebildet. KP/KI/KD aus UART werden als Hundertstel nach float umgerechnet.
Ein int16-Rohwert 10 bedeutet KI=0.10 Prozent/(mm*s).

Der Fehler lag in der Einbindung: TOF_UARTApply loeschte bei neuer Sollhoehe
das komplette Reglergedaechtnis; TOF_CAN_ApplySetpoint tat dasselbe. Zusaetzlich
loeschte der Regelzweig beim erkannten Sollwertwechsel nochmals den Zustand.
Dadurch verlor ein laufendes Sollwertprofil staendig seinen I-Anteil.
Diese Reset-Behandlung war Teil der bisherigen Implementierung.

Bei KP=KI=0.1, KD=0 und Ist etwa 80 mm / Soll etwa 210 mm:
Fehler 130 mm, P=13 Prozent, I-Zuwachs je 50 ms = 0.1*130*0.05=0.65 Prozent.
Ohne Reset ergibt das nach 5 s I=65 Prozent und zusammen PWM=78 Prozent.
Ein Reset bei jedem Regelschritt begrenzt den Wert dagegen auf etwa
13.65 Prozent. Laengere konstante Sollwertabschnitte erzeugen Rampen;
der naechste Sollwertwechsel liess PWM bisher wieder auf den P-Anteil fallen.
Dieses Verhalten passt zu den wiederholten Rampen und Abfaellen im Screenshot.
Die Abbildung allein belegt nicht die Ursache der anfaenglichen Nullphase
oder spaeterer Nullwerte; Sensor-/Zeitfehler und PC-Datenluecken sind daraus
nicht eindeutig unterscheidbar.

## Korrektur

PIDresetDerivative setzt nur D-Gedaechtnis und Erstschrittkennzeichen zurueck.
Das Integral bleibt bei reinen UART-/CAN-Sollwertwechseln erhalten.
Der vorhandene Anti-Windup und die 0..100-Prozent-Begrenzung bleiben aktiv.
Vollstaendiger PIDclear weiterhin bei Verstaerkungs-, Gruppen- und Quellenwechsel,
ungueltigen Messungen, fehlender CAN-Anfangsvorgabe und Zeitueberschreitungen.
Es wird keine Integration nachgeholt, solange die Regelung abgeschaltet ist.

## Softwarepruefung

Tests/pid_test.c prueft den 78-Prozent-Fall mit 100 Schritten zu 50 ms sowie
D-Impulsunterdrueckung bei erhaltenem I-Anteil. Tests/can_group_test.c fuehrt
wechselnde UART-Sollwerte durch die echten main-Helfer und prueft den
Integratorerhalt bei CAN-Sollwertwechseln sowie den Reset bei Gruppenwechsel.
Die bestehenden Tests fuer P/I/D, variable Zeit, Anti-Windup und ungueltige
Eingaben laufen weiterhin. Ausfuehrung als kompilierter ARM-Code im Emulator.
Der Firmware-Build ist erfolgreich. Noch kein Flashen oder Hardwareversuch.

Zum Hardwarevergleich mit festen Parametern erst einen konstanten Sollwert,
danach das bisherige Profil verwenden und height_pid.ISUM sowie die tatsaechliche
TA im Debugger beobachten. Die korrigierte Firmware muss dafuer neu geflasht werden.
