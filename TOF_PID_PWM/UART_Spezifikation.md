# UART-Schnittstellenspezifikation TOF_PID_PWM

Stand: 29.10.2026. Dokumentrevision: 2.0.
Diese Revision bezeichnet das Dokument; auf der Leitung wird keine Protokollversion übertragen.

## 1. Zweck und Geltungsbereich

Diese Spezifikation beschreibt den aktuellen implementierten Datenaustausch
zwischen dem Projekt TOF_PID_PWM auf dem KIT_XMC14_2Go und einem PC über den
USB-COM-Port. Sie basiert auf main.c, Libraries/UART/UARTlib.c/.h,
Libraries/UART/UARTprotocol.c/.h und Tools/uart_monitor.py.

Das Board sendet Höhe, Höhensollwert und PWM-Stellwert zyklisch. Der PC kann
den Höhensollwert ändern. Die beiden Übertragungsrichtungen verwenden
unterschiedliche Paketformate und können gleichzeitig betrieben werden.

## 2. Verbindung und UART-Konfiguration

| Eigenschaft | Aktuelle Einstellung |
| --- | --- |
| PC-Anschluss | Virtueller J-Link-COM-Port über die eingebaute USB-Brücke |
| COM-Nummer | Vom Betriebssystem vergeben; kein fester Protokollparameter |
| Controllerkanal | USIC0_CH0 |
| TX aus Sicht des XMC1404 | P2.1, ALT6, zur RX-Seite der USB-Brücke |
| RX aus Sicht des XMC1404 | P2.2, Eingangsrouting DX3A nach DX0G |
| Angeforderte Baudrate | 128000 Baud, UARTLIB_BAUDRATE |
| Zeichenformat | 8 Datenbits, keine Parität, 1 Stopbit (8N1) |
| Bitreihenfolge innerhalb eines UART-Zeichens | Niederwertigstes Bit zuerst |
| Flusssteuerung | Keine; weder RTS/CTS noch XON/XOFF |
| Zeichenkodierung | Binärdaten; keine ASCII-Zahlen und keine Zeilenenden |
| Mehrbytezahlen | Little Endian: Low-Byte vor High-Byte |

Beide Seiten müssen dieselbe Baudrate verwenden. UARTLIB_ActualBaudrate
enthält die durch XMCLib berechnete Generatorrate. Die voreingestellten
128000 Baud beruhen auf einer Windows-Treiberangabe für den J-Link-Port;
ein fehlerfreier Betrieb dieser Rate an der USB-Brücke ist noch nicht am
Board nachgewiesen. Eine Umstellung, etwa auf 115200 Baud, muss in Firmware
und PC-Programm gemeinsam erfolgen.

## 3. Board → PC: zyklische Telemetrie

### 3.1 Paketaufbau

Jedes Paket besteht aus genau **8 Bytes**:

```text
55 AA ch1_L ch1_H ch2_L ch2_H ch3_L ch3_H
```

Offsets beginnen in dieser Spezifikation bei 0.

| Offset | Länge | Feld | Typ / Bedeutung |
| --- | --- | --- | --- |
| 0 | 1 Byte | Sync 1 | Konstant 0x55 |
| 1 | 1 Byte | Sync 2 | Konstant 0xAA |
| 2–3 | 2 Bytes | ch1 | Höhe, int16_t, mm, Little Endian |
| 4–5 | 2 Bytes | ch2 | Höhensollwert, int16_t, mm, Little Endian |
| 6–7 | 2 Bytes | ch3 | PWM-Stellwert, int16_t, ganze Prozent, Little Endian |

Alle drei Kanäle verwenden vorzeichenbehaftete 16-Bit-Zahlen im
Zweierkomplement. Es gibt kein Padding, kein Längenfeld, keinen
Botschaftszähler und keine CRC im Telemetriepaket.

### 3.2 Semantik

- **ch1:** gleitender Mittelwert aus bis zu vier gültigen TOF-Messungen,
  auf ganze Millimeter gerundet. Die Höhe entspricht dem Sensorabstand;
  eine geometrische Umrechnung erfolgt nicht. Bei Sensorfehler,
  Reichweitenersatzwert 8190 oder einem Wert oberhalb INT16_MAX wird
  **−1**, Bytefolge `FF FF`, gesendet.
- **ch2:** wirksamer PID-Hoehensollwert TOF_Setpoint_mm aus UART oder CAN,
  Bereich **0 bis 300 mm**, Startwert nach Reset **300 mm**.
- **ch3:** zuletzt von der Anwendung gesetzter PWM-Stellwert,
  Bereich **0 bis 100 %**, auf ganze Prozent gerundet. Dies ist keine
  Messung der Drehzahl oder des elektrischen PWM-Ausgangs.

Die Telemetrie prüft das Messwertalter nicht separat. Bei einer veralteten,
ansonsten gültig markierten Messung kann ch1 deshalb den letzten Mittelwert
enthalten, während die Regelung die PWM bereits auf 0 gesetzt hat. Das
Paket enthält kein Alter und kein eigenes Gültigkeitsbit für diesen Fall.

### 3.3 Zeitverhalten

UARTLIB_TELEMETRY_MS beträgt **50 ms**, nominal also **20 Pakete/s**.
Der Versand beginnt nach Abschluss der Sensorinitialisierung, auch wenn
diese fehlgeschlagen ist, sofern UART erfolgreich initialisiert wurde.
Vor einem fälligen Versand werden Messung und gegebenenfalls PID/PWM bearbeitet.
Der PID-Regler arbeitet frühestens alle 50 ms; mehrere Telemetriepakete
können denselben Stellwert enthalten.

Das 50-ms-Intervall ist ein Mindestabstand der Sendeversuche in der
Hauptschleife. OLED-/I2C-Zugriffe und USB-Pufferung können längere Abstände
und gebündelte PC-Leseergebnisse verursachen. Verpasste Intervalle werden
nicht nachgeholt. Es gibt keine garantierte maximale Ende-zu-Ende-Latenz.

Bei 128000 Baud und 8N1 benötigt ein Zeichen 10 Bitzeiten. Die reine
Leitungsdauer eines 8-Byte-Pakets beträgt rechnerisch **0,625 ms**.
Nominal entstehen 160 Nutzbytes/s einschließlich der beiden Sync-Bytes,
entsprechend 1600 Bit/s auf der UART-Leitung.

### 3.4 Empfang und Synchronisation auf dem PC

1. Im Bytestrom nach `55 AA` suchen.
2. Ab diesem Header insgesamt acht Bytes sammeln; USB-Lesegrenzen sind
   keine Paketgrenzen.
3. Die sechs Nutzbytes als drei Little-Endian-int16 dekodieren.
4. Anschließend das nächste Paket suchen. Ein `55 AA` innerhalb eines
   bereits ausgerichteten Nutzdatenbereichs ist gewöhnlicher Dateninhalt.

Eine Sendepause ist nicht erforderlich. Ein abschließendes einzelnes `55`
kann der Beginn eines noch unvollständigen Headers sein und muss beim
Zusammenfügen von Lesezugriffen erhalten bleiben.

Da keine CRC und kein Byte-Escaping vorhanden sind, können beschädigte
Pakete und eine falsche Synchronisation nach Byteverlust nicht sicher
erkannt werden. Bereichsprüfungen sind Plausibilitätsprüfungen und ersetzen
keine Integritätsprüfung. Ein PC-Empfangstimeout ist nicht vorgegeben.

### 3.5 Beispiele

| Werte | Paket, hexadezimal |
| --- | --- |
| Höhe 150 mm, Sollwert 300 mm, PWM 50 % | `55 AA 96 00 2C 01 32 00` |
| Höhe ungültig, Sollwert 150 mm, PWM 0 % | `55 AA FF FF 96 00 00 00` |

## 4. PC → Board: Gruppe, Sollwert und PID-Parameter setzen

### 4.1 Paketaufbau

Ein Befehl besteht aus genau **13 Bytes**. Alle Mehrbytefelder sind Little Endian.

```text
A5 5A Gruppe Soll_L Soll_H KP_L KP_H KI_L KI_H KD_L KD_H CRC_L CRC_H
```

| Offset | Länge | Feld | Bedeutung |
| --- | --- | --- | --- |
| 0–1 | 2 Bytes | Header | Konstant 0xA5 0x5A |
| 2 | 1 Byte | Gruppe | uint8_t, 1..32 |
| 3–4 | 2 Bytes | Sollwert | int16_t, 0..300 mm |
| 5–6 | 2 Bytes | KP | int16_t, Auflösung 0,01 |
| 7–8 | 2 Bytes | KI | int16_t, Auflösung 0,01 |
| 9–10 | 2 Bytes | KD | int16_t, Auflösung 0,01 |
| 11–12 | 2 Bytes | CRC | uint16_t, CRC über Offset 0..10 |

Die Gruppennummer wird in UART_Group gespeichert (Startwert 0 bis zum ersten gültigen Befehl). Sie kennzeichnet
den zuletzt angenommenen Parametersatz. Es gibt keinen UART-Adressfilter und keinen
Speicher mit 32 Parametersätzen: Jede gültige Gruppennummer wird angenommen.

Für jeden Verstärkungsfaktor gilt **Reglerwert = int16-Rohwert / 100**.
Beispiele: 33 entspricht 0,33; 10 entspricht 0,10; 2 entspricht 0,02;
−25 entspricht −0,25. Der vollständige signed Bereich −32768..32767 ist
zulässig, entsprechend −327,68..327,67. Negative Verstärkungen sind daher
technisch übertragbar; ihre Wirkung ist vom Vorzeichen der Regelstrecke abhängig.
KP hat die Einheit Prozent/mm, KI Prozent/(mm*s), KD Prozent*s/mm.
KI und KD werden nicht im PC auf die 50-ms-Reglerzeit umgerechnet; dies erledigt
der PID mit der tatsächlich verstrichenen Zeit in Sekunden.

### 4.2 CRC

CRC-16/CCITT-FALSE: Breite 16 Bit, Polynom 0x1021, Startwert 0xFFFF,
RefIn=false, RefOut=false, XorOut=0x0000. Geschützt sind die ersten **11 Bytes**,
einschließlich Header und Gruppe. Übertragen wird CRC_L vor CRC_H.
Referenz: ASCII `123456789` ergibt 0x29B1.

### 4.3 Übernahme und Wirkung

Gruppe, Sollwert und alle drei Verstärkungen werden ausschließlich bei gültigem
Header, gültiger CRC, Gruppe 1..32 und Sollwert 0..300 gemeinsam übernommen.
Bei einem ungültigen Befehl bleiben alle bisherigen Einstellungen erhalten.
Ein UART-Sollwert von 0 waehlt CAN als Sollwertquelle; 1..300 mm setzen den Sollwert direkt.

Die Uebernahme erfolgt in main vor dem naechsten Regelschritt.
Aenderungen an KP/KI/KD sowie Gruppen- oder Quellenwechsel loeschen Integral-
und D-Gedaechtnis. Reine Sollwertaenderungen behalten das Integral und setzen
nur das D-Gedaechtnis zur Vermeidung eines Sprungimpulses zurueck.
Ein identischer Parametersatz loest keinen Reset aus.
Die Parameter gelten fuer den einen laufenden Hoehenregler.

Alle Einstellungen sind flüchtig. Nach Reset gelten Gruppe 0 (noch kein Empfang), Sollwert 300 mm
und die in main.c definierten HEIGHT_PID_KP/KI/KD-Startwerte. Diese Startwerte
sind float-Konstanten und müssen nicht exakt auf dem 0,01-Übertragungsraster liegen.

Es gibt keine separate ACK-/NACK-Nachricht. Nur der Sollwert erscheint weiterhin
in ch2 der Telemetrie (im CAN-Betrieb der wirksame CAN-Sollwert). Gruppe und KP/KI/KD werden nicht zurückgesendet. Die
Telemetrie bestätigt daher nicht eindeutig die Annahme des gesamten Parametersatzes.
Ein gültiges identisches Telegramm wird erneut gezählt (uart_setpoint_count).
Kommunikationsverlust löscht die Einstellungen nicht.

### 4.4 Fehler, Zeitlimit und Puffer

Der Parser verwendet ein gleitendes 13-Byte-Fenster. Ungültige Fenster werden
byteweise verschoben; nach gültigem Befehl wird das Fenster geleert. Header,
CRC und Gruppe können auch in Nutzdaten auftreten und sind dort normale Bytes.
Alte 6-Byte-Befehle bilden keinen vollständigen gültigen Parameterbefehl.

Nach mindestens 200 ms ohne in main verarbeitete Bytes wird ein Teilpaket
verworfen. Dies ist kein genauer Timeout zwischen physischen Empfangszeitpunkten.
Bei UART-/FIFO-/Ringpufferfehlern werden Restpuffer und Parser verworfen.
Es wird keine Fehlerantwort gesendet.

RX-FIFO: 16 Plätze; RAM-Ringpuffer: 128 Stellen, davon 127 gleichzeitig nutzbar.
Pro main-Aufruf werden maximal 128 Bytes abgeholt. UARTLIB_RxErrors zählt
Empfangsfehlerereignisse, keine CRC-Fehler. Eine maximale Befehlsrate oder
garantierte Antwortzeit ist nicht festgelegt. PC-Anwendungen sollen vollständige
Parametersätze einzeln senden. Bei 128000 Baud und 8N1 dauert ein 13-Byte-Befehl
auf der UART-Leitung rechnerisch 1,015625 ms, ohne USB-Latenz.

### 4.5 Referenztelegramme

Gruppe 1, Sollwert 150 mm, KP=0,33, KI=0,10, KD=0,02:

```text
A5 5A 01 96 00 21 00 0A 00 02 00 20 D9
```

CRC-Zahlenwert: 0xD920. Gruppe 32, Sollwert 300 mm,
KP=327,67, KI=−327,68, KD=0 (Format-Grenzwerttest):

```text
A5 5A 20 2C 01 FF 7F 00 80 00 00 44 6E
```

CRC-Zahlenwert: 0x6E44. Beide Beispiele werden durch Firmware- und PC-Tests geprüft.

## 5. Senderessourcen und Diagnose

Das Board übergibt ein Telemetriepaket nur, wenn im 16-Wort-TX-FIFO mindestens
acht Plätze frei sind. Bei Platzmangel wird das komplette Paket ausgelassen;
es wird kein Teilpaket geschrieben. uart_tx_skipped zählt diese Fälle.
Eine Wiederholung dieses Pakets erfolgt nicht; der nächste Sendeversuch
verwendet die dann aktuellen Werte. Eine erfolgreiche FIFO-Übergabe ist
keine Bestätigung der Zustellung an die PC-Anwendung.

| Debuggervariable | Bedeutung |
| --- | --- |
| UART_Group | Zuletzt angenommene Gruppe, 1..32 |
| height_pid.KP/KI/KD | Aktive PID-Verstärkungen als float |
| TOF_Setpoint_mm | Aktuell gespeicherter Höhensollwert |
| uart_setpoint_count | Anzahl angenommener Parameterbefehle |
| UARTLIB_RxErrors | Erkannte Empfangsfehlerereignisse |
| uart_tx_skipped | Nicht übergebene Telemetriepakete |
| UARTLIB_ActualBaudrate | Berechnete Baudrate des UART-Generators |

Diese Diagnosewerte werden nicht als zusätzliche Felder über UART übertragen.
Der Kommunikationsverlust allein löscht den Sollwert nicht und schaltet den
Regler nicht ab. Die messwertabhängige PWM-Abschaltung ist in README_PID.md
beschrieben.

## 6. Referenzprogramm und Prüfung

Tools/uart_monitor.py verwendet Python und pyserial. Beispiel:

```text
python Tools/uart_monitor.py COM8 --baud 128000 --group 1 --setpoint 150 --kp 0.33 --ki 0.10 --kd 0.02
```

COM8 ist durch den tatsächlichen Port zu ersetzen. Das Programm sendet optional
einen Parameterbefehl und dekodiert die Telemetrie. Seine CSV-Ausgabe
`Hoehe_mm,Sollwert_mm,PWM_Prozent` entsteht erst auf dem PC und ist nicht das
UART-Leitungsformat. Das Hilfsprogramm implementiert keine automatische
Befehlswiederholung und keine explizite Quittierungsprüfung.

Alle fünf Sendeoptionen sind gemeinsam erforderlich; ohne diese Optionen wird
nur empfangen. KP/KI/KD werden auf der Kommandozeile als Dezimalwerte angegeben,
nicht als int16-Rohwerte. Mehr als zwei relevante Nachkommastellen werden abgewiesen.

Die Beispielbefehle wurden mit dem bestehenden PC-Encoder berechnet und
gegen UARTPROTO_Crc geprüft. Tests/validate_uart.py prüft das Firmwareformat,
CRC, Wertebereiche und Parserfälle mit kompiliertem ARM-Code.
Tests/test_uart_monitor.py prüft die PC-Telemetriedekodierung.
Diese Softwareprüfung ersetzt keinen Test von USB-Brücke und Board.

## 7. Anzeige der Gruppe

Die feste, 16 Pixel hohe Titelzeile lautet `TOF_PID_PWM 00` nach Reset.
Nach einem gueltigen Parametertelegramm wird 01..32 zweistellig angezeigt,
auch in der Oszilloskopansicht. Ungueltige Telegramme aendern die Anzeige nicht.
Der Plot liegt darunter auf OLED-Seiten 2..6 (128 x 40 Pixel).
Die Fusszeile auf Seite 7 zeigt die Zeitachsenbeschreibung.

## 8. CAN-Sollwertauswahl

Seit dieser Erweiterung waehlt Sollwert 0 im UART-Befehl den CAN-Empfang
auf ID 0x100 fuer die eigene Gruppe. UART_Setpoint_mm speichert diese
Auswahl dauerhaft bis zum naechsten UART-Befehl oder Reset. ch2 meldet den
wirksamen CAN-Sollwert, nicht die UART-Null. Bis zum ersten passenden Frame
ist dieser Wert 0 und die PWM bleibt aus. Gruppenbezogene CAN-Sende-IDs
und Empfangsformat sind in [CAN_Spezifikation.md](CAN_Spezifikation.md) beschrieben.

Hinweis zur Reglerkorrektur vom 30.10.2026: Reine Sollwertaenderungen
ueber UART oder CAN loeschen den I-Anteil nicht mehr. Ein vollstaendiger Reset
erfolgt weiterhin bei Gruppen-/Quellen-/Verstaerkungsaenderungen sowie
Messfehlern und den vorhandenen Zeitueberschreitungen.
