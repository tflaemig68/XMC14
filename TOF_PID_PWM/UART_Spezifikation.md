# UART-Schnittstellenspezifikation TOF_PID_PWM

Stand: 29.10.2026. Dokumentrevision: 2.0.
Diese Revision bezeichnet das Dokument; auf der Leitung wird keine Protokollversion ÃƒÂ¼bertragen.

## 1. Zweck und Geltungsbereich

Diese Spezifikation beschreibt den aktuellen implementierten Datenaustausch
zwischen dem Projekt TOF_PID_PWM auf dem KIT_XMC14_2Go und einem PC ÃƒÂ¼ber den
USB-COM-Port. Sie basiert auf main.c, Libraries/UART/UARTlib.c/.h,
Libraries/UART/UARTprotocol.c/.h und Tools/uart_monitor.py.

Das Board sendet HÃƒÂ¶he, HÃƒÂ¶hensollwert und PWM-Stellwert zyklisch. Der PC kann
den HÃƒÂ¶hensollwert ÃƒÂ¤ndern. Die beiden ÃƒÅ“bertragungsrichtungen verwenden
unterschiedliche Paketformate und kÃƒÂ¶nnen gleichzeitig betrieben werden.

## 2. Verbindung und UART-Konfiguration

| Eigenschaft | Aktuelle Einstellung |
| --- | --- |
| PC-Anschluss | Virtueller J-Link-COM-Port ÃƒÂ¼ber die eingebaute USB-BrÃƒÂ¼cke |
| COM-Nummer | Vom Betriebssystem vergeben; kein fester Protokollparameter |
| Controllerkanal | USIC0_CH0 |
| TX aus Sicht des XMC1404 | P2.1, ALT6, zur RX-Seite der USB-BrÃƒÂ¼cke |
| RX aus Sicht des XMC1404 | P2.2, Eingangsrouting DX3A nach DX0G |
| Angeforderte Baudrate | 128000 Baud, UARTLIB_BAUDRATE |
| Zeichenformat | 8 Datenbits, keine ParitÃƒÂ¤t, 1 Stopbit (8N1) |
| Bitreihenfolge innerhalb eines UART-Zeichens | Niederwertigstes Bit zuerst |
| Flusssteuerung | Keine; weder RTS/CTS noch XON/XOFF |
| Zeichenkodierung | BinÃƒÂ¤rdaten; keine ASCII-Zahlen und keine Zeilenenden |
| Mehrbytezahlen | Little Endian: Low-Byte vor High-Byte |

Beide Seiten mÃƒÂ¼ssen dieselbe Baudrate verwenden. UARTLIB_ActualBaudrate
enthÃƒÂ¤lt die durch XMCLib berechnete Generatorrate. Die voreingestellten
128000 Baud beruhen auf einer Windows-Treiberangabe fÃƒÂ¼r den J-Link-Port;
ein fehlerfreier Betrieb dieser Rate an der USB-BrÃƒÂ¼cke ist noch nicht am
Board nachgewiesen. Eine Umstellung, etwa auf 115200 Baud, muss in Firmware
und PC-Programm gemeinsam erfolgen.

## 3. Board Ã¢â€ â€™ PC: zyklische Telemetrie

### 3.1 Paketaufbau

Jedes Paket besteht aus genau **8 Bytes**:

```text
55 AA ch1_L ch1_H ch2_L ch2_H ch3_L ch3_H
```

Offsets beginnen in dieser Spezifikation bei 0.

| Offset | LÃƒÂ¤nge | Feld | Typ / Bedeutung |
| --- | --- | --- | --- |
| 0 | 1 Byte | Sync 1 | Konstant 0x55 |
| 1 | 1 Byte | Sync 2 | Konstant 0xAA |
| 2Ã¢â‚¬â€œ3 | 2 Bytes | ch1 | HÃƒÂ¶he, int16_t, mm, Little Endian |
| 4Ã¢â‚¬â€œ5 | 2 Bytes | ch2 | HÃƒÂ¶hensollwert, int16_t, mm, Little Endian |
| 6Ã¢â‚¬â€œ7 | 2 Bytes | ch3 | PWM-Stellwert, int16_t, ganze Prozent, Little Endian |

Alle drei KanÃƒÂ¤le verwenden vorzeichenbehaftete 16-Bit-Zahlen im
Zweierkomplement. Es gibt kein Padding, kein LÃƒÂ¤ngenfeld, keinen
BotschaftszÃƒÂ¤hler und keine CRC im Telemetriepaket.

### 3.2 Semantik

- **ch1:** gleitender Mittelwert aus bis zu vier gÃƒÂ¼ltigen TOF-Messungen,
  auf ganze Millimeter gerundet. Die HÃƒÂ¶he entspricht dem Sensorabstand;
  eine geometrische Umrechnung erfolgt nicht. Bei Sensorfehler,
  Reichweitenersatzwert 8190 oder einem Wert oberhalb INT16_MAX wird
  **Ã¢Ë†â€™1**, Bytefolge `FF FF`, gesendet.
- **ch2:** aktuell gespeicherter PID-HÃƒÂ¶hensollwert TOF_Setpoint_mm,
  Bereich **0 bis 300 mm**, Startwert nach Reset **300 mm**.
- **ch3:** zuletzt von der Anwendung gesetzter PWM-Stellwert,
  Bereich **0 bis 100 %**, auf ganze Prozent gerundet. Dies ist keine
  Messung der Drehzahl oder des elektrischen PWM-Ausgangs.

Die Telemetrie prÃƒÂ¼ft das Messwertalter nicht separat. Bei einer veralteten,
ansonsten gÃƒÂ¼ltig markierten Messung kann ch1 deshalb den letzten Mittelwert
enthalten, wÃƒÂ¤hrend die Regelung die PWM bereits auf 0 gesetzt hat. Das
Paket enthÃƒÂ¤lt kein Alter und kein eigenes GÃƒÂ¼ltigkeitsbit fÃƒÂ¼r diesen Fall.

### 3.3 Zeitverhalten

UARTLIB_TELEMETRY_MS betrÃƒÂ¤gt **50 ms**, nominal also **20 Pakete/s**.
Der Versand beginnt nach Abschluss der Sensorinitialisierung, auch wenn
diese fehlgeschlagen ist, sofern UART erfolgreich initialisiert wurde.
Vor einem fÃƒÂ¤lligen Versand werden Messung und gegebenenfalls PID/PWM bearbeitet.
Der PID-Regler arbeitet frÃƒÂ¼hestens alle 50 ms; mehrere Telemetriepakete
kÃƒÂ¶nnen denselben Stellwert enthalten.

Das 50-ms-Intervall ist ein Mindestabstand der Sendeversuche in der
Hauptschleife. OLED-/I2C-Zugriffe und USB-Pufferung kÃƒÂ¶nnen lÃƒÂ¤ngere AbstÃƒÂ¤nde
und gebÃƒÂ¼ndelte PC-Leseergebnisse verursachen. Verpasste Intervalle werden
nicht nachgeholt. Es gibt keine garantierte maximale Ende-zu-Ende-Latenz.

Bei 128000 Baud und 8N1 benÃƒÂ¶tigt ein Zeichen 10 Bitzeiten. Die reine
Leitungsdauer eines 8-Byte-Pakets betrÃƒÂ¤gt rechnerisch **0,625 ms**.
Nominal entstehen 160 Nutzbytes/s einschlieÃƒÅ¸lich der beiden Sync-Bytes,
entsprechend 1600 Bit/s auf der UART-Leitung.

### 3.4 Empfang und Synchronisation auf dem PC

1. Im Bytestrom nach `55 AA` suchen.
2. Ab diesem Header insgesamt acht Bytes sammeln; USB-Lesegrenzen sind
   keine Paketgrenzen.
3. Die sechs Nutzbytes als drei Little-Endian-int16 dekodieren.
4. AnschlieÃƒÅ¸end das nÃƒÂ¤chste Paket suchen. Ein `55 AA` innerhalb eines
   bereits ausgerichteten Nutzdatenbereichs ist gewÃƒÂ¶hnlicher Dateninhalt.

Eine Sendepause ist nicht erforderlich. Ein abschlieÃƒÅ¸endes einzelnes `55`
kann der Beginn eines noch unvollstÃƒÂ¤ndigen Headers sein und muss beim
ZusammenfÃƒÂ¼gen von Lesezugriffen erhalten bleiben.

Da keine CRC und kein Byte-Escaping vorhanden sind, kÃƒÂ¶nnen beschÃƒÂ¤digte
Pakete und eine falsche Synchronisation nach Byteverlust nicht sicher
erkannt werden. BereichsprÃƒÂ¼fungen sind PlausibilitÃƒÂ¤tsprÃƒÂ¼fungen und ersetzen
keine IntegritÃƒÂ¤tsprÃƒÂ¼fung. Ein PC-Empfangstimeout ist nicht vorgegeben.

### 3.5 Beispiele

| Werte | Paket, hexadezimal |
| --- | --- |
| HÃƒÂ¶he 150 mm, Sollwert 300 mm, PWM 50 % | `55 AA 96 00 2C 01 32 00` |
| HÃƒÂ¶he ungÃƒÂ¼ltig, Sollwert 150 mm, PWM 0 % | `55 AA FF FF 96 00 00 00` |

## 4. PC â†’ Board: Gruppe, Sollwert und PID-Parameter setzen

### 4.1 Paketaufbau

Ein Befehl besteht aus genau **13 Bytes**. Alle Mehrbytefelder sind Little Endian.

```text
A5 5A Gruppe Soll_L Soll_H KP_L KP_H KI_L KI_H KD_L KD_H CRC_L CRC_H
```

| Offset | LÃ¤nge | Feld | Bedeutung |
| --- | --- | --- | --- |
| 0â€“1 | 2 Bytes | Header | Konstant 0xA5 0x5A |
| 2 | 1 Byte | Gruppe | uint8_t, 1..32 |
| 3â€“4 | 2 Bytes | Sollwert | int16_t, 0..300 mm |
| 5â€“6 | 2 Bytes | KP | int16_t, AuflÃ¶sung 0,01 |
| 7â€“8 | 2 Bytes | KI | int16_t, AuflÃ¶sung 0,01 |
| 9â€“10 | 2 Bytes | KD | int16_t, AuflÃ¶sung 0,01 |
| 11â€“12 | 2 Bytes | CRC | uint16_t, CRC Ã¼ber Offset 0..10 |

Die Gruppennummer wird in UART_Group gespeichert (Startwert 0 bis zum ersten gültigen Befehl). Sie kennzeichnet
den zuletzt angenommenen Parametersatz. Es gibt keinen Adressfilter und keinen
Speicher mit 32 ParametersÃ¤tzen: Jede gÃ¼ltige Gruppennummer wird angenommen.

FÃ¼r jeden VerstÃ¤rkungsfaktor gilt **Reglerwert = int16-Rohwert / 100**.
Beispiele: 33 entspricht 0,33; 10 entspricht 0,10; 2 entspricht 0,02;
âˆ’25 entspricht âˆ’0,25. Der vollstÃ¤ndige signed Bereich âˆ’32768..32767 ist
zulÃ¤ssig, entsprechend âˆ’327,68..327,67. Negative VerstÃ¤rkungen sind daher
technisch Ã¼bertragbar; ihre Wirkung ist vom Vorzeichen der Regelstrecke abhÃ¤ngig.
KP hat die Einheit Prozent/mm, KI Prozent/(mm*s), KD Prozent*s/mm.
KI und KD werden nicht im PC auf die 50-ms-Reglerzeit umgerechnet; dies erledigt
der PID mit der tatsÃ¤chlich verstrichenen Zeit in Sekunden.

### 4.2 CRC

CRC-16/CCITT-FALSE: Breite 16 Bit, Polynom 0x1021, Startwert 0xFFFF,
RefIn=false, RefOut=false, XorOut=0x0000. GeschÃ¼tzt sind die ersten **11 Bytes**,
einschlieÃŸlich Header und Gruppe. Ãœbertragen wird CRC_L vor CRC_H.
Referenz: ASCII `123456789` ergibt 0x29B1.

### 4.3 Ãœbernahme und Wirkung

Gruppe, Sollwert und alle drei VerstÃ¤rkungen werden ausschlieÃŸlich bei gÃ¼ltigem
Header, gÃ¼ltiger CRC, Gruppe 1..32 und Sollwert 0..300 gemeinsam Ã¼bernommen.
Bei einem ungÃ¼ltigen Befehl bleiben alle bisherigen Einstellungen erhalten.
Ein Sollwert von 0 ist ein regulÃ¤rer Sollwert, kein eigener Abschaltbefehl.

Die Ãœbernahme erfolgt in main vor dem nÃ¤chsten Regelschritt. Ã„nderungen an
Sollwert, KP, KI oder KD lÃ¶schen Integral- und D-GedÃ¤chtnis. Ein alleiniger
Gruppenwechsel oder ein identischer Parametersatz lÃ¶st diesen Reset nicht aus.
Die Parameter gelten fÃ¼r den einen laufenden HÃ¶henregler.

Alle Einstellungen sind flÃ¼chtig. Nach Reset gelten Gruppe 0 (noch kein Empfang), Sollwert 300 mm
und die in main.c definierten HEIGHT_PID_KP/KI/KD-Startwerte. Diese Startwerte
sind float-Konstanten und mÃ¼ssen nicht exakt auf dem 0,01-Ãœbertragungsraster liegen.

Es gibt keine separate ACK-/NACK-Nachricht. Nur der Sollwert erscheint weiterhin
in ch2 der Telemetrie. Gruppe und KP/KI/KD werden nicht zurÃ¼ckgesendet. Die
Telemetrie bestÃ¤tigt daher nicht eindeutig die Annahme des gesamten Parametersatzes.
Ein gÃ¼ltiges identisches Telegramm wird erneut gezÃ¤hlt (uart_setpoint_count).
Kommunikationsverlust lÃ¶scht die Einstellungen nicht.

### 4.4 Fehler, Zeitlimit und Puffer

Der Parser verwendet ein gleitendes 13-Byte-Fenster. UngÃ¼ltige Fenster werden
byteweise verschoben; nach gÃ¼ltigem Befehl wird das Fenster geleert. Header,
CRC und Gruppe kÃ¶nnen auch in Nutzdaten auftreten und sind dort normale Bytes.
Alte 6-Byte-Befehle bilden keinen vollstÃ¤ndigen gÃ¼ltigen Parameterbefehl.

Nach mindestens 200 ms ohne in main verarbeitete Bytes wird ein Teilpaket
verworfen. Dies ist kein genauer Timeout zwischen physischen Empfangszeitpunkten.
Bei UART-/FIFO-/Ringpufferfehlern werden Restpuffer und Parser verworfen.
Es wird keine Fehlerantwort gesendet.

RX-FIFO: 16 PlÃ¤tze; RAM-Ringpuffer: 128 Stellen, davon 127 gleichzeitig nutzbar.
Pro main-Aufruf werden maximal 128 Bytes abgeholt. UARTLIB_RxErrors zÃ¤hlt
Empfangsfehlerereignisse, keine CRC-Fehler. Eine maximale Befehlsrate oder
garantierte Antwortzeit ist nicht festgelegt. PC-Anwendungen sollen vollstÃ¤ndige
ParametersÃ¤tze einzeln senden. Bei 128000 Baud und 8N1 dauert ein 13-Byte-Befehl
auf der UART-Leitung rechnerisch 1,015625 ms, ohne USB-Latenz.

### 4.5 Referenztelegramme

Gruppe 1, Sollwert 150 mm, KP=0,33, KI=0,10, KD=0,02:

```text
A5 5A 01 96 00 21 00 0A 00 02 00 20 D9
```

CRC-Zahlenwert: 0xD920. Gruppe 32, Sollwert 300 mm,
KP=327,67, KI=âˆ’327,68, KD=0 (Format-Grenzwerttest):

```text
A5 5A 20 2C 01 FF 7F 00 80 00 00 44 6E
```

CRC-Zahlenwert: 0x6E44. Beide Beispiele werden durch Firmware- und PC-Tests geprÃ¼ft.

## 5. Senderessourcen und Diagnose

Das Board ÃƒÂ¼bergibt ein Telemetriepaket nur, wenn im 16-Wort-TX-FIFO mindestens
acht PlÃƒÂ¤tze frei sind. Bei Platzmangel wird das komplette Paket ausgelassen;
es wird kein Teilpaket geschrieben. uart_tx_skipped zÃƒÂ¤hlt diese FÃƒÂ¤lle.
Eine Wiederholung dieses Pakets erfolgt nicht; der nÃƒÂ¤chste Sendeversuch
verwendet die dann aktuellen Werte. Eine erfolgreiche FIFO-ÃƒÅ“bergabe ist
keine BestÃƒÂ¤tigung der Zustellung an die PC-Anwendung.

| Debuggervariable | Bedeutung |
| --- | --- |
| UART_Group | Zuletzt angenommene Gruppe, 1..32 |
| height_pid.KP/KI/KD | Aktive PID-VerstÃ¤rkungen als float |
| TOF_Setpoint_mm | Aktuell gespeicherter HÃƒÂ¶hensollwert |
| uart_setpoint_count | Anzahl angenommener Parameterbefehle |
| UARTLIB_RxErrors | Erkannte Empfangsfehlerereignisse |
| uart_tx_skipped | Nicht ÃƒÂ¼bergebene Telemetriepakete |
| UARTLIB_ActualBaudrate | Berechnete Baudrate des UART-Generators |

Diese Diagnosewerte werden nicht als zusÃƒÂ¤tzliche Felder ÃƒÂ¼ber UART ÃƒÂ¼bertragen.
Der Kommunikationsverlust allein lÃƒÂ¶scht den Sollwert nicht und schaltet den
Regler nicht ab. Die messwertabhÃƒÂ¤ngige PWM-Abschaltung ist in README_PID.md
beschrieben.

## 6. Referenzprogramm und PrÃƒÂ¼fung

Tools/uart_monitor.py verwendet Python und pyserial. Beispiel:

```text
python Tools/uart_monitor.py COM8 --baud 128000 --group 1 --setpoint 150 --kp 0.33 --ki 0.10 --kd 0.02
```

COM8 ist durch den tatsÃƒÂ¤chlichen Port zu ersetzen. Das Programm sendet optional
einen Parameterbefehl und dekodiert die Telemetrie. Seine CSV-Ausgabe
`Hoehe_mm,Sollwert_mm,PWM_Prozent` entsteht erst auf dem PC und ist nicht das
UART-Leitungsformat. Das Hilfsprogramm implementiert keine automatische
Befehlswiederholung und keine explizite QuittierungsprÃƒÂ¼fung.

Alle fÃ¼nf Sendeoptionen sind gemeinsam erforderlich; ohne diese Optionen wird
nur empfangen. KP/KI/KD werden auf der Kommandozeile als Dezimalwerte angegeben,
nicht als int16-Rohwerte. Mehr als zwei relevante Nachkommastellen werden abgewiesen.

Die Beispielbefehle wurden mit dem bestehenden PC-Encoder berechnet und
gegen UARTPROTO_Crc geprÃƒÂ¼ft. Tests/validate_uart.py prÃƒÂ¼ft das Firmwareformat,
CRC, Wertebereiche und ParserfÃƒÂ¤lle mit kompiliertem ARM-Code.
Tests/test_uart_monitor.py prÃƒÂ¼ft die PC-Telemetriedekodierung.
Diese SoftwareprÃƒÂ¼fung ersetzt keinen Test von USB-BrÃƒÂ¼cke und Board.

## 7. Anzeige der Gruppe

Die feste, 16 Pixel hohe Titelzeile lautet `TOF_PID_PWM 00` nach Reset.
Nach einem gueltigen Parametertelegramm wird 01..32 zweistellig angezeigt,
auch in der Oszilloskopansicht. Ungueltige Telegramme aendern die Anzeige nicht.
Der Plot liegt darunter auf OLED-Seiten 2..6 (128 x 40 Pixel).
Die Fusszeile auf Seite 7 zeigt die Zeitachsenbeschreibung.
