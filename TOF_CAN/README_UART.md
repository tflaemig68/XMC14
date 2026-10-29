# UART Ein-/Ausgabe fuer TOF_CAN

## Pins und Ressourcen

KIT_XMC14_2Go: TX=P2.1 (ALT6), RX=P2.2. Die Pins sind bereits mit dem
XMC4200/J-Link-Onboard-Debugger verbunden. PC-Verbindung ueber USB-C und den
virtuellen J-Link-COM-Port; keine zusaetzliche Verdrahtung erforderlich.
RX wird ueber USIC0_CH0.DX3A nach DX0G gefuehrt; der Digitaleingang an P2.2
wird eingeschaltet. UARTLIB_Init() wird nach DAVE_Init() aufgerufen.

Die Bibliothek nutzt USIC0_CH0, IRQ10/USIC0_SR1, RX-FIFO-Worte 32..47 und
TX-FIFO-Worte 48..63. Das bestehende I2C auf USIC0_CH1 verwendet 0..31.
Diese Ressourcen bei weiteren DAVE-APPs freihalten. Kein UART-APP-Modell wird
hinzugefuegt; die Bibliothek konfiguriert den Kanal direkt ueber XMCLib.

## Baudrate und praktische Grenze

Voreinstellung: **128000 Baud, 8 Datenbits, keine Paritaet, 1 Stopbit (8N1),
kein Hardware-/Software-Handshake**. In Libraries/UART/UARTlib.h aenderbar.
UARTLIB_ActualBaudrate zeigt die von XMCLib berechnete Baudrate im Debugger.
Der bestehende Systemtakt wird nicht veraendert.

Windows meldete am 28.10.2026 fuer den angeschlossenen **JLink CDC UART Port
(COM8)** ueber Win32_SerialPort.MaxBaudRate den Wert **128000**. Diese
Treiberangabe wird als Voreinstellung verwendet; sie ist kein gemessener
Nachweis fuer die fehlerfreie physische Uebertragung durch die USB-Bruecke.
Infineons UART-Beispiele verwenden 115200 Baud; diese Rate kann bei Problemen
als Rueckfallwert in UARTlib.h und im PC-Programm eingestellt werden.

Der XMC1404 unterstuetzt laut Datenblatt bis zu 12 Mbit/s UART. Das ist keine
Zusicherung fuer J-Link-VCOM. Eine hoehere verlaessliche Rate ist fuer die
konkrete USB-Bruecke nicht belegt und muss am Board getestet werden. Die aktuelle
Bibliothek nutzt 16-fache Abtastung. Eine nicht einstellbare Rate fuehrt zu
Init-Fehler und Fehler-LED. PC und Firmware muessen dieselbe Baudrate verwenden.

## Binaerausgabe: acht Bytes mit Synchronisation

Alle 50 ms (UARTLIB_TELEMETRY_MS), nach einem eventuell faelligen PWM-Update:

`55 AA ch1_L ch1_H ch2_L ch2_H ch3_L ch3_H`

| Byte ab 1 | Inhalt |
| --- | --- |
| 1–2 | Synchronisation: 0x55, 0xAA |
| 3–4 | ch1: aktuelle geglaettete Hoehe, int16_t Little Endian, mm |
| 5–6 | ch2: separater Hoehensollwert, int16_t Little Endian, mm |
| 7–8 | ch3: zuletzt gesetzter PWM-Sollwert, int16_t Little Endian, Prozent |

Beispiel Hoehe 150 mm, Sollwert 300 mm, PWM 50 Prozent:
`55 AA 96 00 2C 01 32 00`.
Hoehe -1 (FF FF) bedeutet ungueltige Messung. Es gibt keine ASCII-Zahlen,
kein Zeilenende und kein Strukturpadding. Vor dem ersten PWM-Update gilt der
DAVE-Startwert 50 Prozent. Ausgabe beginnt nach Sensorinitialisierung.

UARTLIB_Write8() prueft vor der Uebergabe, ob mindestens acht Plaetze im
16-Wort-TX-FIFO frei sind. Andernfalls entfaellt das komplette Paket;
uart_tx_skipped zaehlt diese Faelle. Kein blockierendes Warten auf UART.
OLED/I2C koennen das 50-ms-Ausgabeintervall verlaengern.

Der PC-Parser sucht 55 AA, wartet auf sechs Nutzdatenbytes und verarbeitet
anschliessend das naechste Paket. Er erhaelt Teilpakete ueber USB-Lesegrenzen
hinweg und verwirft Vorspannbytes. 55 AA innerhalb eines bereits ausgerichteten
Nutzdatenbereichs wird nicht als neuer Paketanfang behandelt. Nach Byteverlust
kann er erneut nach dem Header suchen. Ohne Pruefsumme/Byte-Escaping kann
er jedoch ein beschaedigtes Paket oder einen zufaelligen Header im Nutzinhalt
nicht sicher erkennen. Eine Sendepause ist zur Synchronisation nicht erforderlich.

## Sollwert ueber UART einstellen

TOF_Setpoint_mm ist ein separater int16_t-Wert, Startwert 300 mm, Bereich
0..300 mm. Er wird im RAM gespeichert und nach Reset zurueckgesetzt.
Er wird zurueckgemeldet, aber **noch nicht fuer eine Hoehenregelung verwendet**.
Die bisherige PWM-Kennlinie mit MAX_HEIGHT_mm bleibt bestehen; es wurde kein
neuer Regler oder veraendertes Stellverhalten angefordert.

PC -> Board: sechs Byte langer Befehl:
`A5 5A SollLo SollHi CrcLo CrcHi`.
CRC-16/CCITT-FALSE ueber die ersten vier Bytes, Polynom 0x1021, Startwert 0xFFFF,
keine Reflexion, kein abschliessendes XOR. Auch Sollwert und CRC sind Little Endian.

Nur vollstaendige Befehle mit passender CRC und Sollwert 0..300 werden uebernommen.
Der naechste Telemetriedatensatz enthaelt den neuen Sollwert als Bestaetigung;
es werden keine zusaetzlichen Antwortpakete in den Telemetriestrom eingefuegt.
Der Parser resynchronisiert ueber ein gleitendes Fenster. Teilpakete werden
nach 200 ms ohne verarbeitete Bytes verworfen. UART-/FIFO-/Ringpufferfehler
verwerfen die Teilnachricht. Bei erkannten Fehlern den Befehl erneut senden.
Der IRQ kopiert Bytes in einen 128-Byte-Ringpuffer; die Pruefung laeuft in main().
Befehle einzeln senden und Bestaetigung abwarten, kein unbegrenztes Streaming.

Debugger: TOF_Setpoint_mm, uart_setpoint_count, UARTLIB_RxErrors,
uart_tx_skipped, UARTLIB_ActualBaudrate.

## PC-Beispiel

Mit Python und pyserial:

```text
python -m pip install pyserial
python Tools/uart_monitor.py COM5 --setpoint 150
```

COM5 durch den tatsaechlichen J-Link-Port ersetzen (hier als COM8 erkannt). Das Hilfsprogramm sendet
den geprueften Befehl und dekodiert anschliessend Hoehe, Sollwert und PWM.
Die Synchronisation erfolgt am Header 55 AA, unabhaengig von USB-Lesegrenzen.
--baud muss zur Firmware passen. Ein Textterminal zeigt Binaerdaten nicht sinnvoll.

## Pruefung und Quellen

Tests/validate_uart.py prueft die C-Dateien mit -Wall -Wextra -Werror und fuehrt
UARTprotocol.c als echten ARM-Code im Emulator aus: signed Grenzwerte, Bytefolge,
bekannter CRC-Pruefwert, Sollwertgrenzen, Fehler und erneute Synchronisation.
Emulator-Abhaengigkeiten: unicorn und pyelftools, optionaler Skriptparameter
fuer deren Installationsverzeichnis. FIFO-/IRQ-Verhalten, Pins und die maximale
VCOM-Baudrate muessen am Board geprueft werden. Es wird nicht automatisch geflasht.

Herstellerunterlagen:
- [KIT_XMC14_2Go User Guide, Tabelle 3](https://www.infineon.com/assets/row/public/documents/30/44/infineon-kit-xmc14-2go-usermanual-en.pdf)
- [Infineon RX-Routing DX3 nach DX0](https://community.infineon.com/t5/XMC/xmc2go-and-xmclib-for-UART-in-dave-4/td-p/297533)
- [XMC1400 Datenblatt: UART bis 12 Mbit/s](https://documentation.infineon.com/xmc1000/docs/zir1764741941297)
- [Infineon UART-Beispiel mit 115200 Baud](https://github.com/Infineon/mtb-example-xmc-uart-transmit/blob/master/README.md)
