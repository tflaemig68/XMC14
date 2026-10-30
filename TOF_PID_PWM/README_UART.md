> Aktuelles CAN-Protokoll: [CAN_Spezifikation.md](CAN_Spezifikation.md).
> TX: 0x100 + Gruppe; ohne Gruppe kein Senden. UART-Sollwert 0 waehlt CAN als Quelle.

# UART Ein-/Ausgabe fuer TOF_PID_PWM

Vollstaendige aktuelle Schnittstelle: [UART_Spezifikation.md](UART_Spezifikation.md).

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
| 1-2 | Synchronisation: 0x55, 0xAA |
| 3-4 | ch1: aktuelle geglaettete Hoehe, int16_t Little Endian, mm |
| 5-6 | ch2: separater Hoehensollwert, int16_t Little Endian, mm |
| 7-8 | ch3: zuletzt gesetzter PWM-Sollwert, int16_t Little Endian, Prozent |

Beispiel Hoehe 150 mm, Sollwert 300 mm, PWM 50 Prozent:
`55 AA 96 00 2C 01 32 00`.
Hoehe -1 (FF FF) bedeutet ungueltige Messung. Es gibt keine ASCII-Zahlen,
kein Zeilenende und kein Strukturpadding.
Nach erfolgreichem DAVE_Init setzt main die PWM auf 0 Prozent, bis eine
gueltige Regelung moeglich ist. Ausgabe beginnt nach Sensorinitialisierung.
Zum kurzen DAVE-Startimpuls siehe README_PID.md.

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

## Gruppe, Sollwert und PID ueber UART einstellen

PC -> Board, 13 Bytes:
`A5 5A Gruppe Soll_L Soll_H KP_L KP_H KI_L KI_H KD_L KD_H CRC_L CRC_H`.
Gruppe: uint8_t 1..32, Kennzeichnung ohne Adressfilter, Startwert 0 (noch kein gueltiger Empfang).
Sollwert: int16_t 0..300 mm. KP/KI/KD: int16_t mit Aufloesung 0,01,
Reglerwert = Rohwert / 100. Negative Parameter sind zulaessig.
CRC-16/CCITT-FALSE ueber die ersten 11 Bytes, CRC Little Endian.
Alle Felder werden nur gemeinsam nach erfolgreicher Pruefung uebernommen.
Geaenderte Reglerparameter loeschen das PID-Gedaechtnis. Reine Sollwertwechsel
behalten den I-Anteil und loeschen nur das D-Gedaechtnis.
Alle Werte sind fluechtig; es gibt keine 32 getrennten Reglerspeicher.
Alte 6-Byte-Befehle werden nicht mehr unterstuetzt.

Die Telemetrie bleibt acht Bytes lang und meldet nur Hoehe, Sollwert und PWM.
Gruppe und KP/KI/KD werden nicht quittiert. Details und CRC-Beispiele stehen
in UART_Spezifikation.md. Das Teilpaket-Zeitlimit bleibt 200 ms.

## PC-Beispiel

```text
python Tools/uart_monitor.py COM8 --group 1 --setpoint 150 --kp 0.33 --ki 0.10 --kd 0.02
```

COM8 durch den tatsaechlichen J-Link-Port ersetzen. Alle fuenf Optionen sind
zum Senden gemeinsam erforderlich; ohne Optionen wird nur empfangen.
Die Kommandozeile verwendet reale Dezimalwerte fuer KP/KI/KD. Das Werkzeug
wandelt diese exakt in Hundertstel um und lehnt unpassende Nachkommastellen ab.
Baudrate standardmaessig 128000; --baud muss zur Firmware passen.

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

Hinweis zur Reglerkorrektur vom 30.10.2026: Reine Sollwertaenderungen
ueber UART oder CAN loeschen den I-Anteil nicht mehr. Ein vollstaendiger Reset
erfolgt weiterhin bei Gruppen-/Quellen-/Verstaerkungsaenderungen sowie
Messfehlern und den vorhandenen Zeitueberschreitungen.
