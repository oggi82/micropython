# XMC-Port: Plan für UART, LIN, CAN, ADC, DAC, EBU

Dieses Dokument ist eine Umsetzungsplanung, kein fertiger Code. Es baut auf
der bereits existierenden `machine.Timer`-Implementierung (CCU4/CCU8, siehe
`timer.c`) auf, die die grundlegenden Architekturentscheidungen für diesen
Port bereits getroffen hat. Jeder neue Abschnitt unten folgt bewusst dem
gleichen Muster, damit der Port als Ganzes konsistent bleibt.

Board: RELAX_LITE_KIT (XMC4500-F144x1024, LQFP144), 57 über die Header X1/X2
herausgeführte GPIOs (siehe `boards/RELAX_LITE_KIT/pins.csv`). Alle
Pin-Angaben unten beziehen sich ausschließlich auf diese 57 Pins, nicht auf
das volle Silizium-Pinout.

## Etabliertes Muster (aus der Timer-Implementierung)

Jede neue Peripherie sollte dieses Schema wiederverwenden:

1. **Pin-AF-Tabelle statt Laufzeit-Auflösung.** `pin_af_obj_t` (in `pin.h`)
   bekommt pro Peripherie einen neuen `AF_FN_*`-Wert und einen Satz
   `AF_PIN_TYPE_*`-Werte (in `pin_defs_xmc.h`), die direkt aus dem
   Infineon-Datenblatt (Table 11, "Port I/O Functions") und/oder den
   vendorten XMCLib-Headern (`xmc4_gpio_map.h`, `xmc4_*_map.h`) befüllt
   werden — siehe `boards/RELAX_LITE_KIT/pins_RELAX_LITE_KIT.c` als
   Referenz. `idx` trägt je nach Richtung entweder die GPIO-ALT-Nummer
   (Output) oder den rohen Input-Select-Code aus der jeweiligen
   `xmc4_*_map.h` (Input) — exakt wie bei CCU4/8.
2. **Flache Geräte-IDs statt Modul-Granularität**, wenn ein Modul mehrere
   unabhängige Instanzen enthält (bei CCU4/8 war das die Slice-Granularität;
   bei USIC die Kanal-Granularität, bei CAN die Node-Granularität).
3. **Direkte XMCLib-Aufrufe**, keine eigene Registerarithmetik, wo die Lib
   eine Funktion anbietet. Die Lib ist vollständig genug für alle sechs
   Peripherien hier (siehe unten) — es musste bei CCU4/8 kein einziges
   Register direkt beschrieben werden, und das gilt auch hier.
4. **IRQ-Callbacks über `mp_sched_schedule()`**, nie direkt aus dem
   ISR-Kontext in die Python-VM springen (siehe `xmc_it.c`,
   `MICROPY_ENABLE_SCHEDULER` ist bereits aktiviert).
5. **Ein Pin kann nur eine ALT-Funktion gleichzeitig führen.** Das ist eine
   Hardware-Tatsache, keine Einschränkung dieses Ports: Pins, die schon für
   CCU4/8 in der AF-Tabelle stehen, bekommen trotzdem zusätzliche
   `AF_FN_UART`/`AF_FN_CAN`/… Einträge (ein Pin kann mehrere *mögliche*
   Funktionen in der Tabelle haben), aber zur Laufzeit darf der Nutzer immer
   nur eine davon aktiv konfigurieren. Siehe Pin-Konfliktmatrix weiter unten.

## Priorisierung

Empfohlene Reihenfolge, nach Nutzen/Aufwand/Risiko:

| # | Peripherie | Aufwand | Begründung |
|---|---|---|---|
| 1 | **UART** | mittel | Breitester Nutzen, vollständiger Treiber vorhanden, Vorstufe für LIN |
| 2 | **ADC** | niedrig-mittel | Sehr häufig gebraucht, vollständiger Treiber (`xmc_vadc.h`), 16 reine Analog-Pins schon identifiziert |
| 3 | **DAC** | niedrig | Nur 2 Kanäle, vollständiger dedizierter Treiber (`xmc_dac.h`) — einfacher als erwartet |
| 4 | **CAN** | mittel-hoch | Guter Treiber, aber Message-Objekt-Verwaltung ist komplexer als UART/ADC |
| 5 | **LIN** | niedrig (nach UART) | Kein eigenes Hardware-Mode — reine Software-Schicht über UART |
| 6 | **EBU** | *nicht empfohlen* | Treiber existiert, aber das Board-Pinout ist dafür zu fragmentiert (siehe unten) |

---

## 1. UART (über USIC)

XMC4500 hat keinen eigenen UART-Block wie STM32; serielle Kommunikation
läuft über die USIC-Kanäle (Universal Serial Interface Channel), die auch
SPI/I2C/I2S können. 3 Module × 2 Kanäle = 6 unabhängige Instanzen:
`U0C0, U0C1, U1C0, U1C1, U2C0, U2C1` → flache ID 0-5
(`id = modul*2 + kanal`).

**Treiber:** `Libraries/XMCLib/inc/xmc_uart.h`, vollständig:
- `XMC_UART_CH_CONFIG_t` (baudrate, data_bits, stop_bits, parity_mode, oversampling)
- `XMC_UART_CH_Init()` + `XMC_UART_CH_Start()`
- `XMC_UART_CH_Transmit()` (einzelnes Byte, blockierend ohne FIFO) /
  `XMC_UART_CH_GetReceivedData()`
- FIFO: `XMC_USIC_CH_TXFIFO_PutData()` / `RXFIFO_*` (in `xmc_usic.h`)
- RX-Pin-Routing: `XMC_UART_CH_SetInputSource(channel, input, source)` —
  `source` ist der rohe Code aus `xmc4_usic_map.h`
  (z. B. `USIC1_C0_DX0_P0_4 = 0`), exakt das gleiche Prinzip wie
  `CC4yINS` bei Timer-Capture.
- Events/IRQ: `XMC_UART_CH_EnableEvent()`, `SetInterruptNodePointer()` /
  `SelectInterruptNodePointer()` für SR-Routing — analog zu
  `XMC_CCU4_SLICE_SetInterruptNode()`.

**Verfügbare Pins auf diesem Board** (TX = ALT-Output, RX = Input-Select):

| Pin | TX | RX |
|---|---|---|
| P0.0 | — | U1C1.DX0D |
| P0.1 | ALT2→U1C1.DOUT0 | — |
| P0.4 | — | U1C0.DX0A |
| P0.5 | ALT2→U1C0.DOUT0 | U1C0.DX0B |
| P0.6 | — | U1C0.DX2A |
| P0.7 | — | U0C0.DX2B |
| P0.8 | — | U0C0.DX1B |
| P0.11 | — | U1C0.DX1A |
| P0.12 | — | U1C1.DX2B |
| P1.0 | — | U0C0.DX2A |
| P1.3 | — | U0C0.DX0A |
| P1.4 | HWO0→U0C0.DOUT1 | U0C0.DX0B |
| P1.5 | ALT2→U0C0.DOUT0 | U0C0.DX0A |
| P2.6 | HWO0→U2C0.DOUT3 | U2C0.DX1B |
| P2.14 | ALT2→U1C0.DOUT0 | U1C0.DX0D |
| P2.15 | — | U1C0.DX0C |
| P3.0 | ALT2→U0C1.SCLKOUT* | U0C1.DX1B |
| P3.1 | — | U0C1.DX2B |
| P3.4 | — | U2C1.DX0B |
| P5.0 | ALT1→U2C0.DOUT0 | U2C0.DX0B |
| P5.1 | ALT2→U0C0.DOUT0 | U0C0.DX0A |
| P5.2 | — | U2C0.DX1A |
| P5.7 | HWO0→U2C0.DOUT2 | — |

(\* P3.0 zeigt im Datenblatt `SCLKOUT`, nicht `DOUT` — vor Implementierung
gegenprüfen, ob das als UART-TX taugt oder nur als SPI-Takt gedacht ist.)

Mindestens **U0C0 (P1.5 TX + P1.5/P1.3/P1.0 RX)** und **U1C0 (P0.5 TX +
P0.5/P2.14/P2.15 RX)** haben saubere TX+RX-Paare auf demselben logischen
Kanal — guter Startpunkt für die erste Implementierung/den ersten Test.

**Vorgeschlagenes API** (angelehnt an `machine.UART` anderer Ports):

```python
uart = machine.UART(0, baudrate=115200, tx=machine.Pin.board.P1_5, rx=machine.Pin.board.P1_3)
uart.write(b"hello")
uart.read()
uart.any()
```

`id` = flache USIC-Kanal-ID (0-5) wie oben. `tx=`/`rx=` lösen über die
AF-Tabelle auf (gleiches `find_timer_af`-Muster wie in `timer.c`, nur für
`AF_FN_UART`).

**Offene Fragen:**
- Lohnt sich FIFO-Nutzung von Anfang an, oder erst Polling (einfacher,
  reicht für REPL-Baudraten)? Empfehlung: Polling zuerst, FIFO später für
  höhere Baudraten/DMA-lose Entlastung.
- `HWO0`-markierte TX-Pins (P1.4, P2.6, P5.7) sind "Hardware Output 0"
  (siehe Table 10 im Datenblatt) statt eines einfachen ALTn — das ist ein
  anderer Konfigurationsweg (`Pn_HWSEL`) als die übrigen ALTn-Pins und
  müsste gesondert behandelt werden. Für die erste Implementierung auf die
  ALTn-Pins beschränken, HWO-Pins später nachziehen.

---

## 2. LIN (Software-Schicht über UART)

**Wichtigste Erkenntnis:** XMCLib/USIC hat **keinen eigenen LIN-Hardware-Modus**
(bestätigt: `xmc_uart.h` enthält keine LIN-Enums/Structs/Funktionen). LIN ist
Standard-UART (8N1, meist 9600/19200 Baud) plus Software:

- Sync-Break-Erzeugung zum Frame-Start (USIC kann das: siehe
  `XMC_UART_CH_EnableEvent(XMC_UART_CH_EVENT_SYNCHRONIZATION_BREAK)` und die
  Pulslängen-Konfiguration in `XMC_UART_CH_SetInputSource`-Nachbarschaft,
  Zeile ~594 in `xmc_uart.h` — das deckt den LIN-Break ab, muss aber geprüft
  werden, ob es Senden *und* Erkennen beim Empfang abdeckt).
- LIN-Checksumme (klassisch oder erweitert) — reines Software-Byte-Rechnen,
  kein Hardware-Bedarf.
- Frame-Timing/Identifier-Byte-Parsing — Software auf Basis der UART-Bytes.

**Konsequenz für die Architektur:** Kein eigenes `AF_FN_LIN`, kein eigenes
Peripherie-Objekt. Stattdessen entweder:
- (a) `machine.UART` bekommt einen `lin=True`-Init-Parameter, der Break-Event
  aktiviert und Checksummen-Hilfsmethoden freischaltet, oder
- (b) ein reines Python-Modul `lin.py` (oder C-Modul `machine.LIN`), das
  intern ein `UART`-Objekt hält und Break/Checksumme obendrauf
  implementiert.

Empfehlung: (b), weil es UART nicht mit LIN-Spezialfällen verkompliziert und
komplett nach UART fertig ist, separat und risikoarm nachgezogen werden
kann.

---

## 3. ADC (VADC)

VADC = "Versatile ADC", 4 Gruppen (`VADC_G0`-`G3`) × 8 Kanäle, insgesamt bis
zu 32 Kanäle, 16 Ergebnisregister pro Gruppe.

**Treiber:** `Libraries/XMCLib/inc/xmc_vadc.h`, vollständig:
- `XMC_VADC_GLOBAL_CONFIG_t` + `XMC_VADC_GLOBAL_Init()`
- `XMC_VADC_GROUP_CONFIG_t` + `XMC_VADC_GROUP_Init()`
- `XMC_VADC_CHANNEL_CONFIG_t` pro Kanal
- Drei unabhängige Trigger-Quellen: Scan
  (`XMC_VADC_GROUP_ScanTriggerConversion`), Background
  (`XMC_VADC_GLOBAL_BackgroundTriggerConversion`), Queue
  (`XMC_VADC_GROUP_QueueTriggerConversion`) — für einen einfachen
  `machine.ADC.read()` reicht Scan oder Background (einzelne
  Software-getriggerte Conversion, Ergebnis per `GetResult()` abholen).

**Verfügbare Pins auf diesem Board** (alle rein analog, keine ALT-Funktion
daneben):

| Pin | VADC-Kanal(-Optionen) |
|---|---|
| P14.0 | G0CH0 |
| P14.1 | G0CH1 |
| P14.2 | G0CH2 \| G1CH2 |
| P14.3 | G0CH3 \| G1CH3 |
| P14.4 | G0CH4 \| G2CH0 |
| P14.5 | G0CH5 \| G2CH1 |
| P14.6 | G0CH6 |
| P14.7 | G0CH7 |
| P14.8 | G1CH0 \| G3CH2 (**und DAC.OUT_0**, siehe unten) |
| P14.9 | G1CH1 \| G3CH3 (**und DAC.OUT_1**) |
| P14.12 | G1CH4 |
| P14.13 | G1CH5 |
| P14.14 | G1CH6 |
| P14.15 | G1CH7 |
| P15.2 | G2CH2 |
| P15.3 | G2CH3 |

Keiner der übrigen 41 Board-Pins (P0/P1/P2/P3/P5) hat einen echten
VADC-Eingangskanal — nur P2.2/2.3/2.4/2.10/2.14/2.15 haben
`VADC.EMUXxx`-Einträge, das sind aber Multiplexer-*Steuersignale*, keine
Analogeingänge, und daher für `machine.ADC` nicht relevant.

**Vorgeschlagenes API:**

```python
adc = machine.ADC(machine.Pin.board.P14_0)   # löst Gruppe+Kanal über AF-Tabelle auf
value = adc.read_u16()
```

`AF_FN_ADC`, `unit` = Gruppe (0-3), `idx`/`type` = Kanalnummer (0-7). Da ein
Pin wie P14.2 zwei Gruppen-Optionen hat (G0CH2 *oder* G1CH2), braucht die
AF-Tabelle hier — anders als bei CCU4/8 — zwei Einträge pro Pin mit
unterschiedlichem `unit`, und `ADC(pin)` müsste die erste nehmen oder eine
optionale `group=`-Kwarg anbieten, falls beide Gruppen gleichzeitig
gebraucht werden (z. B. für parallele Conversion in zwei Gruppen).

**Offene Fragen:**
- Referenzspannung/Kalibrierung: `XMC_VADC_GLOBAL_Init()` braucht
  vermutlich Board-spezifische Referenzwerte — im Detail noch nicht
  recherchiert, nötig vor Implementierung.
- Soll `ADC.read_u16()` blockierend pollen (einfach, reicht für die meisten
  Fälle) oder über Interrupt/Scan-Sequenz laufen (nötig für z. B.
  synchrones Mehrkanal-Sampling)? Empfehlung: blockierend zuerst.

---

## 4. DAC

**Wichtigste Erkenntnis:** Anders als ursprünglich angenommen gibt es einen
**vollständigen dedizierten Treiber** `Libraries/XMCLib/inc/xmc_dac.h` —
keine rohe Registerarbeit nötig. 2 Kanäle, 12 Bit, feste Pins:

| Pin | Kanal |
|---|---|
| P14.8 | DAC.OUT_0 |
| P14.9 | DAC.OUT_1 |

**Treiber:**
- `XMC_DAC_CH_CONFIG_t` + `XMC_DAC_CH_Init()`
- `XMC_DAC_CH_Write(dac, channel, value)` — 16-Bit-Wert (effektiv 12 Bit
  genutzt)
- Unterstützt neben "Single Value" auch Pattern/Noise/Ramp-Generator-Modi
  und externe Trigger (u. a. von CCU40/41/80) — für `machine.DAC` reicht
  "Single Value", die anderen Modi sind eine mögliche spätere Erweiterung
  (z. B. ein Software-Sinusgenerator per Ramp-Modus).

**Vorgeschlagenes API:**

```python
dac = machine.DAC(machine.Pin.board.P14_8)
dac.write(2048)   # 0..4095
```

Da es nur 2 feste Pins gibt, ist `AF_FN_DAC` mit `unit` = Kanal (0/1)
trivial — kein Konfliktpotential mit anderen Peripherien auf diesen zwei
Pins außer VADC (P14.8/14.9 können wahlweise als ADC-Eingang *oder*
DAC-Ausgang laufen, nicht beides gleichzeitig).

---

## 5. CAN (MultiCAN)

Ein physischer CAN-Controller-Kern, bis zu 3 logische Nodes
(`CAN_NODE0/1/2`), 64 gemeinsam genutzte Message-Objekte (Pool über alle
Nodes).

**Treiber:** `Libraries/XMCLib/inc/xmc_can.h`, vollständig:
- `XMC_CAN_Init(obj, can_frequency)` — global
- `XMC_CAN_NODE_NOMINAL_BIT_TIME_CONFIG_t` (can_frequency, baudrate,
  sample_point, sjw) + `XMC_CAN_NODE_NominalBitTimeConfigure()`
- `XMC_CAN_MO_t` (Identifier, ID-Mode 11/29-Bit, Priorität, Maske,
  Datenlänge, bis 8 Byte Daten, RX/TX-Typ) + `XMC_CAN_MO_Config()`
- Senden: `XMC_CAN_MO_Transmit()`. Empfangen:
  `XMC_CAN_MO_Receive()` / `ReceiveData()`
- RX-Pin-Routing: `XMC_CAN_NODE_SetReceiveInput(node, input)` mit Enum
  `XMC_CAN_NODE_RECEIVE_INPUT_t` (RXDCA..RXDCH) — Pin→Enum-Zuordnung über
  `xmc_can_map.h`, gleiches Prinzip wie bei USIC/CCU4.
- Events: `XMC_CAN_NODE_EnableEvent()` / `XMC_CAN_MO_EnableEvent()`, NVIC
  wie üblich von Hand verdrahten.

**Verfügbare Pins auf diesem Board:**

| Pin | TX | RX |
|---|---|---|
| P0.0 | ALT2→CAN.N0_TXD | — |
| P1.4 | ALT2→CAN.N0_TXD | CAN.N1_RXDD |
| P1.5 | ALT1→CAN.N1_TXD | CAN.N0_RXDA |
| P1.8 | — | CAN.N2_RXDA |
| P1.9 | ALT2→CAN.N2_TXD | — |
| P1.12 | ALT2→CAN.N1_TXD | — |
| P1.13 | — | CAN.N1_RXDC |
| P2.6 | — | CAN.N1_RXDA |
| P14.3 | — | CAN.N0_RXDB |

Node 0: TX auf P0.0 *oder* P1.4, RX auf P1.5 oder P14.3 — vollständiges
Paar vorhanden (z. B. P1.4 TX + P1.5 RX, beide im selben Header-Bereich).
Node 1 und Node 2 haben ebenfalls je ein brauchbares TX/RX-Paar
(P1.12/P2.6 bzw. P1.9/P1.8).

**Vorgeschlagenes API** (angelehnt an bestehende Ports):

```python
can = machine.CAN(0, tx=machine.Pin.board.P1_4, rx=machine.Pin.board.P1_5, baudrate=500_000)
can.send([0x01, 0x02], id=0x123)
can.recv()
```

`id` = Node-Nummer (0-2). `AF_FN_CAN`, `unit` = Node, `type` = TX/RX, `idx`
= ALT-Nummer (TX) bzw. `XMC_CAN_NODE_RECEIVE_INPUT_t`-Wert (RX).

**Offene Fragen:**
- Message-Objekt-Verwaltung: feste Zuteilung (z. B. MO 0-7 für Node 0) vs.
  dynamische Allokation bei `can.send()`/`recv()`? Feste Zuteilung ist
  einfacher und reicht für den Anfang.
- Filterung: vorerst nur ein RX-Message-Objekt mit offener Maske (alles
  empfangen), feinere Filter später.

---

## 6. EBU (External Bus Unit) — nicht empfohlen für dieses Board

**Treiber existiert vollständig:** `xmc_ebu.h`/`xmc_ebu.c`,
`XMC_EBU_CONFIG_t` + `XMC_EBU_Init()`. Die Einschränkung liegt **nicht** an
der Software, sondern am Pinout dieses konkreten Boards.

**Verfügbare EBU-Signale auf den 57 Board-Pins:**

| Typ | Gefundene Pins | Lücken |
|---|---|---|
| Adress/Daten (AD0-31) | AD0-7, AD12-19, AD21, AD28 | AD8-11, AD20, AD22-27, AD29-31 fehlen komplett |
| Chip-Select | CS0 (P3.2), CS1 (P0.9) | nur 2 von möglichen mehreren |
| Steuersignale | RD (P3.0), RD_WR (P3.1), WAIT (P3.3), HOLD (P3.4), ADV (P0.6), BREQ (P0.11), HLDA (P0.12), BC0/BC1 (P2.14/15) | — |

**Verdikt:** Für einen allgemeinen externen Speicher-/Peripheriebus
(zusammenhängender Adressraum) reicht das nicht — der Adressbus hat zu
große Lücken, und der Datenbus ist über mehrere Ports fragmentiert, was in
der Praxis bedeutet: viele der betroffenen Pins sind *bereits* für
CCU4/CCU8-PWM/Capture in der aktuellen AF-Tabelle genutzt (z. B. P1.2/P1.3
= CCU40, P0.12 = CCU40, P1.12-15 = CCU81), sodass ein EBU-fähiger
Adressbus ohnehin mit der Timer-Pinbelegung kollidieren würde.

**Wenn es trotzdem gebraucht wird:** am ehesten machbar wäre ein 8-Bit
Datenbus (D0-7, vollständig auf P0.2-0.8 vorhanden) mit einem einzigen
Chip-Select (CS0 auf P3.2) für genau *ein* externes 8-Bit-Peripheriegerät
mit fest verdrahteter/dekodierter Adressierung (kein echter Adressbus nötig,
z. B. ein externer Port-Expander oder ein einzelnes Speicher-IC mit eigener
Adresslogik) — nicht als allgemeines `machine.EBU`-API, sondern als
Board-spezifische Spezialanwendung, falls konkret gebraucht. Für den
normalen Port-Funktionsumfang empfehle ich, EBU zurückzustellen, bis ein
konkreter Anwendungsfall mit bekanntem externen Chip vorliegt — dann lässt
sich gezielt prüfen, ob dessen Pinbedarf zu den hier verfügbaren Signalen
passt.

---

## Pin-Konfliktmatrix (Auszug, nur Pins mit mehr als einer neuen Funktion)

Pins, die schon eine CCU4/8-Funktion aus der Timer-Arbeit UND mindestens
eine der hier geplanten Funktionen haben — zur Erinnerung beim späteren
Implementieren, dass hier eine bewusste Wahl nötig ist:

| Pin | CCU4/8 (bereits implementiert) | Neu geplant |
|---|---|---|
| P0.0 | Timer(18) PWM-Out | CAN N0 TX |
| P0.6 | Timer(19) PWM-Out, Timer(18) Capture-In | EBU.ADV |
| P1.4 | Timer(19)/(22) PWM-Out, Timer(4) Capture-In | UART TX, CAN N0 TX/N1 RX |
| P1.5 | Timer(18)/(21) PWM-Out, Timer(5) Capture-In | UART TX/RX, CAN N1 TX/N0 RX |
| P1.12 | Timer(20) PWM-Out | CAN N1 TX |
| P1.13 | Timer(22) PWM-Out | CAN N1 RX |
| P2.6 | Timer(17) PWM-Out, Timer(3) Capture-In | UART TX/RX, CAN N1 RX |
| P2.14 | Timer(18) PWM-Out, Timer(12-15) Capture-In | UART TX/RX |
| P2.15 | Timer(17) PWM-Out, Timer(8-11) Capture-In | UART RX |
| P5.0 | Timer(23) PWM-Out, Timer(20-23) Capture-In | UART TX/RX |
| P5.1 | Timer(23) PWM-Out, Timer(20) Capture-In | UART TX/RX |
| P5.2 | Timer(22) PWM-Out, Timer(21) Capture-In | UART RX |
| P5.7 | Timer(20) PWM-Out | UART TX |
| P14.8/14.9 | — (reine Analog-Pins) | ADC **und** DAC (gegenseitig exklusiv) |

Das ist normal und erwartbar (dieselben physischen Pins werden von
verschiedenen On-Chip-Peripherien beansprucht) — die AF-Tabelle erlaubt
mehrere Einträge pro Pin über verschiedene `fn`-Werte, aber zur Laufzeit
konfiguriert jede `init()`-Aufruf-Kette (Timer, UART, CAN, ADC, DAC) den
GPIO-Modus des Pins neu und überschreibt damit die vorherige Zuordnung —
kein zusätzlicher Schutzmechanismus nötig, das entspricht dem Verhalten
jedes anderen MicroPython-Ports.

## Nächste Schritte

1. UART-Treiber nach obigem Muster (eigener `uart.c`, analog zu `timer.c`
   strukturiert) — kleinster Schritt mit größtem Nutzen.
2. ADC direkt danach (keine Pin-Konflikte mit UART, eigener Analog-Pin-Satz).
3. DAC (trivial, 2 Pins, direkt nach ADC).
4. LIN als dünne Software-Schicht über UART.
5. CAN.
6. EBU zurückstellen, bis ein konkreter externer Chip/Anwendungsfall feststeht.
