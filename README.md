# micro:Maqueen V4.2 + ROS 2: draadloos, kaart maken en zelf verkennen

Eén micro:bit, niets bijkopen. ROS 2 en RViz draaien in Docker; alleen een klein
Bluetooth-scriptje draait rechtstreeks op je computer (Docker kan op Windows niet
bij Bluetooth).

```
[micro:bit op robot] ~~Bluetooth~~ [pc/ble_relay.py] --TCP 7777--> [Docker: robot]  <-- ROS 2 -->  [Docker: desktop + RViz]
  motoren, ultrasoon,                                               bridge + kaart              browser: localhost:6080
  lijnsensoren, kompas                                              + verkennen
```

## Mappen
| map              | wat                                                    |
|------------------|--------------------------------------------------------|
| `microbit/`      | code voor de micro:bit (MakeCode/TypeScript) + flash.ps1 |
| `pc/`            | `ble_relay.py`: Bluetooth <-> netwerk                  |
| `docker/`        | ROS 2-nodes, Dockerfile, RViz-instellingen             |
| `kaarten/`       | hier komen opgeslagen kaarten                           |
| `microbit_kabel/`| optioneel: MicroPython-versie om met USB te testen     |

## 1. micro:bit programmeren (eenmalig)
```powershell
npm install -g makecode          # eenmalig
cd microbit
powershell -ExecutionPolicy Bypass -File .\flash.ps1
```
Het script compileert en kopieert `built\binary.hex` naar de MICROBIT-schijf.
(`makecode build -d` werkt niet op nieuwe Windows 11-versies omdat `wmic` weg is.)

**Kompas kalibreren:** zet daarna de micro:bit op de robot, batterijen erin,
schakelaar ON. Toont hij "TILT TO FILL SCREEN", kantel dan de hele robot in alle
richtingen tot alle lampjes branden. Daarna verschijnt een smiley.

## 2. Elke keer: starten
1. Robot aan (smiley).
2. Terminal 1, op je computer:
   ```powershell
   pip install bleak        # eenmalig
   python pc\ble_relay.py
   ```
   Wacht op "Verbonden met BBC micro:bit" (robot toont een vinkje).
3. Terminal 2, in deze map:
   ```powershell
   docker compose up -d --build
   ```
   De eerste keer duurt dit een tijd (downloads).
4. Open **http://localhost:6080** in je browser. Je ziet een Ubuntu-bureaublad.
   Open daar een terminal (menu linksonder > System Tools > LXTerminal) en typ:
   ```bash
   rviz2 -d /maqueen_rviz/maqueen.rviz
   ```
   Na een update van de bestanden: `docker compose up -d --build` en RViz opnieuw starten.

## 3. Bedienen
Scripts in deze map (rechtsklik > Uitvoeren met PowerShell, of `.\verkennen.ps1`):

| script              | knop op micro:bit | wat                                              |
|---------------------|-------------------|--------------------------------------------------|
| `verkennen.ps1`     | **B**             | zelf verkennen, daarna vanzelf terug naar huis   |
| `stop.ps1`          | **A** (noodstop)  | meteen stoppen                                    |
| `naar-huis.ps1`     | **A + B**         | terug naar het startpunt                          |
| `rijden.ps1`        |                   | zelf besturen met het toetsenbord                 |
| `status.ps1`        |                   | wat doet hij nu                                   |
| `objecten.ps1`      |                   | lijst met gevonden objecten                       |
| `kaart-opslaan.ps1` |                   | kaart opslaan in `kaarten/`                       |
| `kaart-wissen.ps1`  |                   | lege kaart                                        |

Knop **A** is een echte noodstop op de robot zelf: de motoren blijven uit tot je
B, A+B of een script gebruikt. Na de noodstop toont de micro:bit een vierkantje.

### Wat zie je in RViz
| kleur                    | betekenis                                                     |
|--------------------------|---------------------------------------------------------------|
| blauwe stip + pijl       | de robot nu, met de richting waarin hij kijkt + wat hij doet  |
| groene schijf "THUIS"    | het startpunt                                                 |
| blauwe lijn              | waar hij gereden heeft                                        |
| gele lijn + gele bol     | het pad dat hij plant en zijn doel                            |
| paarse cilinder + tekst  | gevonden object met grootte                                   |
| rode puntjes             | metingen van het laatste rondje (zijn "lidar")                |
| oranje kegel             | de ultrasone meting van dit moment                            |
| wit / zwart / grijs      | vrij / muur of obstakel / nog onbekend                        |

### Hoe hij werkt
1. **Rondje meten:** met één afstandssensor moet hij draaien om rondom te kijken.
   Dat doet hij alleen nog bij aankomst op een nieuw doel of na 1 m rijden.
2. **Positie corrigeren:** het rondje wordt over de kaart gelegd. Past het beter
   een paar cm verschoven, dan wordt zijn positie gecorrigeerd (scan matching).
3. **Doel kiezen:** de dichtstbijzijnde grens tussen vrij en onbekend gebied.
4. **Pad plannen** om muren heen, met een veiligheidsmarge, en dat pad volgen.
5. Geen onbekend gebied meer bereikbaar (of na 10 minuten): **terug naar huis**.

**Objecten herkennen:** losse obstakels met vrije ruimte rondom worden als object
gemarkeerd (plaats + grootte). Wat het is (stoel, doos ...) kan hij niet zien:
daarvoor is een camera nodig.

## 4. Afstellen (belangrijk voor een goede kaart)
Pas aan in `docker-compose.yml` en daarna `docker compose up -d`:
- `MAX_LIN`: zet de robot op de grond, laat hem 2 s vooruit rijden met teleop op
  volle snelheid en meet de afstand. Snelheid in m/s = afstand / 2 bij PWM 255.
  Hoe beter dit klopt, hoe beter de kaart.
- `MIN_PWM`: draait de robot niet tijdens het scannen, verhoog dan naar 70-90.

Sneller of voorzichtiger, live aan te passen:
```powershell
docker exec maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 param set /maqueen_slam drive_speed 0.2"
```
| parameter          | standaard | betekenis                                   |
|--------------------|-----------|---------------------------------------------|
| `drive_speed`      | 0.15      | rijsnelheid (m/s)                           |
| `turn_speed`       | 2.0       | draaisnelheid (rad/s)                       |
| `rescan_dist`      | 1.0       | na zoveel meter opnieuw een rondje meten    |
| `safe_dist`        | 0.18      | stoppen als iets dichterbij is (m)          |
| `max_explore_time` | 600       | daarna terug naar huis (s)                  |
| `scan_match`       | true      | positiecorrectie aan/uit                    |
| `cliff_stop` / `black_value` | false / 0 | stoppen aan de tafelrand (zie onder) |

**Tafelrand:** de lijnsensoren zien boven de leegte hetzelfde als zwart. Controleer
met `docker exec -it maqueen bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic echo /maqueen/line"` welke waarde
zwart geeft (0 of 1), zet `black_value` op die waarde en `cliff_stop` op true.
Of plak een rand van zwarte tape rond je speelveld.

## Wat je kunt verwachten
Dit is echte mapping met de sensoren die de Maqueen heeft, maar het is geen
lidar-SLAM:
- **Eén ultrasone sensor** met een kegel van ~15°. Muren worden dikke, wat vage
  lijnen. Gladde muren onder een schuine hoek geven soms geen echo.
- **Geen wielencoders.** De afgelegde weg wordt geschat uit snelheid x tijd, dus de
  positie verloopt langzaam. Korte ritten in een kleine ruimte (1-3 m) werken het best.
- **Richting komt van het kompas.** Dat klopt goed, behalve vlak bij ijzer
  (tafelpoten, radiatoren, kasten) en luidsprekers. Daar wordt de kaart scheef.
- Het kompas zorgt er wel voor dat de richting niet wegloopt, en scan matching
  corrigeert kleine positiefouten. Klopt `MAX_LIN` slecht, dan kan hij toch wat
  scheef thuiskomen of losse muurstukken voor objecten aanzien.
- **Opnieuw beginnen** met een nieuw startpunt: `docker compose restart robot`.

## Problemen
- **Robot-container herstart steeds:** de relay draait niet of is niet verbonden.
  Bekijk `docker logs maqueen`.
- **RViz toont niets:** wacht een paar seconden, en controleer of de Fixed Frame `map` is.
- **Relay vindt niets:** robot aan? Is de micro:bit ooit gekoppeld in Windows, verwijder
  hem dan bij Bluetooth-apparaten.
- **Robot draait de verkeerde kant op of rijdt achteruit:** wissel in `microbit/main.ts`
  `0x00` en `0x02` in `motors()`, of zet een minteken.
