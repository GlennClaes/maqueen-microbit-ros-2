#!/usr/bin/env python3
"""Bluetooth-relay: verbindt de micro:bit (Bluetooth UART) met een TCP-poort.

Draai dit op je computer zelf (niet in Docker), want daar zit de Bluetooth.
Werkt op Windows, Linux en macOS.

    pip install bleak
    python ble_relay.py              # zoekt de eerste "BBC micro:bit"
    python ble_relay.py zuvav        # of een specifieke (naam staat in de app/scan)

De ROS 2-node verbindt daarna met  socket://<deze computer>:7777
Valt Bluetooth weg, dan probeert het script automatisch opnieuw.
"""
import asyncio
import sys

from bleak import BleakClient, BleakScanner

TCP_HOST = "0.0.0.0"   # ook bereikbaar vanuit Docker; zet op 127.0.0.1 zonder Docker
TCP_PORT = 7777
UART_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"


class Relay:
    def __init__(self, naamfilter):
        self.naamfilter = naamfilter.lower()
        self.client = None
        self.schrijf_char = None
        self.met_antwoord = True
        self.tcp = None

    # ---------- Bluetooth ----------
    def van_microbit(self, _char, data: bytearray):
        if self.tcp is not None:
            self.tcp.write(bytes(data))

    async def ble_lus(self):
        while True:
            print("Zoeken naar micro:bit...")
            apparaat = await BleakScanner.find_device_by_filter(
                lambda d, _ad: bool(d.name)
                and "micro:bit" in d.name.lower()
                and self.naamfilter in d.name.lower(),
                timeout=10.0,
            )
            if apparaat is None:
                print("Niet gevonden. Staat de robot aan? Opnieuw proberen...")
                continue

            print(f"Gevonden: {apparaat.name} ({apparaat.address}), verbinden...")
            verbroken = asyncio.Event()
            opties = dict(disconnected_callback=lambda _c: verbroken.set())
            if sys.platform == "win32":
                # Windows onthoudt de oude diensten na opnieuw flashen; altijd vers opvragen
                opties["winrt"] = {"use_cached_services": False}
            try:
                try:
                    client = BleakClient(apparaat, **opties)
                except TypeError:
                    opties.pop("winrt", None)
                    client = BleakClient(apparaat, **opties)
                async with client:
                    dienst = client.services.get_service(UART_SERVICE)
                    if dienst is None:
                        print("Geen UART-dienst gevonden. Diensten op de micro:bit:")
                        for d in client.services:
                            print("   ", d.uuid, d.description)
                        print("Staat de nieuwe code erop? Zet de robot uit en weer aan.")
                        await asyncio.sleep(5)
                        continue

                    # De micro:bit gebruikt de UUID's omgekeerd t.o.v. Nordic,
                    # dus we kiezen de kenmerken op basis van hun eigenschappen.
                    for c in dienst.characteristics:
                        if "indicate" in c.properties or "notify" in c.properties:
                            await client.start_notify(c, self.van_microbit)
                        if "write" in c.properties or "write-without-response" in c.properties:
                            self.schrijf_char = c
                            self.met_antwoord = "write" in c.properties

                    self.client = client
                    print(f"Verbonden met {apparaat.name}")
                    await verbroken.wait()
            except Exception as e:
                print("Bluetooth-fout:", e)
            finally:
                self.client = None
                self.schrijf_char = None
            print("Verbinding verbroken, opnieuw verbinden...")
            await asyncio.sleep(1)

    # ---------- TCP ----------
    async def tcp_klant(self, reader, writer):
        print("ROS-bridge verbonden via TCP")
        self.tcp = writer
        try:
            while True:
                regel = await reader.readline()
                if not regel:
                    break
                if self.client is not None and self.schrijf_char is not None:
                    try:
                        await self.client.write_gatt_char(
                            self.schrijf_char, regel, response=self.met_antwoord
                        )
                    except Exception as e:
                        print("Schrijffout:", e)
        finally:
            self.tcp = None
            writer.close()
            print("ROS-bridge losgekoppeld")


async def main():
    relay = Relay(sys.argv[1] if len(sys.argv) > 1 else "")
    server = await asyncio.start_server(relay.tcp_klant, TCP_HOST, TCP_PORT)
    print(f"Wacht op ROS-bridge op poort {TCP_PORT}")
    async with server:
        await asyncio.gather(server.serve_forever(), relay.ble_lus())


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
