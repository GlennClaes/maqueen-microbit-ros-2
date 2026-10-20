"""Logger die naar de ROS-log én naar /kaarten/maqueen_log.txt schrijft."""
import time


class Logboek:
    def __init__(self, ros_logger, naam, pad="/kaarten/maqueen_log.txt"):
        self.ros = ros_logger
        self.naam = naam
        self.pad = pad

    def _bestand(self, niveau, tekst):
        try:
            with open(self.pad, "a", encoding="utf-8") as f:
                f.write(f"{time.strftime('%H:%M:%S')} {niveau:5s} [{self.naam}] {tekst}\n")
        except OSError:
            pass

    def info(self, tekst):
        self.ros.info(tekst)
        self._bestand("INFO", tekst)

    def warn(self, tekst):
        self.ros.warn(tekst)
        self._bestand("WARN", tekst)

    def error(self, tekst):
        self.ros.error(tekst)
        self._bestand("FOUT", tekst)

    def debug(self, tekst):
        self._bestand("DEBUG", tekst)