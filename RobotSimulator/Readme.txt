HTWG_Robot_Simulator_V2.0
09.03.2016
Oliver Bittel
Fakultät Informatik


Umgebung:
=========
Module wurden mit Python 2.7 getestet und
der Entwicklungsumgebung PyCharm 3.4.

Installierte Pakete:
numpy (zum Rechnen mit Vekoren und Matrizen)
matplotlib (nützlich, falls Graphiken geplottet werden sollen; ähnlich wie plot von matlab)


Benutzung:
==========
1. demo_xxx:
demo_xxx sind kleine Hauptprogramme, die vorhandene Klassen testen.


2. graphics.py:
graphics.py ist ein kleines Paket zum Zeichnen von Graphikobjekten.
Dokumentation siehe graphics.pdf.
Das Paket stammt von http://mcsp.wartburg.edu/zelle/python/


3. Robot.py und World.py:
Diese beiden Klassen bilden den eigentlichen Simulator.
Eine Simulation besteht aus genau einem Objekt myRobot der Klasse Robot und einem
Objekt myWorld der Klasse World.
myRobot wird in der Welt myWorld mit dem Aufruf
myWorld.setRobot(myRobot,x,y,theta) positioniert.

Der Roboter lässt sich über myRobot.move() steuern.
Über myRobot.sense() bzw. myRobot.senseBoxes() können
die Sensordaten abgefragt werden.

Siehe demo_Simulator_1.py


4. CursorController.py:
Der Roboter kann manuell (zu Demozwecken) über die Cursortasten gesteuert werden.
Siehe demo_Simulator_2.py


5. xxxWorld.py
Verschiedene bereits vordefinierte Welten.
officeWorld ist unsere Zielumgebung.


6. OccupancyGrid.py:
Klasse für Belegtheitsgitter.
Mit der Methode World.getOccupancyGrid() kann aus den
Weltdaten ein Belegtheitsgitter erzeugt werden.

Siehe demo_Simulator_4.py
