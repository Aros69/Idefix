# idefix

## Tuto extension ev3dev pour Visual Studio Code
https://github.com/ev3dev/vscode-ev3dev-browser

## Tuto pour initialiser un répertoire Python+Ev3
https://github.com/ev3dev/vscode-hello-python

Correction : vous pouvez ici directement clone le projet.

## Problèmes d'import des libs sous Python
Il est possible de run du code sans installer les libs sur sa machine. Le ev3 
tool les comprends. Je vous conseille de les installer tout de même pour pouvoir 
faire de la complétion de code.
```
pip3 install python-ev3dev2
```
## Code demo
https://github.com/ev3dev/ev3dev-lang-python-demo

## Documentation library
https://python-ev3dev.readthedocs.io/en/ev3dev-stretch/

## Imports classique pour chaque fichier Python
```python
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
```

## Threading en Python
https://docs.python.org/3/library/threading.html