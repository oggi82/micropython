**TODO systick zu ende implementieren -> thread einbauen
**TODO in systick berechnen der freq einbauen
**DONE auslagern der SystemCoreClockSetup in separate datei.
**TODO Berechnen der Freq?
**TODOImplementieren von timer,
*** TODO: Timer.init liefert das nackte einfache ccu object zurück
    Danch kann mit channel das ganze konfiguriert werden, prescaler kann man immer noch verschieben, falls man mag
**DONE Umbenennen von pyb_timerxxx in machine_timer_xxx
**TODO XMC interrupts in xmc_it.c definieren
**TODO implementieren module time
**TODO implementieren module os
**TODO implement callbacks in timer interface