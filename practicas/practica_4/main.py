from hardware import *
from so import *
import log

##
##  MAIN 
##
if __name__ == '__main__':
    log.setupLogger()
    log.logger.info('Starting emulator')

    ## setup our hardware and set memory size to 25 "cells"
    HARDWARE.setup(25)

    ## Switch on computer
    HARDWARE.switchOn()

    ##creamos el scheduler con el que vamos a trabajar
    # scheduler = RoundRobinScheduler(2) ##en el constructor del RoundRobinScheduler se le indica el Quantum con el que se va a trabajar
    scheduler = FCFSScheduler()

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo con el scheduler a utilizar
    kernel = Kernel(scheduler)

    # creamos el graficador de gant con el kernel con el que va a trabajar y lo suscribimos al clock
    graficadorDeGant = GantGraficator(kernel)
    HARDWARE.clock.addSubscriber(graficadorDeGant)

    prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    prg2 = Program("prg2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])
    prg3 = Program("prg3.exe", [ASM.CPU(3)])
    # ---------------------------------------------------------------------------------------------------
    # prg1 = Program("prg1.exe", [ASM.CPU(2)])
    # prg2 = Program("prg2.exe", [ASM.CPU(4)])
    # prg3 = Program("prg3.exe", [ASM.CPU(3)])

    # execute all programs "concurrently"
    kernel.run(prg1)
    kernel.run(prg2)
    kernel.run(prg3)
