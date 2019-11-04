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
    HARDWARE.setup(24, 4)#the second number is the frameSize with wich the mmu will work

    ## Switch on computer
    HARDWARE.switchOn()

    ##creamos el scheduler con el que vamos a trabajar
    scheduler = RoundRobinScheduler(3)
    #scheduler = FCFSScheduler()
    #scheduler = PriorityPreemtiveScheduler()
    #scheduler = PriorityNoPreemtiveScheduler()

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo con el scheduler a utilizar
    kernel = Kernel(scheduler)

    # creamos el graficador de gant con el kernel con el que va a trabajar y lo suscribimos al clock
    graficadorDeGant = GantGraficator(kernel)
    HARDWARE.clock.addFirstSubscriber(graficadorDeGant)

    #prg1 = Program("prg1.exe", [ASM.CPU(2), ASM.IO(), ASM.CPU(3), ASM.IO(), ASM.CPU(2)])
    #prg2 = Program("prg2.exe", [ASM.CPU(4), ASM.IO(), ASM.CPU(1)])
    #prg3 = Program("prg3.exe", [ASM.CPU(3)])
    # ---------------------------------------------------------------------------------------------------
    prg1 = Program("prg1.exe", [ASM.CPU(2)])
    prg2 = Program("prg2.exe", [ASM.CPU(4)])
    prg3 = Program("prg3.exe", [ASM.CPU(3)])

    # execute all programs "concurrently"
    kernel.run(prg1, 1)
    kernel.run(prg2, 2)
    kernel.run(prg3, 3)
