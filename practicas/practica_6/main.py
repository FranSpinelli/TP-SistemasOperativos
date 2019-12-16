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
    scheduler = RoundRobinScheduler(3) #COMPORTAMIENTOS RAROS CON ESTE SCHEDULER
    #scheduler = FCFSScheduler()
    #scheduler = PriorityPreemtiveScheduler()
    #scheduler = PriorityNoPreemtiveScheduler()

    #creamos el algorithmo con el que se seleccionara la victima para poder realizar el SWAP
    algorithm = FifoAlgorithm()
    #algorithm = SecondOportunityAlgorithm()
    #algorithm = LRUAlgorithm() #LRU = least recently used

    ## new create the Operative System Kernel
    # "booteamos" el sistema operativo con el scheduler y el algoritmo a utilizar
    kernel = Kernel(scheduler, algorithm)

    # creamos el graficador de gant con el kernel con el que va a trabajar y lo suscribimos al clock
    graficadorDeGant = GantGraficator(kernel)
    HARDWARE.clock.addFirstSubscriber(graficadorDeGant)

    # ---------------------------------------------------------------------------------------------------
    #MODELOS A EJECUTAR

    #prg1 = Program("prg1.exe", [ASM.CPU_READ(2), ASM.IO(), ASM.CPU_READ(3), ASM.IO(), ASM.CPU_READ(2)])
    #prg2 = Program("prg2.exe", [ASM.CPU_READ(4), ASM.IO(), ASM.CPU_READ(1)])
    #prg3 = Program("prg3.exe", [ASM.CPU_READ(3)])

    #prg1 = Program("prg1.exe", [ASM.CPU_READ(10)])
    #prg2 = Program("prg2.exe", [ASM.CPU_READ(10)])
    #prg3 = Program("prg3.exe", [ASM.CPU_READ(10)])

    prg1 = Program("prg1.exe", [ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 1"), ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 2")])
    prg2 = Program("prg2.exe", [ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 3"), ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 4")])
    prg3 = Program("prg3.exe", [ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 5"), ASM.CPU_READ(4), ASM.CPU_WRITE("hello World 6")])
    #----------------------------------------------------------------------------------------------------

    #write programs in the fileSystem
    kernel.fileSystem.write("C:/prg1.exe", prg1)
    kernel.fileSystem.write("C:/prg2.exe", prg2)
    kernel.fileSystem.write("C:/prg3.exe", prg3)

    #execute from a path
    kernel.run("C:/prg1.exe", 1)
    kernel.run("C:/prg2.exe", 2)
    kernel.run("C:/prg3.exe", 3)
