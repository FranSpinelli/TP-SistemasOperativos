#!/usr/bin/env python

from tabulate import tabulate
from time import sleep
from threading import Thread, Lock
import log

##  Estas son la instrucciones soportadas por nuestro CPU
INSTRUCTION_IO = 'IO'
INSTRUCTION_CPU_WRITE = 'CPU_WRITE'
INSTRUCTION_CPU_READ = 'CPU_READ'
INSTRUCTION_EXIT = 'EXIT'


## Helper for emulated machine code
class ASM():

    @classmethod
    def EXIT(self, times):
        return [INSTRUCTION_EXIT] * times

    @classmethod
    def IO(self):
        return INSTRUCTION_IO

    @classmethod
    def CPU_WRITE(self, times):
        return [INSTRUCTION_CPU_WRITE] * times

    @classmethod
    def CPU_READ(self, times):
        return [INSTRUCTION_CPU_READ] * times

    @classmethod
    def isCPU_WRITE(self, instruction):
        return INSTRUCTION_CPU_WRITE == instruction

    @classmethod
    def isEXIT(self, instruction):
        return INSTRUCTION_EXIT == instruction

    @classmethod
    def isIO(self, instruction):
        return INSTRUCTION_IO == instruction


##  Estas son la interrupciones soportadas por nuestro Kernel
KILL_INTERRUPTION_TYPE = "#KILL"
IO_IN_INTERRUPTION_TYPE = "#IO_IN"
IO_OUT_INTERRUPTION_TYPE = "#IO_OUT"
NEW_INTERRUPTION_TYPE = "#NEW"
TIMEOUT_INTERRUPTION_TYPE = "#TIMEOUT"
PAGE_FAULT_INTERRUPTION_TYPE = "#PAGE_FAULT"
CPU_WRITE_INTERRUPTION_TYPE = "#CPU_WRITE"


## emulates an Interrupt request
class IRQ:

    def __init__(self, type, parameters=None):
        self._type = type
        self._parameters = parameters

    @property
    def parameters(self):
        return self._parameters

    @property
    def type(self):
        return self._type


## emulates the Interrupt Vector Table
class InterruptVector():

    def __init__(self):
        self._handlers = dict()
        self.lock = Lock()

    def register(self, interruptionType, interruptionHandler):
        self._handlers[interruptionType] = interruptionHandler

    def handle(self, irq):
        log.logger.info(
            "Handling {type} irq with parameters = {parameters}".format(type=irq.type, parameters=irq.parameters))
        self.lock.acquire()
        self._handlers[irq.type].execute(irq)
        self.lock.release()


## emulates the Internal Clock
class Clock():

    def __init__(self):
        self._subscribers = []
        self._running = False

    def addFirstSubscriber(self, subscriber):
        self._subscribers.insert(0, subscriber)

    def addSubscriber(self, subscriber):
        self._subscribers.append(subscriber)

    def stop(self):
        self._running = False

    def start(self):
        log.logger.info("---- :::: START CLOCK  ::: -----")
        self._running = True
        t = Thread(target=self.__start)
        t.start()

    def __start(self):
        tickNbr = 0
        while (self._running):
            self.tick(tickNbr)
            tickNbr += 1

    def tick(self, tickNbr):
        log.logger.info("        --------------- tick: {tickNbr} ---------------".format(tickNbr=tickNbr))
        ## notify all subscriber that a new clock cycle has started
        for subscriber in self._subscribers:
            subscriber.tick(tickNbr)
        ## wait 1 second and keep looping
        sleep(1)

    def do_ticks(self, times):
        log.logger.info("---- :::: CLOCK do_ticks: {times} ::: -----".format(times=times))
        for tickNbr in range(0, times):
            self.tick(tickNbr)


## emulates the main memory (RAM)
class Memory():

    def __init__(self, size):
        self._size = size
        self._cells = [''] * size

    def write(self, addr, value):
        self._cells[addr] = value

    def read(self, addr):
        return self._cells[addr]

    @property
    def size(self):
        return self._size

    def __repr__(self):
        return tabulate(enumerate(self._cells), tablefmt='psql')
        ##return "Memoria = {mem}".format(mem=self._cells)


## emulates the Memory Management Unit (MMU)
class MMU():

    def __init__(self, memory):
        self._memory = memory
        self._frameSize = 0
        self._limit = 999
        self._tlb = dict()

    @property
    def limit(self):
        return self._limit

    @limit.setter
    def limit(self, limit):
        self._limit = limit

    @property
    def frameSize(self):
        return self._frameSize

    @frameSize.setter
    def frameSize(self, frameSize):
        self._frameSize = frameSize

    @property
    def tlb(self):
        return self._tlb

    @tlb.setter
    def tlb(self, aTlb):
        self._tlb = aTlb

    def resetTLB(self):
        self._tlb = dict()

    # def setPageFrame(self, pageId, frameId):
    #    self._tlb[pageId] = frameId

    def fetch(self, logicalAddress):
        if (logicalAddress > self._limit):
            raise Exception(
                "Invalid Address,  {logicalAddress} is higher than process limit: {limit}".format(limit=self._limit,
                                                                                                  logicalAddress=logicalAddress))
        #
        # calculamos la pagina y el offset correspondiente a la direccion logica recibida 
        pageId = logicalAddress // self._frameSize
        offset = logicalAddress % self._frameSize
        #
        # buscamos la direccion Base del frame donde esta almacenada la pagina
        try:
            frameId = self._tlb[pageId][0]
        except:
            pageFaultIRQ = IRQ(PAGE_FAULT_INTERRUPTION_TYPE, pageId)
            HARDWARE.interruptVector.handle(pageFaultIRQ)
            # una vez resuelto el pageFault, volvemos a buscar en la Page Table
            # ya que la pagina, ahora debe estar cargada si o si
            frameId = self._tlb[pageId][0]

        ### setear los flags manejados por el MMU para los algoritmos de seleccion de victima

        #
        ##calculamos la direccion fisica resultante
        frameBaseDir = self._frameSize * frameId
        physicalAddress = frameBaseDir + offset

        log.logger.info("PhysicalAddres={physicalAddr}".format(physicalAddr=physicalAddress))

        # obtenemos la instrucción alocada en esa direccion
        return self._memory.read(physicalAddress)

    def getInstructionsOfFrame(self, aFrameID):
        instructionsOfTheFrame = []
        frameBaseDir = aFrameID * self._frameSize
        for offset in range(0, self._frameSize):
            instr = self._memory.read(frameBaseDir + offset)
            instructionsOfTheFrame.append(instr)

        return instructionsOfTheFrame


## emulates the main Central Processor Unit
class Cpu():

    def __init__(self, mmu, interruptVector):
        self._mmu = mmu
        self._interruptVector = interruptVector
        self._pc = -1
        self._ir = None

    def tick(self, tickNbr):
        if (self.isBusy()):
            self._fetch()
            self._decode()
            self._execute()
        else:
            log.logger.info("cpu - NOOP")

    def _fetch(self):
        self._ir = self._mmu.fetch(self._pc)
        self._pc += 1

    def _decode(self):
        ## decode no hace nada en este caso
        pass

    def _execute(self):
        if ASM.isEXIT(self._ir):
            killIRQ = IRQ(KILL_INTERRUPTION_TYPE)
            self._interruptVector.handle(killIRQ)
        elif ASM.isIO(self._ir):
            ioInIRQ = IRQ(IO_IN_INTERRUPTION_TYPE, self._ir)
            self._interruptVector.handle(ioInIRQ)
        elif ASM.isCPU_WRITE(self._ir):
            cpuWriteIRQ = IRQ(CPU_WRITE_INTERRUPTION_TYPE,self._pc)
            self._interruptVector.handle(cpuWriteIRQ)
        else:
            log.logger.info("cpu - Exec: {instr}, PC={pc}".format(instr=self._ir, pc=self._pc))

    def isBusy(self):
        return self._pc > -1

    @property
    def pc(self):
        return self._pc

    @pc.setter
    def pc(self, addr):
        self._pc = addr

    def __repr__(self):
        return "CPU(PC={pc})".format(pc=self._pc)


## emulates an Input/output device of the Hardware
class AbstractIODevice():

    def __init__(self, deviceId, deviceTime):
        self._deviceId = deviceId
        self._deviceTime = deviceTime
        self._busy = False

    @property
    def deviceId(self):
        return self._deviceId

    @property
    def is_busy(self):
        return self._busy

    @property
    def is_idle(self):
        return not self._busy

    ## executes an I/O instruction
    def execute(self, operation):
        if (self._busy):
            raise Exception(
                "Device {id} is busy, can't  execute operation: {op}".format(id=self.deviceId, op=operation))
        else:
            self._busy = True
            self._ticksCount = 0
            self._operation = operation

    def tick(self, tickNbr):
        if (self._busy):
            self._ticksCount += 1
            if (self._ticksCount > self._deviceTime):
                ## operation execution has finished
                self._busy = False
                ioOutIRQ = IRQ(IO_OUT_INTERRUPTION_TYPE, self._deviceId)
                HARDWARE.interruptVector.handle(ioOutIRQ)
            else:
                log.logger.info("device {deviceId} - Busy: {ticksCount} of {deviceTime}".format(deviceId=self.deviceId,
                                                                                                ticksCount=self._ticksCount,
                                                                                                deviceTime=self._deviceTime))


class PrinterIODevice(AbstractIODevice):
    def __init__(self):
        super(PrinterIODevice, self).__init__("Printer", 3)


class Timer:

    def __init__(self, cpu, interruptVector):
        self._cpu = cpu
        self._interruptVector = interruptVector
        self._tickCount = 0  # cantidad de de ciclos “ejecutados” por el proceso actual
        self._active = False  # por default esta desactivado
        self._quantum = 0  # por default esta desactivado

    def tick(self, tickNbr):
        # registro que el proceso en CPU corrio un ciclo mas 
        self._tickCount += 1
        if self._active and (self._tickCount > self._quantum) and self._cpu.isBusy():
            # se “cumplio” el limite de ejecuciones
            timeoutIRQ = IRQ(TIMEOUT_INTERRUPTION_TYPE)
            self._interruptVector.handle(timeoutIRQ)
        else:
            self._cpu.tick(tickNbr)

    def reset(self):
        self._tickCount = 0

    @property
    def quantum(self):
        return self._quantum

    @quantum.setter
    def quantum(self, quantum):
        self._active = True
        self._quantum = quantum


## emulates the Hardware that were the Operative System run
class Hardware():

    ## Setup our hardware
    def setup(self, memorySize, frameSize):
        ## add the components to the "motherboard"
        self._memory = Memory(memorySize)
        self._interruptVector = InterruptVector()
        self._clock = Clock()
        self._ioDevice = PrinterIODevice()
        self._mmu = MMU(self._memory)
        self.mmu.frameSize = frameSize
        self._cpu = Cpu(self._mmu, self._interruptVector)
        self._timer = Timer(self._cpu, self._interruptVector)
        self._clock.addSubscriber(self._ioDevice)
        self._clock.addSubscriber(self._timer)

    def switchOn(self):
        log.logger.info(" ---- SWITCH ON ---- ")
        return self.clock.start()

    def switchOff(self):
        self.clock.stop()
        log.logger.info(" ---- SWITCH OFF ---- ")

    @property
    def cpu(self):
        return self._cpu

    @property
    def clock(self):
        return self._clock

    @property
    def interruptVector(self):
        return self._interruptVector

    @property
    def memory(self):
        return self._memory

    @property
    def mmu(self):
        return self._mmu

    @property
    def ioDevice(self):
        return self._ioDevice

    @property
    def timer(self):
        return self._timer

    def __repr__(self):
        return "HARDWARE state {cpu}\n{mem}".format(cpu=self._cpu, mem=self._memory)


### HARDWARE is a global variable
### can be access from any
HARDWARE = Hardware()
