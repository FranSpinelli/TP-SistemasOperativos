#!/usr/bin/env python

from hardware import *
import log


## emulates a compiled program
class Program():

    def __init__(self, name, instructions):
        self._name = name
        self._instructions = self.expand(instructions)

    @property
    def name(self):
        return self._name

    @property
    def instructions(self):
        return self._instructions

    def addInstr(self, instruction):
        self._instructions.append(instruction)

    def expand(self, instructions):
        expanded = []
        for i in instructions:
            if isinstance(i, list):
                ## is a list of instructions
                expanded.extend(i)
            else:
                ## a single instr (a String)
                expanded.append(i)

        ## now test if last instruction is EXIT
        ## if not... add an EXIT as final instruction
        last = expanded[-1]
        if not ASM.isEXIT(last):
            expanded.append(INSTRUCTION_EXIT)

        return expanded

    def __repr__(self):
        return "Program({name}, {instructions})".format(name=self._name, instructions=self._instructions)


## emulates an Input/Output device controller (driver)
class IoDeviceController():

    def __init__(self, device):
        self._device = device
        self._waiting_queue = []
        self._currentPCB = None

    def runOperation(self, pcb, instruction):
        pair = {'pcb': pcb, 'instruction': instruction}

        # append: adds the element at the end of the queue
        self._waiting_queue.append(pair)
        # try to send the instruction to hardware's device (if is idle)
        self.__load_from_waiting_queue_if_apply()

    def getFinishedPCB(self):
        finishedPCB = self._currentPCB
        self._currentPCB = None
        self.__load_from_waiting_queue_if_apply()
        return finishedPCB

    def __load_from_waiting_queue_if_apply(self):
        if (len(self._waiting_queue) > 0) and self._device.is_idle:
            ## pop(): extracts (deletes and return) the first element in queue
            pair = self._waiting_queue.pop(0)
            # print(pair)
            pcb = pair['pcb']
            instruction = pair['instruction']
            self._currentPCB = pcb
            self._device.execute(instruction)

    def __repr__(self):
        return "IoDeviceController for {deviceID} running: {currentPCB} waiting: {waiting_queue}".format(
            deviceID=self._device.deviceId, currentPCB=self._currentPCB, waiting_queue=self._waiting_queue)


## emulates the  Interruptions Handlers
class AbstractInterruptionHandler():
    def __init__(self, kernel):
        self._kernel = kernel

    @property
    def kernel(self):
        return self._kernel

    def execute(self, irq):
        log.logger.error("-- EXECUTE MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def pcbIn(self, pcb):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher
        scheduler = self.kernel.scheduler

        if not pcbTable.runningPCB:
            pcb.state = RUNNING_PCB_STATE
            pcbTable.runningPCB = pcb

            dispatcher.load_pcb(pcb, self.kernel.memoryManager.getPageTable(pcb.pid))

        else:
            if scheduler.mustExpropiate(pcb, pcbTable.runningPCB):
                pcbRunning = pcbTable.runningPCB
                pcbTable.runningPCB = None
                dispatcher.save_pcb(pcbRunning)
                pcbRunning.state = READY_PCB_STATE
                scheduler.addToReadyQueue(pcbRunning)

                dispatcher.load_pcb(pcb, self.kernel.memoryManager.getPageTable(pcb.pid))
                pcbTable.runningPCB = pcb
                pcb.state = RUNNING_PCB_STATE
            else:
                pcb.state = READY_PCB_STATE
                scheduler.addToReadyQueue(pcb)

    def pcbOut(self):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher
        scheduler = self.kernel.scheduler

        if not scheduler.readyQueueIsEmpty():
            pcbAEjecutar = scheduler.getPcb()
            pcbAEjecutar.state = RUNNING_PCB_STATE
            dispatcher.load_pcb(pcbAEjecutar, self.kernel.memoryManager.getPageTable(pcbAEjecutar.pid))
            pcbTable.runningPCB = pcbAEjecutar

    def quitPCBFromRunningAndChangeStateTo(self, state):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher

        pcb = pcbTable.runningPCB
        pcbTable.runningPCB = None
        dispatcher.save_pcb(pcb)

        pcb.state = state


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        self.quitPCBFromRunningAndChangeStateTo(TERMINATED_PCB_STATE)

        self.pcbOut()


class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        operation = irq.parameters
        pcb = pcbTable.runningPCB

        self.quitPCBFromRunningAndChangeStateTo(WAITING_PCB_STATE)

        self.kernel.ioDeviceController.runOperation(pcb, operation)
        log.logger.info(self.kernel.ioDeviceController)

        self.pcbOut()


class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.ioDeviceController.getFinishedPCB()
        self.pcbIn(pcb)


class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pair = irq.parameters
        path = pair[0]
        priority = pair[1]

        pcbTable = self.kernel.pcbTable

        pid = pcbTable.nuevoPid
        pcb = Pcb(pid, 0, path, priority)
        pcbTable.add_pcb(pcb)

        self.kernel.loader.loadNewPageTableOf(pcb)

        self.pcbIn(pcb)

        log.logger.info("\n Executing program: {name}".format(name=pcb.path))
        log.logger.info(HARDWARE)


class TimeOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher
        scheduler = self.kernel.scheduler

        if not scheduler.readyQueueIsEmpty():
            pcb = pcbTable.runningPCB
            pcbTable.runningPCB = None
            dispatcher.save_pcb(pcb)
            pcb.state = WAITING_PCB_STATE
            scheduler.addToReadyQueue(pcb)

            pcbAEjecutar = scheduler.getPcb()
            pcbAEjecutar.state = RUNNING_PCB_STATE
            dispatcher.load_pcb(pcbAEjecutar, self.kernel.memoryManager.getPageTable(pcbAEjecutar.pid))
            pcbTable.runningPCB = pcbAEjecutar
        else:
            HARDWARE.timer.reset()

class PageFaultInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pageIDToPutInMemory = irq.parameters
        pcb = self.kernel.pcbTable.runningPCB

        self.kernel.loader.loadPageOfPCB(pageIDToPutInMemory, pcb)

# emulates the core of an Operative System
class Kernel():

    def __init__(self, schedulerToUse, algorithmToUse):
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        timeOutHandler = TimeOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(TIMEOUT_INTERRUPTION_TYPE, timeOutHandler)

        pageFaultHandler = PageFaultInterruptionHandler(self)
        HARDWARE.interruptVector.register(PAGE_FAULT_INTERRUPTION_TYPE, pageFaultHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        ## driver for interaction with cpu and mmu
        self._dispatcher = Dispatcher(HARDWARE.cpu, HARDWARE.mmu)

        ##specific component to load programs in main memmory
        self._loader = Loader(self)

        ##helps kernel to keep process state
        self._pcbTable = PcbTable()

        ##Specific component to organize the logic mem
        self._memoryManager = MemoryManager(self, algorithmToUse)

        ##specific component that simulates a fileSystem very simply
        self._fileSystem = FileSystem(self)

        ##the scheduler with which the kernel will work
        self._scheduler = schedulerToUse
        self.scheduler.setUpTimer()  # set up the timer of the system with the quantum that the scheduler has

    @property
    def fileSystem(self):
        return self._fileSystem

    @property
    def memoryManager(self):
        return self._memoryManager

    @property
    def scheduler(self):
        return self._scheduler

    @property
    def pcbTable(self):
        return self._pcbTable

    @property
    def loader(self):
        return self._loader

    @property
    def dispatcher(self):
        return self._dispatcher

    @property
    def ioDeviceController(self):
        return self._ioDeviceController

    ## emulates a "system call" for programs execution
    def run(self, path, priority):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, (path, priority))
        HARDWARE._interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "


# estos son ls pocibles estados de un pcb
RUNNING_PCB_STATE = "#running"
WAITING_PCB_STATE = "#waiting"
READY_PCB_STATE = "#ready"
TERMINATED_PCB_STATE = "#terminated"


class Pcb():

    def __init__(self, pid, unPC, unPath, priority):
        self._pid = pid
        self._pc = unPC
        self._path = unPath
        self._state = READY_PCB_STATE
        self._priority = priority

    @property
    def priority(self):
        return self._priority

    @property
    def pid(self):
        return self._pid

    @property
    def pc(self):
        return self._pc

    @property
    def path(self):
        return self._path

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, nuevoState):
        self._state = nuevoState

    @pc.setter
    def pc(self, nuevoPC):
        self._pc = nuevoPC

    def __repr__(self):
        return "Pid: {} - Name: {} - PC: {} - priority: {}".format(self.pid, self.path, self.pc, self.priority)


class PcbTable():

    def __init__(self):
        self._pidActual = 0
        self._pcbTable = []
        self.runningPCB = None

    @property
    def nuevoPid(self):
        valorARetornar = self._pidActual
        self._pidActual = self._pidActual + 1
        return valorARetornar

    @property
    def pcbTable(self):
        return self._pcbTable

    def get_pcb(self, pid):
        return self._pcbTable[pid]

    def add_pcb(self, pcb):
        self._pcbTable.append(pcb)

    def remove_pcb(self, pid):
        new_list = []
        for pcb in self._pcbTable:
            if pcb.pid != pid:
                new_list.append(pcb)
        self._pcbTable = new_list

    @property
    def runningPCB(self):
        return self._runningPCB

    @runningPCB.setter
    def runningPCB(self, nuevorunningpcb):
        self._runningPCB = nuevorunningpcb


class Dispatcher():

    def __init__(self, unCPU, unMMu):
        self._cpu = unCPU
        self._mmu = unMMu

    def load_pcb(self, pcb, pCBpageTable):
        self._cpu.pc = pcb.pc
        self.__loadPageTableInTLB(pCBpageTable)
        HARDWARE.timer.reset()

    def __loadPageTableInTLB(self, pageTable):
        HARDWARE.mmu.resetTLB()
        HARDWARE.mmu.tlb = pageTable

    def save_pcb(self, pcb):
        pcb.pc = self._cpu.pc
        self._cpu.pc = -1


class Loader():

    def __init__(self, kernel):
        self._kernel = kernel

    def loadPageOfPCB(self, pageIDToPutInMemory, pcb):
        pcbProgram = self._kernel.fileSystem.read(pcb.path)
        instructionsOfPageWithID = self.__instructionsOfPage(pageIDToPutInMemory, pcbProgram)
        framesToUseID = self._kernel.memoryManager.allocFrames(1)

        frameOfPage = None
        for instructionNumber in range(0, len(instructionsOfPageWithID)):
            ##calculamos la pagina y el offset correspondiente a la instruccion correspondiente
            pageId = instructionNumber // HARDWARE.mmu.frameSize
            offset = instructionNumber % HARDWARE.mmu.frameSize

            ##buscamos el frame correspondiente a la pagina calculada para dicha pagina
            try:
                frameId = framesToUseID[pageId]
            except:
                raise Exception("ERROR \n algo salio mal en el loader")

            ##calculamos la direccion fisica resultante
            frameBaseDir = HARDWARE.mmu.frameSize * frameId
            physicalAddress = frameBaseDir + offset

            frameOfPage = frameId

            HARDWARE.memory.write(physicalAddress, instructionsOfPageWithID[instructionNumber])

        self._kernel.memoryManager.completePageTableOfWith(pcb.pid, pageIDToPutInMemory, frameOfPage)

    def loadNewPageTableOf(self, pcb):
        self._kernel.memoryManager.putPageTable(pcb.pid,  dict())

    def __instructionsOfPage(self, pageIDToPutInMemory, pcbProgram):
        instructions = pcbProgram.instructions
        instructionsOfPage = []

        for instructionNumber in range(0, len(instructions)):
            if instructionNumber // HARDWARE.mmu.frameSize == pageIDToPutInMemory:
                instructionsOfPage.append(instructions[instructionNumber])

        return instructionsOfPage

    def __framesNeeded(self, programSize):
        result = (programSize // HARDWARE.mmu.frameSize)

        if programSize % HARDWARE.mmu.frameSize > 0:
            result += 1

        return result

class AbstractScheduler:

    def __init__(self):
        self._readyQueue = []

    def setUpTimer(self):
        log.logger.error(
            "-- setUpTimer MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def addToReadyQueue(self, pcb):
        log.logger.error(
            "-- addToReadyQueue MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def readyQueueIsEmpty(self):
        log.logger.error(
            "-- readyQueueIsEmpty MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def getPcb(self):
        log.logger.error("-- getPcb MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

    def mustExpropiate(self, pcbInCPU, pcbToAdd):
        return False


class RoundRobinScheduler(AbstractScheduler):

    def __init__(self, quantum):
        self._quantum = quantum
        super(RoundRobinScheduler, self).__init__()

    def setUpTimer(self):
        HARDWARE.timer.quantum = self._quantum

    def addToReadyQueue(self, pcb):
        self._readyQueue.append(pcb)

    def readyQueueIsEmpty(self):
        return not self._readyQueue

    def getPcb(self):
        return self._readyQueue.pop(0)


class FCFSScheduler(AbstractScheduler):

    def setUpTimer(self):
        pass

    def addToReadyQueue(self, pcb):
        self._readyQueue.append(pcb)

    def readyQueueIsEmpty(self):
        return not self._readyQueue

    def getPcb(self):
        return self._readyQueue.pop(0)


class PriorityNoPreemtiveScheduler(AbstractScheduler):

    def setUpTimer(self):
        pass

    def addToReadyQueue(self, pcb):
        if len(self._readyQueue) == 0:
            self._readyQueue.append(pcb)
        else:
            if len(self._readyQueue) == 1:
                if self.hasHigherPriority(pcb, self._readyQueue[0]):
                    self._readyQueue.insert(0, pcb)
                else:
                    self._readyQueue.append(pcb)
            else:
                pcbsHigherPriority = []
                while (len(self._readyQueue) > 0) and (self.hasHigherPriority(self._readyQueue[0], pcb)):
                    pcbsHigherPriority.append(self._readyQueue.pop(0))
                (pcbsHigherPriority.append(pcb)).extend(self._readyQueue)

    def hasHigherPriority(self, pcb1, pcb2):
        return pcb1.priority < pcb2.priority

    def readyQueueIsEmpty(self):
        return not self._readyQueue

    def getPcb(self):
        return self._readyQueue.pop(0)


class PriorityPreemtiveScheduler(PriorityNoPreemtiveScheduler):

    def mustExpropiate(self, pcbToAdd, pcbInCPU):
        return self.hasHigherPriority(pcbToAdd, pcbInCPU)

class VictimSelectionAlgorithmAbstract:

    def __init__(self):
        self._pageTable = dict()

    def putPageTable(self, pid, pageTable):
            self._pageTable[pid] = pageTable

    def getPageTable(self, pid):
        if pid in self._pageTable:
            return self._pageTable[pid]
        else:
            raise Exception("\n*\n* ERROR \n*\n Error en el Memory Manager \nNo se cargo el proceso  {pid}".format(
                pid=str(pid)))

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameOfPage):
        pageTable = self._pageTable[pid]
        pageTable[pageIDToPutInMemory] = frameOfPage

    def selectVictimUsing(self):
        log.logger.error(
            "-- selectVictimUsing MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

class FifoAlgorithm(VictimSelectionAlgorithmAbstract):

    def __init__(self):
        super(FifoAlgorithm, self).__init__()
        self._victimQueue = []

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameIDOfPage):
        super(FifoAlgorithm, self).completePageTableOfWith(pid, pageIDToPutInMemory, frameIDOfPage)
        self._victimQueue.append(frameIDOfPage)

    def selectVictimUsing(self):
        return self._victimQueue.pop(0)

class MemoryManager:

    def __init__(self, kernel, algorithmToUse):
        self._kernel = kernel
        self._VSA = algorithmToUse # VSA = Victim Selection Algorithm
        self._freeFrames = list(range(HARDWARE.memory.size // HARDWARE.mmu.frameSize))

    @property
    def freeFrames(self):
        return self._freeFrames

    def putPageTable(self, pid, pageTable):
            #self._pageTable[pid] = pageTable
            self._VSA.putPageTable(pid, pageTable)

    def getPageTable(self, pid):

        #if pid in self._pageTable:
        #    return self._pageTable[pid]
        #else:
        #    raise Exception("\n*\n* ERROR \n*\n Error en el Memory Manager \nNo se cargo el proceso  {pid}".format(
        #        pid=str(pid)))
        return self._VSA.getPageTable(pid)

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameOfPage):

        #pageTable = self._pageTable[pid]
        #pageTable[pageIDToPutInMemory] = frameOfPage
        self._VSA.completePageTableOfWith(pid, pageIDToPutInMemory, frameOfPage)

    def NumberOfFreeMemCells(self):
        return len(self._freeFrames) * HARDWARE.mmu.frameSize

    def allocFrames(self, cantOfFramesRequired):

        if cantOfFramesRequired > len(self.freeFrames):
            #raise Exception("\n*\n* ERROR \n*\n  Out of memory Exception \nCantidad de frames disponibles: {freeFrames}"
            #                "\nCantidad de frames solicitados: {framesRequired}"
            #                .format(freeFrames=len(self._freeFrames), framesRequired=cantOfFramesRequired))

            victimFrameToSWAP = self._VSA.selectVictimUsing()
            instructionsOfTheFrame = HARDWARE.mmu.getInstructionsOfFrame(victimFrameToSWAP)
            self._kernel.fileSystem.swap(victimFrameToSWAP, instructionsOfTheFrame)#pongo la pag en la memoria virtual
            self.freeTheFrames(victimFrameToSWAP)#libero el frame de la pag que fue puesta en memoria virtual(ya puede ser usada)
            return self._getFreeFrames(cantOfFramesRequired)

        # no tirar error, hay que seleccionar victima y hacer el SWAP, deberia tener un OBJ con el
        # algoritmo que se encargue de la seleccion
        else:
            #listWithFreeFrames = self._freeFrames[:cantOfFramesRequired]
            #self._freeFrames = self._freeFrames[cantOfFramesRequired:]
            #return listWithFreeFrames

            return self._getFreeFrames(cantOfFramesRequired)

    def _getFreeFrames(self,cantOfFramesRequired):

        listWithFreeFrames = self._freeFrames[:cantOfFramesRequired]
        self._freeFrames = self._freeFrames[cantOfFramesRequired:]

        return listWithFreeFrames

    def freeTheFrames(self, releasedFrames):
        self._freeFrames.append(releasedFrames)


class FileSystem:

    def __init__(self, kernel):
        self._fileSystem = dict()
        self._virtualMem = VirtualMemory(HARDWARE.memory.size // HARDWARE.mmu.frameSize, kernel)

    def write(self, path, program):
        self._fileSystem[path] = program

    def read(self, path):
        if path in self._fileSystem:
            return self._fileSystem[path]
        else:
            raise Exception("-- the path does not exist.")

    def swap(self, aVictimFrameToSwap, aListOfInstructions):
        self._virtualMem.savePage((aVictimFrameToSwap, aListOfInstructions))

class VirtualMemory:

    def __init__(self, aNumberOfFrames, aKernel):
        self._kernel = aKernel
        self._virtualMemory = dict()
        for x in range(0, aNumberOfFrames):
            self._virtualMemory[x] = ""

    def savePage(self, aPair):
        key = self._firstEmptyKey()
        self._virtualMemory[key] = aPair
        return key

    def _firstEmptyKey(self):
        for key in self._virtualMemory:
            if self._virtualMemory[key] == "":
                return key
        raise Exception("\nLa Memoria Virtual esta llena")


# -----------------------------------Graficador de diagrama de gant-----------------------------------------------------

class GantGraficator():

    def __init__(self, kernel):
        self._kernel = kernel
        self._representacionDeEjecucion = []
        self._headers = ["procesos"]

    def tick(self, ticknmbr):
        todosEnDead = True
        if ticknmbr == 1:
            self._representacionDeEjecucion = self.armarCuadro(self._kernel.pcbTable.pcbTable)
            todosEnDead = self.actualizarRepresentacion(ticknmbr, self._kernel.pcbTable.pcbTable)

        if ticknmbr > 1:
            todosEnDead = self.actualizarRepresentacion(ticknmbr, self._kernel.pcbTable.pcbTable)

        if todosEnDead:
            log.logger.info(self.__repr__())
            HARDWARE.switchOff()

    def actualizarRepresentacion(self, ticknmbr, pcbTable):
        self._headers.append(ticknmbr)

        todosEnDead = True
        nroDeProceso = 0
        for pcb in pcbTable:
            valorARetornar = "D"

            if pcb.state == RUNNING_PCB_STATE:
                valorARetornar = "*"
                todosEnDead = False
            if pcb.state == READY_PCB_STATE:
                valorARetornar = "R"
                todosEnDead = False
            if pcb.state == WAITING_PCB_STATE:
                valorARetornar = "W"
                todosEnDead = False

            self._representacionDeEjecucion[nroDeProceso].append(valorARetornar)
            nroDeProceso = nroDeProceso + 1

        return todosEnDead

    def armarCuadro(self, pcbTable):
        listaAEntregar = []
        for pcb in pcbTable:
            listaAEntregar.append([pcb.path])
        return listaAEntregar

    def __repr__(self):
        return tabulate(self._representacionDeEjecucion, headers=self._headers, tablefmt='grid', stralign='center')
