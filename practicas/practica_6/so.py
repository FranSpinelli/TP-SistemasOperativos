from hardware import *
import log


## emulates a compiled program
class Program:

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
        log.logger.info(HARDWARE)
        log.logger.info(self.kernel.fileSystem.virtualMemory())

class CpuWriteInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcb = self.kernel.pcbTable.runningPCB
        dataToWrite = irq.parameters

        firstFreeStackId = self.kernel.memoryManager.firstFreeStackId(pcb)

        if firstFreeStackId is None:
            log.logger.info("no habia stack creado")
            self.kernel.loader.loadNewStack(pcb, dataToWrite)

        else:
            log.logger.info("habia stack creado")
            pageTable = self.kernel.memoryManager.getPageTable(pcb.pid)
            tuple = pageTable[firstFreeStackId]

            self.kernel.loader.addNewDataToTheStack(dataToWrite, tuple[0])


        log.logger.info(HARDWARE)
        log.logger.info(self.kernel.fileSystem.virtualMemory())

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

        cpuWriteHandler = CpuWriteInterruptionHandler(self)
        HARDWARE.interruptVector.register(CPU_WRITE_INTERRUPTION_TYPE, cpuWriteHandler)

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
        self._stacks = 0

    @property
    def stacks(self):
        return self.stacks

    def getStackID(self):
        respsta = self._stacks
        self._stacks += 1
        return respsta


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
        frameToUseID = self._kernel.memoryManager.allocFrames(1)[0]

        if self._kernel.memoryManager.isPageOfPCBInVirtualMemory(pageIDToPutInMemory, pcb.pid):
            pageTable = self._kernel.memoryManager.getPageTable(pcb.pid)

            locationInVirtualMem = pageTable[pageIDToPutInMemory] % 10
            tupleWithInstructionsAndState = self._kernel.fileSystem.swapIn(locationInVirtualMem)

            self._kernel.loader.loadInstructionsUsingFrame(tupleWithInstructionsAndState[0], frameToUseID)
            self._kernel.memoryManager.completePageTableOfWith(pcb.pid, pageIDToPutInMemory, frameToUseID, tupleWithInstructionsAndState[1])
        else:
            self.loadInstructionsUsingFrame(instructionsOfPageWithID, frameToUseID)
            self._kernel.memoryManager.completePageTableOfWith(pcb.pid, pageIDToPutInMemory, frameToUseID, False)

    def loadStackOfPCBFromVirtualMem(self, pageID, pcb):
        #calculo la tupla instr-estado de una pagina que esta en la virtual mem, y la elimino de esta
        tuple = self._kernel.fileSystem.swapIn(pageID % 10)

        # tomo un frame para poder guardar las instrucciones
        frameToUseID = self._kernel.memoryManager.allocFrames(1)[0]

        pageAndPID = self._kernel.memoryManager.pageIdAndPIDOf(frameToUseID)

        log.logger.info("pagina del frame a sacar= {a}".format(a=pageAndPID[0]))

        if int(str(pageAndPID[0])[:3]) == 999: #es un stack y hay que persistirlo
            log.logger.info("es un stack y hay que swapear")

            pageTable = self._kernel.memoryManager.getPageTable(pageAndPID[1])
            state = pageTable[pageAndPID[0]][1]
            instructionsOfTheFrame = HARDWARE.mmu.getInstructionsOfFrame(frameToUseID)

            nuevaFrameID = self._kernel.fileSystem.swapOut((instructionsOfTheFrame, state[1]))
            self._kernel.memoryManager.completePageTableOfWith(pageAndPID[1], pageAndPID[0], nuevaFrameID, state[1])

        #pongo la data en la mem, actualizo la informacion de la page table
        self._kernel.loader.loadInstructionsUsingFrame(tuple[0], frameToUseID)
        self._kernel.memoryManager.completePageTableOfWith(pcb.pid, pageID, frameToUseID, tuple[1])

        #libero la virtual mem
        self._kernel.fileSystem._virtualMem.removePage(pageID % 10)

    def loadNewStack(self, pcb, data):

        id = pcb.getStackID()
        idToUse = int(str(999) + str(id))

        frameToUse = self._kernel.memoryManager.allocFrames(1)[0]
        self.loadInstructionsUsingFrame([data], frameToUse)
        self._kernel.memoryManager.completePageTableOfWith(pcb.pid, idToUse, frameToUse, True)

    def addNewDataToTheStack(self, dataToWrite, frame):

        instructions = HARDWARE.mmu.getInstructionsOfFrame(frame)
        instruccionesFinales = []

        for instruct in instructions:
            if instruct != "":
                instruccionesFinales.append(instruct)

        instruccionesFinales.append(dataToWrite)

        self.loadInstructionsUsingFrame(instruccionesFinales, frame)

    def loadInstructionsUsingFrame(self, instructions, aFrame):

        while len(instructions) < 4:
            instructions.append("")

        for instructionNumber in range(0, len(instructions)):
            ##calculamos el offset correspondiente a la instruccion correspondiente
            offset = instructionNumber % HARDWARE.mmu.frameSize

            ##calculamos la direccion fisica resultante
            frameBaseDir = HARDWARE.mmu.frameSize * aFrame
            physicalAddress = frameBaseDir + offset

            HARDWARE.memory.write(physicalAddress, instructions[instructionNumber])

    def loadNewPageTableOf(self, pcb):
        self._kernel.memoryManager.putPageTable(pcb.pid, dict())

    def __instructionsOfPage(self, pageIDToPutInMemory, pcbProgram):
        instructions = pcbProgram.instructions
        instructionsOfPage = []

        for instructionNumber in range(0, len(instructions)):
            if instructionNumber // HARDWARE.mmu.frameSize == pageIDToPutInMemory:
                instructionsOfPage.append(instructions[instructionNumber])

        return instructionsOfPage

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

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameOfPage, aBoolean):
        pageTable = self._pageTable[pid]
        pageTable[pageIDToPutInMemory] = (frameOfPage, aBoolean)# the second value indicates if in the frame was a CPU_WRITE instruction

    def isPageOfPCBInVirtualMemory(self, aPageID, aPID):
        pageTable = self.getPageTable(aPID)
        try:
            frameOfPageID = pageTable[aPageID][0]
        except:
            return False
        return 111 == int(str(frameOfPageID)[:3])

    def getPageAndPIDOf(self, aFrame):

        for pid in self._pageTable.keys():
            pageTable = self._pageTable[pid]
            for pag in pageTable.keys():
                if pageTable[pag][0] == aFrame:
                    return (pag, pid)
        log.logger.info("no se esta usando el frame: {frame}".format(frame = aFrame))

    def firstFreeStackId(self, pcb, kernel):
        aFileSystemToWorkWith = kernel.fileSystem

        pageTable = self.getPageTable(pcb.pid)

        for pageID in pageTable.keys():

            log.logger.info("pageID={a}".format(a = pageID))

            if int(str(pageID)[:3]) == 999: #quiere decir que es un stack
                log.logger.info("es un stack")
                tuple = pageTable[pageID]
                log.logger.info("tuple={a}".format(a=tuple[0]))
                if int(str(tuple[0])[:3]) == 111: #quiere decir que el stack esta en la virtual mem
                    log.logger.info("que esta en virtual mem")

                    if aFileSystemToWorkWith.virtualMem.numberOfCellsUsedOfFrame(tuple[0] % 10) < 4: #quiere decir que el stack de la virtual mem se puede seguir usando
                        #len(aFileSystemToWorkWith.virtualMem().getPage(tuple[0] % 10)[0]) < 4:

                        log.logger.info("y se puede seguir usando")
                        # carguo el stack que puede ser usado en la memoria principal y retorno la pageID para que lo utilize el handler


                        kernel.loader.loadStackOfPCBFromVirtualMem(pageID, pcb)

                        return pageID

                else:#no esta en virtualMem
                    log.logger.info("que esta en principal mem")

                    if HARDWARE.mmu.numberOfcellsUsedOfFrame(tuple[0]) < 4:
                        log.logger.info("y se puede seguir usando")
                        return pageID
        log.logger.info("no encontro nada")
        return None

    def selectVictim(self):
        log.logger.error(
            "-- selectVictim MUST BE OVERRIDEN in class {classname}".format(classname=self.__class__.__name__))

class FifoAlgorithm(VictimSelectionAlgorithmAbstract):

    def __init__(self):
        super(FifoAlgorithm, self).__init__()
        self._victimQueue = []

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameIDOfPage, aBoolean):
        super(FifoAlgorithm, self).completePageTableOfWith(pid, pageIDToPutInMemory, frameIDOfPage, aBoolean)

        if (self._victimQueue[-1:] != [frameIDOfPage]) & (int(str(frameIDOfPage)[:3]) != 111):
            self._victimQueue.append(frameIDOfPage)
            log.logger.info("victim Queue state= {victimQueue}".format(victimQueue=self._victimQueue))


    def selectVictim(self):
        return self._victimQueue.pop(0)


class LRUAlgorithm(VictimSelectionAlgorithmAbstract):

    def __init__(self):
        super(LRUAlgorithm, self).__init__()
        self._victimQueue = []

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameIDOfPage, aBoolean):
        super(LRUAlgorithm, self).completePageTableOfWith(pid, pageIDToPutInMemory, frameIDOfPage, aBoolean)
        if frameIDOfPage in self._victimQueue:
            self._victimQueue.remove(frameIDOfPage)
            self._victimQueue.append(frameIDOfPage)
            log.logger.info("victim Queue state= {victimQueue}".format(victimQueue= self._victimQueue))

        elif (self._victimQueue[-1:] != [frameIDOfPage]) & (int(str(frameIDOfPage)[:3]) != 111):
            self._victimQueue.append(frameIDOfPage)
            log.logger.info("victim Queue state= {victimQueue}".format(victimQueue=self._victimQueue))

    def selectVictim(self):
        victim = self._victimQueue.pop(0)
        return victim


class SecondOportunityAlgorithm(VictimSelectionAlgorithmAbstract):

    def __init__(self):
        super(SecondOportunityAlgorithm, self).__init__()
        self._aguja = None

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameOfPage, aBoolean):
        super(SecondOportunityAlgorithm,self).completePageTableOfWith(pid, pageIDToPutInMemory, frameOfPage, aBoolean)

        if self._aguja is None:
            self._aguja = (pid, pageIDToPutInMemory)

    def selectVictim(self):
        victim = None
        while victim is None:
            pcbPageTable = self._pageTable[self._aguja[0]]
            tupleWithAguja = pcbPageTable[self._aguja[1]]

            log.logger.info("process id de aguja={physicalAddr}".format(physicalAddr=self._aguja[0]))
            log.logger.info("pagina de la aguja ={physicalAddr}".format(physicalAddr=self._aguja[1]))
            log.logger.info("tupla=(FrameID: {p1}, Fue escrito: {p2})".format(p1=tupleWithAguja[0], p2=tupleWithAguja[1]))

            if tupleWithAguja[1]:
                self.completePageTableOfWith(self._aguja[0], self._aguja[1], tupleWithAguja[0], False)
            else:
                victim = tupleWithAguja[0]
            self._aguja = self._calculateNextAgujaPosition()
        return victim

    def _calculateNextAgujaPosition(self):
        pID = self._aguja[0]
        pageID = self._aguja[1]

        pIDList = list(self._pageTable.keys())
        pagesIDList = list(self._pageTable[pID].keys())

        if len(pagesIDList) == pagesIDList.index(pageID) + 1:
            nuevoPageID = pagesIDList[0]
            #-----------------------------------------
            if len(pIDList) == pIDList.index(pID) + 1:
                nuevoPID = pIDList[0]
            else:
                nuevoPID = pIDList.index(pID) + 1
            #-------------------------------------------
        else:
            nuevoPageID = pagesIDList[pagesIDList.index(pageID)+1]
            nuevoPID = pID

        log.logger.info("nueva aguja=({p1}, {p2})".format(p1=nuevoPID, p2=nuevoPageID))
        return (nuevoPID, nuevoPageID)

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

        return self._VSA.getPageTable(pid)

    def completePageTableOfWith(self, pid, pageIDToPutInMemory, frameOfPage, aBoolean):

        self._VSA.completePageTableOfWith(pid, pageIDToPutInMemory, frameOfPage, aBoolean)

    def NumberOfFreeMemCells(self):
        return len(self._freeFrames) * HARDWARE.mmu.frameSize

    def allocFrames(self, cantOfFramesRequired):

        if cantOfFramesRequired > len(self.freeFrames):

            victimFrameToSWAP = self._VSA.selectVictim()
            instructionsOfTheFrame = HARDWARE.mmu.getInstructionsOfFrame(victimFrameToSWAP)

            pageAndPIDOfFrame = self._VSA.getPageAndPIDOf(victimFrameToSWAP)

            if int(str(pageAndPIDOfFrame[0])[:3]) == 999:

                estado = self._VSA.getPageTable(pageAndPIDOfFrame[1])[pageAndPIDOfFrame[0]][1]
                newPageLocation = self._kernel.fileSystem.swapOut((instructionsOfTheFrame, estado))#pongo la pag en la memoria virtual

                self._VSA.completePageTableOfWith(pageAndPIDOfFrame[1], pageAndPIDOfFrame[0], newPageLocation, estado)

            self.freeTheFrames(victimFrameToSWAP)#libero el frame de la pag que fue puesta en memoria virtual(ya puede ser usada)
            return self._getFreeFrames(cantOfFramesRequired)
        else:
            return self._getFreeFrames(cantOfFramesRequired)

    def isPageOfPCBInVirtualMemory(self, aPageID, aPID):
        return self._VSA.isPageOfPCBInVirtualMemory(aPageID, aPID)

    def _getFreeFrames(self,cantOfFramesRequired):

        listWithFreeFrames = self._freeFrames[:cantOfFramesRequired]
        self._freeFrames = self._freeFrames[cantOfFramesRequired:]

        return listWithFreeFrames

    def freeTheFrames(self, releasedFrames):
        self._freeFrames.append(releasedFrames)

    def firstFreeStackId(self, pcb):
        return self._VSA.firstFreeStackId(pcb, self._kernel)

    def pageIdAndPIDOf(self, aFrame):
        return self._VSA.getPageAndPIDOf(aFrame)
class FileSystem:

    def __init__(self, kernel):
        self._kernel = kernel
        self._fileSystem = dict()
        self._virtualMem = VirtualMemory(HARDWARE.memory.size // HARDWARE.mmu.frameSize, kernel)

    @property
    def virtualMem(self):
        return self._virtualMem

    def virtualMemory(self):
        return self._virtualMem

    def write(self, path, program):
        self._fileSystem[path] = program

    def read(self, path):
        if path in self._fileSystem:
            return self._fileSystem[path]
        else:
            raise Exception("-- the path does not exist.")

    def swapOut(self, aTuple):
        return self._virtualMem.savePage(aTuple)

    def swapIn(self, aLocationInVirtualMem):
        return self._virtualMem.getPage(aLocationInVirtualMem)

class VirtualMemory:

    def __init__(self, aNumberOfFrames, aKernel):
        self._kernel = aKernel
        self._virtualMemory = dict()
        for x in range(0, aNumberOfFrames):
            self._virtualMemory[x] = ("","")

    def savePage(self, aTuple):
        key = self._firstEmptyKey()
        self._virtualMemory[key] = aTuple
        return int(str(111)+str(key))#las pag que estan en la mem virtual tendran frames que comenzaran con "111"

    def getPage(self, aKey):
        return self._virtualMemory[aKey]

    def _firstEmptyKey(self):
        for key in self._virtualMemory:
            if (self._virtualMemory[key][0] == "") & (self._virtualMemory[key][1] == ""):
                return key
        raise Exception("\nLa Memoria Virtual esta llena")

    def numberOfCellsUsedOfFrame(self, aKey):
        instructionsOfPage = self.getPage(aKey)[0]
        contador = 0

        for instr in instructionsOfPage:
            if instr != "":
                contador += 1
        return contador

    def removePage(self, aKey):
        for key in self._virtualMemory:
            if key == aKey:
                self._virtualMemory[key] = ("","")

    def __repr__(self):

        lista = []
        for key in list(self._virtualMemory.keys()):
            tuple = self._virtualMemory[key]
            lista.append([key, tuple[0], tuple[1]])

        return tabulate(lista,headers=["key", "data", "state"], tablefmt='grid', stralign='center')
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
