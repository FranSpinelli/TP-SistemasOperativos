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

        if pcbTable.runningPCB is None:
            pcb.state = RUNNING_PCB_STATE
            pcbTable.runningPCB = pcb

            dispatcher.load_pcb(pcb)

        else:
            if scheduler.mustExpropiate(pcb, pcbTable.runningPCB):
                log.logger.info("Expropio: sale {} - entra {}".format(pcbTable.runningPCB, pcb))
                pcbRunning = pcbTable.runningPCB
                pcbTable.runningPCB = None
                dispatcher.save_pcb(pcbRunning)
                pcbRunning.state = READY_PCB_STATE
                scheduler.addToReadyQueue(pcbRunning)

                dispatcher.load_pcb(pcb)
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
            dispatcher.load_pcb(pcbAEjecutar)
            pcbTable.runningPCB = pcbAEjecutar


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher

        pcb = pcbTable.runningPCB
        dispatcher.save_pcb(pcb)
        pcb.state = TERMINATED_PCB_STATE

        #pcbTable.remove_pcb(pcb.pid)
        pcbTable.runningPCB = None

        self.pcbOut()

class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher

        operation = irq.parameters

        pcb = pcbTable.runningPCB
        pcbTable.runningPCB = None
        dispatcher.save_pcb(pcb)
        pcb.state = WAITING_PCB_STATE

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
        program = pair[0]
        priority = pair[1]

        pcbTable = self.kernel.pcbTable

        baseDir = self._kernel.loader.load_program(program)

        pid = pcbTable.nuevoPid
        pcb = Pcb(pid, baseDir, 0, program.name, priority)

        pcbTable.add_pcb(pcb)
        self.pcbIn(pcb)

        log.logger.info("\n Executing program: {name}".format(name=program.name))
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
            dispatcher.load_pcb(pcbAEjecutar)
            pcbTable.runningPCB = pcbAEjecutar
        else:
            HARDWARE.timer.reset()


# emulates the core of an Operative System
class Kernel():

    def __init__(self, schedulerToUse):
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

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        ## driver for interaction with cpu and mmu
        self._dispatcher = Dispatcher(HARDWARE.cpu, HARDWARE.mmu)

        ##specific component to load programs in main memmory
        self._loader = Loader()

        ##helps kernel to keep process state
        self._pcbTable = PcbTable()

        ##the scheduler with which the kernel will work
        self._scheduler = schedulerToUse
        self.scheduler.setUpTimer()  # set up the timer of the system with the quantum that the scheduler has

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
    def run(self, program, priority):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, (program, priority))
        HARDWARE._interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "


# estos son ls pocibles estados de un pcb
RUNNING_PCB_STATE = "#running"
WAITING_PCB_STATE = "#waiting"
READY_PCB_STATE = "#ready"
TERMINATED_PCB_STATE = "#terminated"


class Pcb():

    def __init__(self, pid, unaBaseDir, unPC, unPath, priority):
        self._pid = pid
        self._baseDir = unaBaseDir
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
    def baseDir(self):
        return self._baseDir

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

    def load_pcb(self, pcb):
        log.logger.info("Loading: {}".format(pcb))
        self._cpu.pc = pcb.pc
        self._mmu.baseDir = pcb.baseDir
        HARDWARE.timer.reset()

    def save_pcb(self, pcb):
        pcb.pc = self._cpu.pc
        self._cpu.pc = -1


class Loader():

    def __init__(self):
        self._baseDir = 0

    def load_program(self, program):
        ## loads the program in main memory
        baseDirdelPrograma = self._baseDir
        primeraDireccioLibre = self._baseDir
        progSize = len(program.instructions)
        for index in range(0, progSize):
            inst = program.instructions[index]
            HARDWARE.memory.write(primeraDireccioLibre, inst)
            primeraDireccioLibre = primeraDireccioLibre + 1
        self._baseDir = primeraDireccioLibre
        return baseDirdelPrograma


class AbstractScheduler():

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
        log.logger.info("ReadyQueue: {}".format(self._readyQueue))

    def hasHigherPriority(self, pcb1, pcb2):
        return pcb1.priority < pcb2.priority


    def readyQueueIsEmpty(self):
        return not self._readyQueue

    def getPcb(self):
        return self._readyQueue.pop(0)

class PriorityPreemtiveScheduler(PriorityNoPreemtiveScheduler):

    def mustExpropiate(self, pcbToAdd, pcbInCPU):
        return self.hasHigherPriority(pcbToAdd, pcbInCPU)

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
            todosEnDead  = self.actualizarRepresentacion(ticknmbr, self._kernel.pcbTable.pcbTable)

        if todosEnDead:
            log.logger.info(self.__repr__())
            HARDWARE.switchOff()

    def actualizarRepresentacion(self,ticknmbr, pcbTable):
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

    def armarCuadro(self,pcbTable):
        listaAEntregar = []
        for pcb in pcbTable:
            listaAEntregar.append([pcb.path])
        return listaAEntregar



    def __repr__(self):
        return tabulate(self._representacionDeEjecucion, headers=self._headers, tablefmt='grid', stralign='center')
