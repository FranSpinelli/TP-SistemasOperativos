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


class KillInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher

        pcb = pcbTable.runningPCB
        dispatcher.save_pcb(pcb)
        pcb.state = TERMINATED_PCB_STATE #si uso el setter no funca

        pcbTable.remove_Terminateds()
        pcbTable.runningPCB = None #si uso el setter no funca

        if pcbTable.is_any_pcb_Ready():

            pcbAEjecutar = pcbTable.get_first_pcb_ready()
            pcbAEjecutar.state = RUNNING_PCB_STATE #si uso el setter no funca
            dispatcher.load_pcb(pcbAEjecutar)
            pcbTable.runningPCB = pcbAEjecutar #si uso el setter no funca

class IoInInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable
        dispatcher = self.kernel.dispatcher

        operation = irq.parameters

        pcb = pcbTable.runningPCB
        pcbTable.runningPCB = None #si uso el setter no funca
        dispatcher.save_pcb(pcb)
        pcb.state = WAITING_PCB_STATE #si uso el setter no funca

        self.kernel.ioDeviceController.runOperation(pcb, operation)
        log.logger.info(self.kernel.ioDeviceController)

        if pcbTable.is_any_pcb_Ready():

            pcbAEjecutar = pcbTable.get_first_pcb_ready()
            pcbAEjecutar.state = RUNNING_PCB_STATE #si uso el setter no funca
            dispatcher.load_pcb(pcbAEjecutar)
            pcbTable.runningPCB = pcbAEjecutar #si uso el setter no funca

class IoOutInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable

        pcb = self.kernel.ioDeviceController.getFinishedPCB()

        if pcbTable.runningPCB is None:
            self.kernel.dispatcher.load_program(pcb)
            pcb.state = RUNNING_PCB_STATE
            pcbTable.runningPCB = pcb
        else:
            pcb.state = READY_PCB_STATE

class NewInterruptionHandler(AbstractInterruptionHandler):

    def execute(self, irq):
        pcbTable = self.kernel.pcbTable

        baseDir = self._kernel.load_program(irq.parameters)

        pcb = Pcb(baseDir, 0, irq.parameters.name)
        pid = pcbTable.nuevoPid

        pcbTable.add_pcb(pcb, pid)

        if pcbTable.runningPCB is None:
            pcb.state = RUNNING_PCB_STATE #lo tuve que hacer public por que sino tira 'str' object is not callable
            pcbTable.runningPCB = pcb # lo tuve que hacer public por que decia que no puedo usar el setter
            self._kernel.dispatcher.load_pcb(pcb)

        log.logger.info("\n Executing program: {name}".format(name=irq.parameters.name))
        log.logger.info(HARDWARE)

# emulates the core of an Operative System
class Kernel():

    def __init__(self):
        ## setup interruption handlers
        killHandler = KillInterruptionHandler(self)
        HARDWARE.interruptVector.register(KILL_INTERRUPTION_TYPE, killHandler)

        ioInHandler = IoInInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_IN_INTERRUPTION_TYPE, ioInHandler)

        ioOutHandler = IoOutInterruptionHandler(self)
        HARDWARE.interruptVector.register(IO_OUT_INTERRUPTION_TYPE, ioOutHandler)

        newHandler = NewInterruptionHandler(self)
        HARDWARE.interruptVector.register(NEW_INTERRUPTION_TYPE, newHandler)

        ## controls the Hardware's I/O Device
        self._ioDeviceController = IoDeviceController(HARDWARE.ioDevice)

        ## driver for interaction with cpu and mmu
        self._dispatcher = Dispatcher(HARDWARE.cpu, HARDWARE.mmu)

        ##specific component to load programs in main memmory
        self._loader = Loader()

        ##helps kernel to keep process state
        self._pcbTable = PcbTable();

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

    def load_program(self, program):
        return self._loader.load_program(program)

    ## emulates a "system call" for programs execution
    def run(self, program):
        newIRQ = IRQ(NEW_INTERRUPTION_TYPE, program)
        HARDWARE._interruptVector.handle(newIRQ)

    def __repr__(self):
        return "Kernel "


# estos son ls pocibles estados de un pcb
RUNNING_PCB_STATE = "#running"
WAITING_PCB_STATE = "#waiting"
READY_PCB_STATE = "#ready"
TERMINATED_PCB_STATE = "#terminated"


class Pcb():

    def __init__(self, unaBaseDir, unPC, unPath):
        self._baseDir = unaBaseDir
        self._pc = unPC
        self._path = unPath
        self.state = READY_PCB_STATE

    @property
    def baseDir(self):
        return self._baseDir
    @property
    def pc(self):
        return self._pc

    @property
    def path(self):
        return self._path

    #@property
    #def state(self):
    #    return self._state

    #@state.setter
    #def state(self, nuevoState):
    #    self._state = nuevoState

    @pc.setter
    def pc(self, nuevoPC):
        self._pc = nuevoPC


class PcbTable():

    def __init__(self):
        self._pidActual = 0
        self._pcbTable = dict()
        self.runningPCB = None

    @property
    def nuevoPid(self):
        valorARetornar = self._pidActual
        self._pidActual = self._pidActual + 1
        return valorARetornar

    def get_first_pcb_ready(self):
        respuesta = None
        for(key,value) in self._pcbTable.items():
            if (value.state == READY_PCB_STATE) & (respuesta is None):
                respuesta = value
        return respuesta

    def is_any_pcb_Ready(self):
        respuesta = False
        for (key,value) in self._pcbTable.items():
            if value.state == READY_PCB_STATE:
                respuesta = True
        return respuesta

    def remove_Terminateds(self):
        new_dict = dict()
        for (key, value) in self._pcbTable.items():
            if value.state != TERMINATED_PCB_STATE:
                new_dict[key] = value
        self._pcbTable = new_dict

    def get_pcb(self, pid):
        return self._pcbTable[pid]

    def add_pcb(self, pcb, pID):
        self._pcbTable[pID] = pcb

    def remove_pcb(self, pid):
        self._pcbTable.pop(pid)

    # @property
    # def runningPCB(self):
    #    return self._runningPCB

    # @runningPCB.setter
    # def runningPCB(self, nuevorunningpcb):
    #    self._runningPCB = nuevorunningpcb


class Dispatcher():

    def __init__(self, unCPU, unMMu):
        self._cpu = unCPU
        self._mmu = unMMu

    def load_pcb(self, pcb):
        self._cpu.pc = pcb.pc # no anda si uso el setter
        self._mmu.baseDir = pcb.baseDir # no anda si uso el setter

    def save_pcb(self, pcb):
        pcb.pc = self._cpu.pc # no anda si uso el setter
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
            primeraDireccioLibre = primeraDireccioLibre + 1  ##WTF pasa si hago primeraDirLibre=+1???
        self._baseDir = primeraDireccioLibre
        return baseDirdelPrograma
