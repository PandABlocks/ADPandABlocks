from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, \
    makeTemplateInstance
from iocbuilder.arginfo import *
from iocbuilder.modules.motor import MotorLib


# Perform template subsitution
@includesTemplates(ADBaseTemplate)
class _MainDbFile(AutoSubstitution):
    TemplateFile = 'ADPandABlocks.template'


class _PosBusTemplate(AutoSubstitution):
    TemplateFile = 'ADPandABlocksPosBus.template'


class _MotorSyncTemplate(AutoSubstitution):
    """Synchronises motor record MRES, OFFSET, UNITS with PandABlocks INENC and
    sets position after home"""
    TemplateFile = 'ADPandABlocksMotorSync.template'


class _CustomParamTemplate(AutoSubstitution):
    TemplateFile = 'ADPandABlocksCustomParam.template'


class _TTLControl(AutoSubstitution):
    TemplateFile = 'ADPandABlocksTTLControl.template'


numTTLControl = 0
class TTLControl(Device):
    def __init__(self, PORT, P, R, TTL_IND=1):
        global numTTLControl
        PARAM1_IND = "%02d" % ((numTTLControl * 2) + 1)
        PARAM2_IND = "%02d" % ((numTTLControl * 2) + 2)
        self.template = _TTLControl(PORT=PORT, P=P, R=R, TTL_IND=TTL_IND, PARAM1_IND=PARAM1_IND,
                                    PARAM2_IND=PARAM2_IND)
        numTTLControl += 1

    ArgInfo = makeArgInfo(__init__,
                          P=Simple("Device prefix", str),
                          R=Simple("Device suffix", str),
                          PORT=Simple("Asyn port", str),
                          TTL_IND=Choice("TTL output index", [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]))


# Use TTL output for software triggering a detector with a gated pulse
# Probably not something we should encourage, just use malcolm!
# class SoftTTLTrigger(AutoSubstitution):
#    TemplateFile = 'ADPandABlocksSoftTTLTrigger.template'


class MotorSync(Device):

    def __init__(self, MOTOR, P, R, PORT, ENC_IND,
                 DIR="+", MULT=1, READONLY=True, HOMESETTLE=5):
        self.__super.__init__()
        self.MOTOR = MOTOR
        self.P = P
        self.R = R
        self.PORT = PORT
        self.ENC_IND = ENC_IND
        self.DIR = DIR
        self.MULT = MULT
        self.READONLY_VALUE = 0
        if READONLY is True:
            self.READONLY_VALUE = 1
        self.HOMESETTLE = HOMESETTLE
        _MotorSyncTemplate(
            MOTOR=self.MOTOR,
            P=self.P,
            R=self.R,
            PORT=self.PORT,
            ENC_IND=self.ENC_IND,
            DIR=self.DIR,
            MULT=self.MULT,
            READONLY=self.READONLY_VALUE,
            HOMESETTLE=self.HOMESETTLE
        )

    ArgInfo = _MotorSyncTemplate.ArgInfo + makeArgInfo(__init__,
                                                       MOTOR=Simple("PV of motor to sync with", str),
                                                       P=Simple("Device prefix", str),
                                                       R=Simple("Device suffix", str),
                                                       PORT=Simple("Asyn port", str),
                                                       ENC_IND=Choice("Motor encoder index", [1, 2, 3, 4]),
                                                       DIR=Choice("Motor direction", ["+", "-"]),
                                                       MULT=Simple("Scale factor multiplier", float),
                                                       READONLY=Simple("Should embedded screen be read-only", bool),
                                                       HOMESETTLE=Simple("Calibration delay after homing", int))


# Main class for the ADPandABlocks device
class ADPandABlocks(AsynPort):
    # We depend upon Asyn
    Dependencies = (ADCore, Asyn)

    # Make sure our DBD file gets used or created
    DbdFileList = ['ADPandABlocks']

    # Make sure our library gets included by dependent IOCs
    LibFileList = ['ADPandABlocks']

    UniqueName = "PORT"
    N_POSBUS = 32
    N_CUSTOM = 0

    def __init__(self, PORT, HOST, MAXBUF=1000, MAXMEM=0, **args):

        # create asyn ports for PandA
        self.control_port = AsynIP('%s:8888' % HOST, '%s_CTRL' % PORT)
        self.data_port = AsynIP('%s:8889' % HOST, '%s_DATA' % PORT)

        # Call init on Device superclass
        self.__super.__init__(PORT)

        # Store arguments away to use later
        self.PORT = PORT
        self.HOST = HOST
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM
        self.NELM = 100000

        # Perform template subsitutions to create our DB file
        makeTemplateInstance(_MainDbFile, locals(), args)
        locals().update(args)

        # Create the templates for the position bus entries
        for i in range(self.N_POSBUS):
            makeTemplateInstance(_PosBusTemplate, locals(), {'POSBUS_IND': ("%d" % i)})

        # Create templates for custom params
        for i in range(self.N_CUSTOM):
            makeTemplateInstance(_CustomParamTemplate, locals(), {'PARAM_IND': ("%02d" % (i + 1))})

    def Initialise(self):
        # Print the command to create the device in the startup script
        # print "# Create AsynIP ports for PandA"
        # print 'drvAsynIPPortConfigure("%(PORT)s_CTRL", "%(HOST)s:8888", 100, 0, 0)'  % self.__dict__
        # print 'drvAsynIPPortConfigure("%(PORT)s_DATA", "%(HOST)s:8889", 100, 0, 0)'  % self.__dict__
        print "# Create driver"
        print 'ADPandABlocksConfig("%(PORT)s", "%(HOST)s", ' \
              '%(NELM)d, %(MAXBUF)d, %(MAXMEM)d, 0)' % self.__dict__

    # tell xmlbuilder what args to supply
    ArgInfo = _MainDbFile.ArgInfo + makeArgInfo(__init__,
                                                PORT=Simple("Asyn port name for the created driver", str),
                                                HOST=Simple("PandA Box - can be hostname or IP address", str),
                                                MAXBUF=Simple("Maximum number of buffers (areaDetector)", int),
                                                MAXMEM=Simple("Maximum memory (areaDetector)", int))
