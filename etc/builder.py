from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, \
    makeTemplateInstance
from iocbuilder.arginfo import *
from iocbuilder.modules.motor import MotorLib

# Peform template subsitution
@includesTemplates(ADBaseTemplate)
class _MainDbFile (AutoSubstitution):
    TemplateFile = 'ADPandABlocks.template'

class _PosBusTemplate (AutoSubstitution):
    TemplateFile = 'ADPandABlocksPosBus.template'

class _MotorSyncTemplate (AutoSubstitution):
    """Synchronises motor record MRES, OFFSET, UNITS with PandABlocks INENC and
    sets position after home"""
    TemplateFile = 'ADPandABlocksMotorSync.template'

class MotorSync (Device):

    def __init__(   self, MOTOR, P, R, PORT, ENC_IND,
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
            DIR = self.DIR,
            MULT = self.MULT,
            READONLY = self.READONLY_VALUE,
            HOMESETTLE = self.HOMESETTLE
        )

    ArgInfo = _MotorSyncTemplate.ArgInfo + makeArgInfo(__init__,
        MOTOR = Simple("PV of motor to sync with", str),
        P = Simple("Device prefix", str),
        R = Simple("Device suffix", str),
        PORT = Simple("Asyn port", str),
        ENC_IND = Choice("Motor encoder index", [1, 2, 3, 4]),
        DIR = Choice("Motor direction", ["+", "-"]),
        MULT = Simple("Scale factor multiplier", float),
        READONLY = Simple("Should embedded screen be read-only", bool),
        HOMESETTLE = Simple("Calibration delay after homing", int))


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

    def __init__(self, PORT, ADDRESS, MAXBUF=1000, MAXMEM=0, **args):
        
        # create asyn ports for PandA
        self.control_port = AsynIP('%s:8888' % ADDRESS, '%s_CTRL' % PORT)
        self.data_port = AsynIP('%s:8889' % ADDRESS, '%s_DATA' % PORT)

        # Call init on Device superclass
        self.__super.__init__(PORT)

        # Store arguments away to use later
        self.PORT = PORT
        self.ADDRESS = ADDRESS
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM
        self.NELM = 100000       
        
        # Perform template subsitutions to create our DB file
        makeTemplateInstance(_MainDbFile, locals(), args)
        locals().update(args)

        # Create the templates for the position bus entries
        for i in range(self.N_POSBUS):
            makeTemplateInstance(_PosBusTemplate, locals(), {'POSBUS_IND' : ("%d" % i)})

    def Initialise(self):
        # Print the command to create the device in the startup script
        #print "# Create AsynIP ports for PandA"
        #print 'drvAsynIPPortConfigure("%(PORT)s_CTRL", "%(ADDRESS)s:8888", 100, 0, 0)'  % self.__dict__
        #print 'drvAsynIPPortConfigure("%(PORT)s_DATA", "%(ADDRESS)s:8889", 100, 0, 0)'  % self.__dict__
        print "# Create driver"
        print 'ADPandABlocksConfig("%(PORT)s", "%(ADDRESS)s", ' \
            '%(NELM)d, %(MAXBUF)d, %(MAXMEM)d, 0)' % self.__dict__

    # tell xmlbuilder what args to supply
    ArgInfo = _MainDbFile.ArgInfo + makeArgInfo(__init__,
        PORT     = Simple("Asyn port name for the created driver", str),
        ADDRESS  = Simple("Address of pandaBox", str),
        MAXBUF   = Simple("Maximum number of buffers (areaDetector)", int),
        MAXMEM   = Simple("Maximum memory (areaDetector)", int))


