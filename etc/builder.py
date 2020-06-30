from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort, AsynIP
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, \
    makeTemplateInstance
from iocbuilder.arginfo import *
from iocbuilder.modules.motor import MotorLib


# Peform template subsitution
@includesTemplates(ADBaseTemplate)
class _MainDbFile(AutoSubstitution):
    TemplateFile = 'ADPandABlocks.template'


class _PosBusTemplate(AutoSubstitution):
    TemplateFile = 'ADPandABlocksPosBus.template'


class _MotorSyncTemplate(AutoSubstitution):
    """Synchronises motor record MRES, OFFSET, UNITS with PandABlocks INENC and
    sets position after home"""
    TemplateFile = 'ADPandABlocksMotorSync.template'


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

    def __init__(self, PORT, HOST, P, R, MAXBUF=1000, MAXMEM=0, **args):
        # create asyn ports for PandA
        self.control_port = AsynIP('%s:8888' % HOST, '%s_CTRL' % PORT)
        self.data_port = AsynIP('%s:8889' % HOST, '%s_DATA' % PORT)

        # Call init on Device superclass
        self.__super.__init__(PORT)

        # Store arguments away to use later
        self.PORT = PORT
        self.HOST = HOST
        self.P = P
        self.R = R
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
        print "# Create driver"
        print 'ADPandABlocksConfig("%(PORT)s", "%(HOST)s", ' \
            '%(NELM)d, %(MAXBUF)d, %(MAXMEM)d, 0)' % self.__dict__

    # Arguments to supply for XMLBuilder
    ArgInfo = _MainDbFile.ArgInfo + makeArgInfo(__init__,
        PORT     = Simple("Asyn port name for the created driver", str),
        HOST     = Simple("PandA Box - can be hostname or IP address", str),
        P        = Simple("PV prefix", str),
        R        = Simple("PV suffix", str),
        MAXBUF   = Simple("Maximum number of buffers (areaDetector)", int),
        MAXMEM   = Simple("Maximum memory (areaDetector)", int))


class MotorSync(Device):

    def __init__(self, PANDA, MOTOR, ENC_IND, DIR="+", MULT=1,
                 READONLY=True, HOMESETTLE=5):
        self.__super.__init__()

        # Convert read only value from bool to int
        READONLY_VALUE = 0
        if READONLY is True:
            READONLY_VALUE = 1

        # Instantiate the motor sync template
        _MotorSyncTemplate(
            MOTOR=MOTOR,
            P=PANDA.P,
            R=PANDA.R,
            PORT=PANDA.PORT,
            ENC_IND=ENC_IND,
            DIR=DIR,
            MULT=MULT,
            READONLY=READONLY_VALUE,
            HOMESETTLE=HOMESETTLE)

    # Arguments to supply for XMLBuilder
    ArgInfo = makeArgInfo(__init__,
        PANDA      = Ident("Parent PandA", ADPandABlocks),
        MOTOR      = Simple("PV of motor to sync with", str),
        ENC_IND    = Choice("Motor encoder index", [1, 2, 3, 4]),
        DIR        = Choice("Motor direction", ["+", "-"]),
        MULT       = Simple("Scale factor multiplier", float),
        READONLY   = Simple("Should embedded screen be read-only", bool),
        HOMESETTLE = Simple("Calibration delay after homing", int))
