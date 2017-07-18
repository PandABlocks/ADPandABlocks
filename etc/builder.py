from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort
from iocbuilder.modules.ADCore import ADCore, ADBaseTemplate, includesTemplates, \
    makeTemplateInstance
from iocbuilder.arginfo import *

# Peform template subsitution
@includesTemplates(ADBaseTemplate)
class _MainDbFile (AutoSubstitution):
    TemplateFile = 'ADPandABlocks.template'

# Main class for the ADPandABlocks device
class ADPandABlocks(AsynPort):

    # We depend upon Asyn
    Dependencies = (ADCore,)

    # Make sure our DBD file gets used or created
    DbdFileList = ['ADPandABlocks']

    # Make sure our library gets included by dependent IOCs
    LibFileList = ['ADPandABlocks']

    UniqueName = "PORT"

    def __init__(self, PORT, CMDPORT, DATAPORT, MAXBUF=1000, MAXMEM=0, **args):

        # Call init on Device superclass
        self.__super.__init__(PORT)

        # Store arguments away to use later
        self.PORT = PORT
        self.CMDPORT = CMDPORT
        self.DATAPORT = DATAPORT
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM
        self.NELM = 100000

        # Perform template subsitutions to create our DB file
        makeTemplateInstance(_MainDbFile, locals(), args)

    def Initialise(self):
        # Print the command to create the device in the startup script
        print "# Create driver"
        print 'ADPandABlocksConfig("%(PORT)s", "%(CMDPORT)s", "%(DATAPORT)s", ' \
            '%(NELM)d, %(MAXBUF)d, %(MAXMEM)d, 0)' % self.__dict__

    # tell xmlbuilder what args to supply
    ArgInfo = _MainDbFile.ArgInfo + makeArgInfo(__init__,
        PORT     = Simple("Asyn port name for the created driver", str),
        CMDPORT  = Ident("Low level Asyn port for sending control comands", AsynPort),
        DATAPORT = Ident("Low level Asyn port for receiving data", AsynPort),
        MAXBUF   = Simple("Maximum number of buffers (areaDetector)", int),
        MAXMEM   = Simple("Maximum memory (areaDetector)", int))

