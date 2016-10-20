from iocbuilder import AutoSubstitution, Device
from iocbuilder.modules.asyn import Asyn, AsynPort
from iocbuilder.modules.ADCore import ADCore
from iocbuilder.modules.adUtil import AdUtil
from iocbuilder.arginfo import *

# Peform template subsitution
class MainDbFile (AutoSubstitution):
    TemplateFile = 'pandabox.template'

# Main class for the pandabox device
class Pandabox( AsynPort):

    # We depend upon Asyn
    Dependencies = (Asyn,ADCore,AdUtil)

    # Make sure our DBD file gets used or created
    DbdFileList = ['pandabox']
    
    # Make sure our library gets included by dependent IOCs
    LibFileList = ['pandabox']
    
    # This seems to be necessary
    AutoInstantiate = True
    
    # Is an Asyn device
    IsAsyn = True
    
    def __init__(self, name, P="P", R="R", CMDPORT="CMDPORT", DATAPORT="DATAPORT", ADDR=0, TIMEOUT=1000, MAXBUF=5, MAXMEM=0):
    
        # Call init on Device superclass
        self.__super.__init__(name)
        
        # Store arguments away to use later
        self.P = P
        self.R = R
        self.CMDPORT = CMDPORT
        self.DATAPORT = DATAPORT
        self.name = name
        self.ADDR = ADDR
        self.TIMEOUT = TIMEOUT
        self.MAXBUF = MAXBUF
        self.MAXMEM = MAXMEM
        self.NELM = 100000
        
        # Perform template subsitutions to create our DB file
        makeDb = MainDbFile(PORT=self.name, P=P, R=R, ADDR=ADDR, TIMEOUT=TIMEOUT)
    
    def InitialiseOnce(self):
        # Print some description of what we're doing in the startup script
        print "# pandabox driver"
        
    def Initialise(self):
        # Print the command to create the device in the startup script
        print "# Create driver"
        #print "pandaboxDriverCreate(\"{0}\", \"{1}\", \"{2}\", 0)".format(self.name, self.CMDPORT, self.DATAPORT)   
        print "pandaboxConfig(\"{0}\", \"{1}\", \"{2}\", \"{3}\", \"{4}\", \"{5}\", 0)".format(self.name, self.CMDPORT, self.DATAPORT, self.NELM, self.MAXBUF, self.MAXMEM )   
#         print 'pandaboxConfig("%(PORT)s", "%(cmdSerialPort)s", "%(dataSerialPort)s", %(NELM)s, %(MAXBUF)d, %(MAXMEM)d)' % self.__dict__
    
    # tell xmlbuilder what args to supply
    ArgInfo = makeArgInfo(__init__,
        P        = Simple("Device prefix", str),
        R        = Simple("Device suffix", str),
        CMDPORT  = Simple("Low level Asyn port for sending control comands", str),
        DATAPORT = Simple("Low level Asyn port for receiving data", str),
        name     = Simple("Test name", str),
        ADDR     = Simple("Address of this controller", str),
        TIMEOUT  = Simple("Timeout for communications, ms", str),
        MAXBUF   = Simple("Maximum number of buffers (areaDetector)", int),
        MAXMEM   = Simple("Maximum memory (areaDetector)", int))
        
    
        

    
