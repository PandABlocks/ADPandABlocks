#ifndef PANDABOX_H
#define PANDABOX_H

#include <cstring>
#include <string>
#include <map>
#include <vector>

#include <libxml/xmlreader.h>

#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"
#include "ADDriver.h"
#include "drvAsynIPPort.h"
#include "asynShellCommands.h"

/* This is the number of messages on our queue */
#define NQUEUE 10000

/* The size of our transmit and receive buffers,
 * max filename length and string param buffers */
// BK: ambigious names, maybe N_IO_BUFF and N_FILEPATH_BUFF, N_STRING_PARAM_BUFF
#define NBUFF 255
#define NBUFF2 65536

/* This is the number of waveforms to store */
#define NARRAYS 10

/* This is the number of filtered waveforms to allow */
#define NFILT 4

/* The timeout waiting for a response from pandabox */
#define TIMEOUT 1.0

/* This is the frame height for the NDArrays */
#define FRAMEHEIGHT 1

/* We want to block while waiting on an asyn port forever.
 * Unfortunately putting 0 or a large number causes it to
 * poll and take up lots of CPU. This number seems to work
 * and causes it to block for a reasonably long time (in seconds)
 */
#define LONGWAIT 1000.0

class Pandabox: public ADDriver {
private:
    /**Typedefs**/
    //vector of maps to store header data. Each map is for an individual xml node
    typedef std::vector<std::map<std::string, std::string> > headerMap; 

public:
    Pandabox(const char *portName, const char* cmdSerialPortName, const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory);

    /** These should be private, but get called from C, so must be public */
    void readTaskCtrl();
    void readTaskData();

    // BK: any reason why these are not private?
    asynStatus sendCtrl(std::string txBuffer);
    asynStatus sendData(std::string txBuffer);
    asynStatus send(std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

protected:

#define FIRST_PARAM pandaboxIsConnected
    int pandaboxIsConnected;        // int32 read  - is pandabox connected?
    int pandaboxHeader;             // string read - data header
    int testparam;                 // int32 write - initiates test setup
    int pandaboxPCTime;             // float64array read - position compare timestamps
#define LAST_PARAM pandaboxPCTime
#define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)

private:
    void setDataFormat();
    headerMap parseHeader(std::string* headerString);
    void parseData(std::vector<char> dataBuffer, int dataLen);
    void allocateFrame();
    void wrapFrame();
    asynStatus extractHeaderData(xmlTextReaderPtr xmlreader, std::map<std::string, std::string>* values);
    std::string getHeaderValue(int index, std::string attribute);
    void setParams();
    void getAllData(std::vector<char>* inBuffer, int dataLen, int buffLen);
    void outputData(int dataLen, int dataNo, std::vector<char> data);
    asynStatus readHeaderLine(char* rxBuffer, size_t* nBytesIn);
    asynStatus readDataBytes(char* rxBuffer, int nBytes);
    void endCapture();

private:
    NDArray *pArray; 
    asynUser *pasynUser_ctrl;
    asynOctet *pasynOctet_ctrl;
    void *octetPvt_ctrl; // BK: is there a good reason for this to be part of the global state?
    asynUser *pasynUser_data;
    asynCommon *pasynCommon_data;
    void *pcommonPvt_data;
    asynOctet *pasynOctet_data;
    void *octetPvt_data;
    int arrayCounter, numImagesCounter, imgMode, imgNo;
    bool capture;
    std::string header;
    size_t readBytes; // BK: should be local, never really used as a class field

    //vector of maps for the header values
    headerMap headerValues;

    //states for readDataTask state machine
    enum readStates{waitHeaderStart=0, waitHeaderEnd, waitDataStart, receivingData, dataEnd,}; // BK: First value is always zero in enum, there is no direct dependency on this 0; name of the enum should probably be in singular as it names a type
    readStates state; //init state for the data read

    std::map<asynStatus, std::string> errorMsg; // BK: Could be moved outside of class
};
#endif
