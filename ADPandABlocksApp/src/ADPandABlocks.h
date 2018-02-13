#ifndef ADPandABlocks_H
#define ADPandABlocks_H

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
#define NBUFF 255
#define N_BUFF_CTRL 255
#define N_BUFF_DATA 65536

/* This is the number of waveforms to store */
#define NARRAYS 10

/* This is the number of positions on the posbus*/
#define NPOSBUS 32

/* This is the number of filtered waveforms to allow */
#define NFILT 4

/* The timeout waiting for a response from ADPandABlocks */
#define TIMEOUT 1.0

/* This is the frame height for the NDArrays */
#define FRAMEHEIGHT 1

/* We want to block while waiting on an asyn port forever.
 * Unfortunately putting 0 or a large number causes it to
 * poll and take up lots of CPU. This number seems to work
 * and causes it to block for a reasonably long time (in seconds)
 */
#define LONGWAIT 1000.0

class ADPandABlocks: public ADDriver {
private:
    /**Typedefs**/
    //vector of maps to store header data. Each map is for an individual xml node
    typedef std::vector<std::map<std::string, std::string> > headerMap;

public:
    ADPandABlocks(const char *portName, const char* cmdSerialPortName,
            const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory);

    /** These should be private, but get called from C, so must be public */
    void readTaskData();

    /** These functions are used in the tests, so they are public */
    asynStatus sendCtrl(const std::string txBuffer);
    asynStatus sendData(const std::string txBuffer);
    asynStatus send(const std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser);

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

protected:

#define FIRST_PARAM ADPandABlocksIsConnected
    int ADPandABlocksIsConnected;        // int32 read  - is ADPandABlocks connected?
    int ADPandABlocksHeader;             // string read - data header
    int ADPandABlocksDataEnd;            // string read - end of data string
    int ADPandABlocksPCTime;             // float64array read - position compare timestamps
#define LAST_PARAM ADPandABlocksPCTime
    int ADPandABlocksScale[NPOSBUS];              // string read - motor scale
    int ADPandABlocksPosFields[NPOSBUS];   // string read - position field names
    //int ADPandABlocksOff;                // string read - motor offset
    //int ADPandABlocksUnits;              // string read - motor units
    //int ADPandABlocksCaptureType;        // string read - pcap capture type
#define NUM_PARAMS (&LAST_PARAM - &FIRST_PARAM + 1)

private:
    headerMap parseHeader(const std::string& headerString);
    void parseData(std::vector<char> dataBuffer, const int dataLen);
    void allocateFrame();
    void wrapFrame();
    std::vector<std::string> readFieldNames(int* numFields);
    asynStatus extractHeaderData(const xmlTextReaderPtr xmlreader, std::map<std::string, std::string>& values)const;
    std::string getHeaderValue(const int index, const std::string attribute)const;
    void getAllData(std::vector<char>& inBuffer, const int dataLen,const  int buffLen)const;
    void outputData(const int dataLen, const int dataNo, const std::vector<char> data);
    asynStatus readHeaderLine(char* rxBuffer, const size_t buffSize)const;
    asynStatus readDataBytes(char* rxBuffer, const size_t nBytes)const;

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

    //vector of maps for the header values
    headerMap headerValues;

    //Vector containing vector of strings for bit mask values
    std::vector<std::vector<std::string> > bitMasks;

    //Vector containing vector of strings for position fields
    std::vector<std::vector<std::string> > posFields;

    //states for readDataTask state machine
    enum readState{waitHeaderStart, waitHeaderEnd, waitDataStart, receivingData, dataEnd,};
    readState state; //init state for the data read

};
#endif
