// BK: This driver will only work on machines with the same endianness as panda.
// BK: const modifiers on input parameters and methods are missing
// BK: Prefer const references in params instead of pointers where the passed 
//     objects are not changed, references if the memory position of the object
//     is not changed in method (the reference cannot be null), pointers are 
//     usually reserved for output parameters
// BK: this-> used in some cases and not in others, also in the same function 
//     which is confusing

#include "pandabox.h"

#include <stdint.h>
#include <epicsExport.h>
#include <iocsh.h>

#include <libxml/xmlreader.h>

#include "epicsThread.h"


// BK: the following functions should be defined in the cpp file
/* C function to call new message from  task from epicsThreadCreate */
static void readTaskCtrlC(void *userPvt) {
    Pandabox *pPvt = (Pandabox *) userPvt;
    pPvt->readTaskCtrl();
}

static void readTaskDataC(void *userPvt) {
    Pandabox *pPvt = (Pandabox *) userPvt;
    pPvt->readTaskData();
}

static const char *driverName = "pandabox";
static std::map<asynStatus, std::string> errorMsg;

Pandabox::Pandabox(const char* portName, const char* cmdSerialPortName, const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory) :
        ADDriver(portName, 1 /*maxAddr*/, NUM_PARAMS, maxBuffers, maxMemory,
                asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask,
                asynInt8ArrayMask | asynFloat64ArrayMask | asynInt32Mask | asynFloat64Mask | asynOctetMask,
                ASYN_CANBLOCK, /*ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0 */
                1, /*autoConnect*/ 0, /*default priority */ 0 /*default stack size*/) {

    //private variables
    state = waitHeaderStart; //init state for the data read

    errorMsg[asynSuccess] = "asynSuccess";
    errorMsg[asynTimeout] = "asynTimeout";
    errorMsg[asynOverflow] = "asynOverflow";
    errorMsg[asynError] = "asynError";
    errorMsg[asynDisconnected] = "asynDisconnected";
    errorMsg[asynDisabled] = "asynDisabled";

    const char *functionName = "Pandabox";
    asynStatus status = asynSuccess;
    asynInterface *pasynInterface;

    /* For areaDetector image */
    this->pArray = NULL;
    this->arrayCounter = 0;
    this->numImagesCounter = 0;
    this->imgMode = ADImageContinuous;
    this->imgNo = 0;
    this->capture = true;
    this->readBytes = NBUFF2-1;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &pandaboxIsConnected);
    setIntegerParam(pandaboxIsConnected, 0);

    /*Create a parameter to store the header value */
    createParam("HEADER", asynParamOctet, &pandaboxHeader);

    /* initialise areaDetector parameters */
    setStringParam(ADManufacturer, "Diamond Light Source Ltd.");
    setStringParam(ADModel, "Pandabox");
    setIntegerParam(ADMaxSizeX, NARRAYS + 1);
    setIntegerParam(ADMaxSizeY, FRAMEHEIGHT);
//    setIntegerParam(NDDataType, 7);
    setIntegerParam(ADStatus, ADStatusIdle);
    setStringParam(ADStatusMessage, "Idle");

    /* Connect to the device port */
    /* Copied from asynOctecSyncIO->connect */
    pasynUser_ctrl = pasynManager->createAsynUser(0, 0);
    //pasynInterface = connectToDevicePort(pasynUser_ctrl, cmdSerialPortName);
    status = pasynManager->connectDevice(pasynUser_ctrl, cmdSerialPortName, 0);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, cmdSerialPortName, status);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
        return;
    }
    
    pasynOctet_ctrl = (asynOctet *) pasynInterface->pinterface;
    octetPvt_ctrl = pasynInterface->drvPvt;

    /* Set EOS and flush */
    pasynOctet_ctrl->flush(octetPvt_ctrl, pasynUser_ctrl);
    pasynOctet_ctrl->setInputEos(octetPvt_ctrl, pasynUser_ctrl, "\n", 1);
    pasynOctet_ctrl->setOutputEos(octetPvt_ctrl, pasynUser_ctrl, "\n", 1);

    /* Create the thread that reads from the device  */
    if (epicsThreadCreate("PandaboxReadTask", epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) readTaskCtrlC, this) == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }


    /* Connect to the data port */
    /* Copied from asynOctecSyncIO->connect */
    // BK : this block of code duplicates a lot of things done in the previous block
    pasynUser_data = pasynManager->createAsynUser(0, 0); // BK: we already have a pasynUser_ctrl with same params
    status = pasynManager->connectDevice(pasynUser_data, dataSerialPortName, 0);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, dataSerialPortName, status);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_data, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynCommon_data = (asynCommon *) pasynInterface->pinterface; // BK: saving a single pointer multiple times with different casts to a class state
    pcommonPvt_data = pasynInterface->drvPvt; // BK: same pointer saved multiple times?
    pasynInterface = pasynManager->findInterface(pasynUser_data, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
        return;
    }
    pasynOctet_data = (asynOctet *) pasynInterface->pinterface;
    octetPvt_data = pasynInterface->drvPvt;

    /* Set EOS and flush */
    pasynOctet_data->flush(octetPvt_data, pasynUser_data);
    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
    pasynOctet_data->setOutputEos(octetPvt_data, pasynUser_data, "\n", 1);
    pasynUser_data->timeout = LONGWAIT;

    /* Create the thread that reads from the device  */
    if (epicsThreadCreate("PandaboxReadTask2", epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) readTaskDataC, this) == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }

    /*set the receiving format on the data channel*/
    sendData("XML FRAMED SCALED\n");
};

asynInterface* Pandabox::connectToDevicePort(asynUser* pasynUser, const char* serialPortName) {
    const char *functionName = "connectToDevicePort";
    asynInterface *pasynInterface;
    /* Connect to the device port */
    /* Copied from asynOctecSyncIO->connect */
    asynStatus status = pasynManager->connectDevice(pasynUser, serialPortName, 0);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, serialPortName, status);
    }
    pasynInterface = pasynManager->findInterface(pasynUser, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return pasynInterface;
    }
    pasynInterface = pasynManager->findInterface(pasynUser, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
        return pasynInterface;
    }
    //return status;
}

/* This is the function that will be run for the read thread */
void Pandabox::readTaskCtrl() {
    const char *functionName = "readTaskCtrl";
    //char *rxBuffer;
    char rxBuffer[NBUFF];
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);

    while (true) {
        pasynUserRead->timeout = LONGWAIT;
        status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, NBUFF - 1,
                &nBytesIn, &eomReason);
        if (status) {
            epicsThreadSleep(TIMEOUT);
        } else if (eomReason & ASYN_EOM_EOS) {
            // Replace the terminator with a null so we can use it as a string
            rxBuffer[nBytesIn] = '\0';
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
        }
    }
}

asynStatus Pandabox::readHeaderLine(char* rxBuffer, size_t *nBytesIn) { // BK: nBytesIn is always discarded when this function is called, do we need this param?maybe all requirements on this size could be checked from this function?
    const char *functionName = "readHeaderLine";
    int eomReason;
    asynStatus status = asynTimeout;

    while (status == asynTimeout) {
        status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, 
                NBUFF2, nBytesIn, &eomReason); // BK: functions only expect 4 bytes to be set in nBytesIn, we are reading NBUFF2 bytes. Also size of rxBuffer is not checked anywhere, if somebody provides a buffer that is smaller we are going to write all over the memory.
    }
    
    if(status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
                driverName, functionName, errorMsg[status].c_str());
    }
    assert (eomReason == ASYN_EOM_EOS); // BK: I would also log this before asserting, just to make debugging easier if it happens
    return status;
}

asynStatus Pandabox::readDataBytes(char* rxBuffer, int nBytes) { // BK: should nBytes be size_t?
    const char *functionName = "readDataBytes";
    int eomReason;
    size_t nBytesIn;
    asynStatus status = asynTimeout;

    while (status == asynTimeout) {
        status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, 
                nBytes, &nBytesIn, &eomReason);
    }

    if(status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
                driverName, functionName, errorMsg[status].c_str());
    }
    assert (nBytes == nBytesIn); // BK: I would also log this before asserting, just to make debugging easier if it happens
    return status;
}

/*this function reads from the data port*/
void Pandabox::readTaskData() {
    const char *functionName = "readTaskData";
    size_t nBytesIn;
    asynStatus status = asynSuccess;
    std::vector<char> dataPacket(0,0);
    char rxBuffer[NBUFF2];
    int eomReason;
    uint32_t dataLength = 0;

    while (true) {
        switch(state) {
            case waitHeaderStart:
                readHeaderLine(rxBuffer, &nBytesIn);
                if (strcmp(rxBuffer, "<header>\0") == 0) {
                    //we have a header so we have started acquiring
                    this->setIntegerParam(ADAcquire, 1);
                    setIntegerParam(NDArrayCounter, 0);
                    header.append(rxBuffer);
                    header.append("\n");
                    state = waitHeaderEnd;
                    callParamCallbacks();
                }
                break;

            case waitHeaderEnd:
                readHeaderLine(rxBuffer, &nBytesIn);
                /*accumulate the header until we reach the end, then process*/
                header.append(rxBuffer);
                header.append("\n");
                if (strcmp(rxBuffer, "</header>\0") == 0) {
                    headerValues = parseHeader(&header);
                    //change the input eos as the data isn't terminated with a newline
                    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "", 0);

                    //read the extra newline at the end of the header
                    status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, 1,
                            &nBytesIn, &eomReason);

                    state = waitDataStart;
                }
                break;

            case waitDataStart:
                // read "BIN " or "END "
                readDataBytes(rxBuffer, 4);
                if (strncmp(rxBuffer, "BIN ", 4) == 0) {
                    state = receivingData;
                }
                else if (strncmp(rxBuffer, "END ", 4) == 0) {
                    state = dataEnd;
                }
                break;

            case receivingData:
                // read next four bytes to get the packet size
                readDataBytes(rxBuffer, 4);
                dataLength = ((uint32_t*) rxBuffer)[0] - 8; // BK: only works for one endianesness, also looks rather odd, I'd create a function that does this explicitly with bitshifts.
                // read the rest of the packet
                readDataBytes(rxBuffer, dataLength);
                dataPacket.clear();
                dataPacket.insert(dataPacket.begin(), rxBuffer, rxBuffer + dataLength);
                parseData(dataPacket, dataLength);
                state = waitDataStart;
                break;

            case dataEnd:
                endCapture();
                state = waitHeaderStart;
                break;
        }
    }
    callParamCallbacks();
}

void  Pandabox::endCapture()
{
    this->state = waitHeaderStart;
    //check the next 4 bytes to see if it matches the total arrays read.
    //reset the header string
    this->header = "";
    //change the input eos back to newline for the header
    this->pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
    this->readBytes = NBUFF2-1; //reset the amount of bytes to read
    //set the acquire light to 0
    this->setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
}
Pandabox::headerMap Pandabox::parseHeader(std::string* headerString)
{
/**return a map containing the header data corresponding to each xml node
 * the first map will always be the 'data' node,
 * then each field will be pushed onto the vector sequentially
 */
    std::map<std::string, std::string> tmpValues;
    headerMap tmpHeaderValues;

    //set the header parameter
    setStringParam(pandaboxHeader, headerString->c_str());

    asynStatus status = asynSuccess;
    int ret = 0;
    xmlTextReaderPtr xmlreader = xmlReaderForMemory(headerString->c_str(), (int)headerString->length(), NULL, NULL, 0);

    if (xmlreader == NULL){
        //do some error handling
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "ERROR PARSING HEADER\n");
        status = asynError;
    }
    if(status == asynSuccess)
    {
        /*walk the xml nodes*/
        while ((xmlTextReaderRead(xmlreader)) == 1)
        {
            const xmlChar* xmlNodeName = NULL; // BK, can be defined on the same line, no NULL necesarry;
            /*get the node names*/
            xmlNodeName = xmlTextReaderConstName(xmlreader);
            if (xmlNodeName != NULL)
            {
                std::string name((const char*)xmlNodeName);
                /*get the attributes for the data and field nodes and put on vector*/
                if( name == "data" || name == "field")
                {
                    extractHeaderData(xmlreader, &tmpValues);
                    tmpHeaderValues.push_back(tmpValues);
                }
            }
        }
    }

    callParamCallbacks(); // BK: I don't think we are changing any of the params in this function
    return tmpHeaderValues;
}

asynStatus Pandabox::extractHeaderData(xmlTextReaderPtr xmlreader, std::map<std::string, std::string>* values)
{
    /*Get the attribute values for a node and place in the map values*/
    xmlNodePtr node= xmlTextReaderCurrentNode(xmlreader);
    if (xmlTextReaderNodeType(xmlreader)==1 && node && node->properties) {
        xmlAttr* attribute = node->properties;
        while(attribute && attribute->name && attribute->children)
        {
          xmlChar* xvalue = xmlNodeListGetString(node->doc, attribute->children, 1);

          /*Insert the values into the data_value map*/
          std::string value((const char*)xvalue);
          std::string name((const char*)attribute->name);
          (*values)[name] = value;

          xmlFree(xvalue);
          attribute = attribute->next;
        }
    }
    return asynSuccess;
}

std::string Pandabox::getHeaderValue(int index, std::string attribute)
{
    /*return the value of an attribute of a given element*/
    return headerValues[index].find(attribute)->second; // BK: What happens if not in map?
}

void Pandabox::getAllData(std::vector<char>* inBuffer, int dataLen, int buffLen)
{
    const char *functionName = "getAllData";
    size_t nBytesIn;
    size_t readBytes = dataLen - buffLen;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_data, 0, 0);
    char rxBuffer[readBytes];
    status = pasynOctet_data->read(octetPvt_data, pasynUserRead, rxBuffer, readBytes,
            &nBytesIn, &eomReason);

    inBuffer->insert(inBuffer->end(), rxBuffer, rxBuffer+nBytesIn);
    if(status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %d'\n",
                driverName, functionName, status);
    }
}

void Pandabox::parseData(std::vector<char> dataBuffer, int dataLen){
    const char *functionName = "parseData";
    int buffLen = dataBuffer.size(); //actual length of received input data stream (could be multiple lines)
    int dataNo = headerValues.size() - 1; //number of received data points (total header fields - 'data' = number of captured fields)
    //check to see if we have read all the data in, and do another read if we haven't
    if(dataLen > buffLen)
    {
        getAllData(&dataBuffer, dataLen, buffLen);
    }
    outputData(dataLen, dataNo, dataBuffer);
}

//void Pandabox::outputData(int dataLen, int dataNo, double* floatData)
void Pandabox::outputData(int dataLen, int dataNo, std::vector<char> data) // BK: data vector must be declared const even if you pass it to value to ensure that it is not reallocated while you are iterating over it with raw pointers (it should not be but iterating over nonconst vector with raw pointers is undefined behaviour)
{
    int linecount = 0; //number of lines of data received and parsed
    //get the length of an individual dataSet
    int setLen = 0;
    for(int i = 0; i < headerValues.size()-1; i++) // BK : ++i
    {
        if(getHeaderValue(i+1, "type") == "double")
        {
            setLen += sizeof(double);
        }
        else if (getHeaderValue(i+1, "type") == "uint32")
        {
            setLen += sizeof(uint32_t);
        }
    }
    int idx = 0;
    int ptridx = 0; //skip the first 8 bytes
    int noDataSets = data.size() / setLen; //number of data sets in the received binary data
    double* doubleData = (double*) &data[ptridx];
    uint32_t* uint32Data = (uint32_t*) &data[ptridx];
    std::string dataType;
    //find other possible data types..

    //loop over the data sets in the received data
    for(int j = 0; j < noDataSets; j++) // BK: ++j
    {
        //allocate a frame for each data set
        allocateFrame();
        if(this->pArray != NULL) {
            //loop over each data point in the data set
            for(int i = 0; i < dataNo; i++) // BK: ++i
            {
                idx = (j*dataNo + i);//current data point index in the float array
                    // NDAttributes are used to store the actual captured data
                    std::string desc("sample value");
                    NDAttrSource_t sourceType = NDAttrSourceUndefined;
                    const char *pSource = "source string";
                    //find out what type the individual point is
                    //from the header and assign the approperiate pointer.
                    dataType = getHeaderValue(i+1, "type");
                    if(dataType == "double")
                    {
                    // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        NDAttribute *pAttribute = new NDAttribute(
                            getHeaderValue(i+1, "name").c_str(),
                            desc.c_str(), sourceType,
                            pSource, NDAttrFloat64,
                            &doubleData[0]); // BK: just doubleData, already a pointer to an element
                        this->pArray->pAttributeList->add(pAttribute);
                        ((double *)this->pArray->pData)[i] = doubleData[0];
                        ptridx += sizeof(double);
                    }
                    else if(dataType == "uint32")
                    {
                                        // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        NDAttribute *pAttribute = new NDAttribute(
                            getHeaderValue(i+1, "name").c_str(),
                            desc.c_str(), sourceType,
                            pSource, NDAttrUInt32,
                            &uint32Data[0]); // BK: just uint32Data, already a pointer to an element
                        this->pArray->pAttributeList->add(pAttribute);
                        ((uint32_t *)this->pArray->pData)[i] = uint32Data[0];
                        ptridx += sizeof(uint32_t);
                    };
                    doubleData = (double*) &data[ptridx]; // BK: you can just have one pointer and cast it according to the data, increasing it according to the data
                    uint32Data = (uint32_t*) &data[ptridx];
            }
        }
        /* Ship off the NDArray*/
        wrapFrame();

        /* Increment number of lines processed*/
        linecount++;
        callParamCallbacks();
    }
}

void Pandabox::allocateFrame() {
    // Release the old NDArray if it exists
    if (this->pArray != NULL) {
        this->pArray->release();
        this->pArray = NULL;
    }
    // Allocate a new NDArray
    int arraysize = headerValues.size() -1;
    size_t dims[2];
    int nDims = 2;
    dims[0] = arraysize;
    dims[1] = FRAMEHEIGHT;
    this->pArray = this->pNDArrayPool->alloc(nDims, dims, NDFloat64, 0, NULL);
    //clear the attribute list to get rid of previous scans
    this->pArray->pAttributeList->clear();
}

void Pandabox::wrapFrame() {
    if(this->capture)
    {
        getIntegerParam(NDArrayCounter, &(this->arrayCounter));
        getIntegerParam(ADNumImagesCounter, &(this->numImagesCounter));
        // Set the time stamp
        epicsTimeStamp arrayTime;
        epicsTimeGetCurrent(&arrayTime);
        this->pArray->timeStamp = arrayTime.secPastEpoch;
        // Save the NDAttributes if there are any
        getAttributes(this->pArray->pAttributeList);
        // Update statistics
        this->arrayCounter++;
        this->numImagesCounter++;
        //send disarm signal if we are in a mode that requires it
        if(this->imgMode == ADImageSingle &&  this->arrayCounter == 1)
        {
            this->capture = false;
        }
        else if(this->imgMode == ADImageMultiple && this->arrayCounter == this->imgNo)
        {
            this->capture = false;
        }
        // Set the unique ID
        this->pArray->uniqueId = this->arrayCounter;
        // Ship the array off
        doCallbacksGenericPointer(this->pArray, NDArrayData, 0);
        // Update the counters
        setIntegerParam(NDArrayCounter, this->arrayCounter);
        setIntegerParam(ADNumImagesCounter, this->numImagesCounter);
    }
    if(!this->capture)
    {
        std::string cmdString = "*PCAP.DISARM="; // BK: unnecessary local
        sendCtrl(cmdString);
        this->setIntegerParam(ADAcquire, 0);
        endCapture();
	callParamCallbacks();
    }
}

/* Send helper function
 * called with lock taken
 */
asynStatus Pandabox::sendData(std::string txBuffer) {
    asynStatus status = send(txBuffer, pasynOctet_data, octetPvt_data, pasynUser_data);
    return status;
}

asynStatus Pandabox::sendCtrl(std::string txBuffer) {
    asynStatus status = send(txBuffer, pasynOctet_ctrl, octetPvt_ctrl, pasynUser_ctrl);
    return status;
}

asynStatus Pandabox::send(std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser) {
    const char *functionName = "send";
    asynStatus status = asynSuccess;
    int connected;
    size_t nBytesOut;
    pasynUser->timeout = TIMEOUT;
    status = pasynOctet->write(octetPvt, pasynUser, txBuffer.c_str(), txBuffer.length(),
            &nBytesOut);
    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: Send: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
    if (status != asynSuccess) {
        // Can't write, port probably not connected
        getIntegerParam(pandaboxIsConnected, &connected);
        if (connected) {
            setIntegerParam(pandaboxIsConnected, 0);
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Can't write to pandabox: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
        }
    }
    return status;
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus Pandabox::writeInt32(asynUser *pasynUser, epicsInt32 value) {
    const char *functionName = "writeInt32";
    asynStatus status = asynError;

    /* Any work we need to do */
    int param = pasynUser->reason;

    status = this->setIntegerParam(param, value);
    //change writing depending on imagemode
    if(param == ADImageMode)
    {
        this->imgMode = value;
    }

    if(param == ADNumImages)
    {
        this->imgNo = value;
    }

    if(param == ADAcquire)
    {
        if(value)
        {
            //set the current array number
            this->capture = true;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, // BK, Trace error is probably not the right value?
                    "SEND ARM CMD:\n");
            std::string cmdString = "*PCAP.ARM="; // BK, unnecessary local
            sendCtrl(cmdString);
        }
        else
        {
            this->capture = false;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, // BK, Trace error is probably not the right value?
                    "SEND DISARM CMD:\n");
            std::string cmdString = "*PCAP.DISARM="; // BK, unnecessary local
            sendCtrl(cmdString);
            endCapture();
        }
    }

    if(status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
                driverName, functionName);
    }

    callParamCallbacks();
    return status;
}


extern "C" int pandaboxConfig(const char *portName, const char* cmdSerialPortName,
        const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory) {
    new Pandabox(portName, cmdSerialPortName, dataSerialPortName, maxPts, maxBuffers, maxMemory);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg pandaboxConfigArg0 = { "Port name", iocshArgString };
static const iocshArg pandaboxConfigArg1 = { "Cmd Serial port name", iocshArgString };
static const iocshArg pandaboxConfigArg2 = { "Data Serial port name", iocshArgString };
static const iocshArg pandaboxConfigArg3 = { "Max number of points to capture in position compare", iocshArgInt };
static const iocshArg pandaboxConfigArg4 = { "maxBuffers for areaDetector", iocshArgInt };
static const iocshArg pandaboxConfigArg5 = { "maxMemory for areaDetector", iocshArgInt };
static const iocshArg* const pandaboxConfigArgs[] = { &pandaboxConfigArg0,
        &pandaboxConfigArg1, &pandaboxConfigArg2, &pandaboxConfigArg3, &pandaboxConfigArg4, &pandaboxConfigArg5  };
static const iocshFuncDef configpandabox = { "pandaboxConfig", 6, pandaboxConfigArgs };
static void configpandaboxCallFunc(const iocshArgBuf *args) {
    pandaboxConfig(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}

static void pandaboxRegister(void) {
    iocshRegister(&configpandabox, configpandaboxCallFunc);
}

extern "C" {
epicsExportRegistrar(pandaboxRegister);
}
