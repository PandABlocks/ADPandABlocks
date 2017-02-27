// BK: const modifiers on input parameters and methods are missing
// BK: Prefer const references in params instead of pointers where the passed 
//     objects are not changed, references if the memory position of the object
//     is not changed in method (the reference cannot be null), pointers are 
//     usually reserved for output parameters

#include "pandabox.h"

#include <stdint.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <stdexcept>
#include <iostream>

#include <libxml/xmlreader.h>

#include <epicsEndian.h>
#include "epicsThread.h"


/* C function to call new message from  task from epicsThreadCreate */
static void readTaskCtrlC(void *userPvt) {
    Pandabox *pPvt = (Pandabox *) userPvt;
    pPvt->readTaskCtrl();
}

static void readTaskDataC(void *userPvt) {
    Pandabox *pPvt = (Pandabox *) userPvt;
    pPvt->readTaskData();
}

typedef int static_assert_endianness[EPICS_BYTE_ORDER != EPICS_ENDIAN_BIG ? 1 : -1];

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
    pArray = NULL;
    arrayCounter = 0;
    numImagesCounter = 0;
    imgMode = ADImageContinuous;
    imgNo = 0;
    capture = true;
    readBytes = N_BUFF_DATA-1;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &pandaboxIsConnected);
    setIntegerParam(pandaboxIsConnected, 0);

    /*Create a parameter to store the header value */
    createParam("HEADER", asynParamOctet, &pandaboxHeader);
    
    /*Create a parameter to store the end of data string */
    createParam("DATAEND", asynParamOctet, &pandaboxDataEnd);
    setStringParam(pandaboxDataEnd, "");
    
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, cmdSerialPortName, status);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_ctrl, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }


    /* Connect to the data port */
    /* Copied from asynOctecSyncIO->connect */
    pasynUser_data = pasynManager->createAsynUser(0, 0);
    status = pasynManager->connectDevice(pasynUser_data, dataSerialPortName, 0);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, dataSerialPortName, status);
        return;
    }
    pasynInterface = pasynManager->findInterface(pasynUser_data, asynCommonType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynCommon_data = (asynCommon *) pasynInterface->pinterface;
    pcommonPvt_data = pasynInterface->drvPvt;
    pasynInterface = pasynManager->findInterface(pasynUser_data, asynOctetType, 1);
    if (!pasynInterface) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }

    /*set the receiving format on the data channel*/
    sendData("XML FRAMED SCALED\n");
};

/* This is the function that will be run for the read thread */
void Pandabox::readTaskCtrl() {
    const char *functionName = "readTaskCtrl";
    //char *rxBuffer;
    char rxBuffer[N_BUFF_CTRL];
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);

    while (true) {
        pasynUserRead->timeout = LONGWAIT;
        status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
                &nBytesIn, &eomReason);
        if (status) {
            epicsThreadSleep(TIMEOUT);
        } else if (eomReason & ASYN_EOM_EOS) {
            // Replace the terminator with a null so we can use it as a string
            rxBuffer[nBytesIn] = '\0';
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
        }
    }
}
asynStatus Pandabox::readHeaderLine(char* rxBuffer, const size_t buffSize) const {
    /*buffSize is the size fo rxBuffer*/
    const char *functionName = "readHeaderLine";
    int eomReason;
    size_t nBytesIn;
    asynStatus status = asynTimeout;
    //check to see if rxBuffer is
    while (status == asynTimeout) {
        status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, 
                buffSize, &nBytesIn, &eomReason);
    }
    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
                driverName, functionName, errorMsg[status].c_str());
    }
    if (eomReason != ASYN_EOM_EOS) {
         throw std::runtime_error("attribute not in map");
    }
    //assert (eomReason == ASYN_EOM_EOS); // BK: I would also log this before asserting, just to make debugging easier if it happens
    return status;
}

asynStatus Pandabox::readDataBytes(char* rxBuffer, const size_t nBytes) const {
    const char *functionName = "readDataBytes";
    int eomReason;
    size_t nBytesIn;
    asynStatus status = asynTimeout;

    while (status == asynTimeout) {
        status = pasynOctet_data->read(octetPvt_data, pasynUser_data, rxBuffer, 
                nBytes, &nBytesIn, &eomReason);
    }

    if(status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
                driverName, functionName, errorMsg[status].c_str());
    }
    if(nBytes != nBytesIn) {
         throw std::runtime_error("attribute not in map");
    }
   // assert (nBytes == nBytesIn); // BK: I would also log this before asserting, just to make debugging easier if it happens
    return status;
}

/*this function reads from the data port*/
void Pandabox::readTaskData() {
    size_t nBytesIn;
    asynStatus status = asynSuccess;
    std::vector<char> dataPacket(0,0);
    char rxBuffer[N_BUFF_DATA];
    int eomReason;
    uint32_t dataLength = 0;
    try{
        while (true) {
            std::cout << "STATE: " << state << std::endl;
            switch(state) {
                case waitHeaderStart:
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    if (strcmp(rxBuffer, "<header>\0") == 0) {
                        //we have a header so we have started acquiring
                        setIntegerParam(ADAcquire, 1);
                        header.append(rxBuffer);
                        header.append("\n");
                        state = waitHeaderEnd;
                        callParamCallbacks();
                    }
                    break;

                case waitHeaderEnd:
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    /*accumulate the header until we reach the end, then process*/
                    header.append(rxBuffer);
                    header.append("\n");
                    if (strcmp(rxBuffer, "</header>\0") == 0) {
                        headerValues = parseHeader(header);
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
                    {
                        uint32_t message_length;
                        readDataBytes(reinterpret_cast<char *>(&message_length),
                                4);
                        dataLength = message_length - 8; // size of the packet prefix information is 8
                    }
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
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    setStringParam(pandaboxDataEnd, rxBuffer);
                    callParamCallbacks();
                    break;
            }
        }
    }
    catch(const std::runtime_error& e){
    //return to beginning state if there is an exception
        state = waitHeaderStart;
        status = asynError;
    }
    callParamCallbacks();
}

void Pandabox::endCapture()
{
    //check the next 4 bytes to see if it matches the total arrays read.
    //reset the header string
    header = "";
    //change the input eos back to newline for the header
    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
    //read the rest of the end of data string and update the param
    readBytes = N_BUFF_DATA-1; //reset the amount of bytes to read
    //set the acquire light to 0
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
}

Pandabox::headerMap Pandabox::parseHeader(const std::string& headerString)
{
/**return a map containing the header data corresponding to each xml node
 * the first map will always be the 'data' node,
 * then each field will be pushed onto the vector sequentially
 */
    std::map<std::string, std::string> tmpValues;
    headerMap tmpHeaderValues;

    //set the header parameter
    setStringParam(pandaboxHeader, headerString.c_str());

    asynStatus status = asynSuccess;
    xmlTextReaderPtr xmlreader = xmlReaderForMemory(headerString.c_str(), (int)headerString.length(), NULL, NULL, 0);

    if (xmlreader == NULL){
        //do some error handling
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "ERROR PARSING HEADER\n");
        status = asynError;
    }
    if(status == asynSuccess)
    {
        /*walk the xml nodes*/
        while ((xmlTextReaderRead(xmlreader)) == 1)
        {
            /*get the node names*/
            const xmlChar* xmlNodeName = xmlTextReaderConstName(xmlreader);
            if (xmlNodeName != NULL)
            {
                std::string name((const char*)xmlNodeName);
                /*get the attributes for the data and field nodes and put on vector*/
                if( name == "data" || name == "field")
                {
                    extractHeaderData(xmlreader, tmpValues);
                    tmpHeaderValues.push_back(tmpValues);
                }
            }
        }
    }
    xmlFreeTextReader(xmlreader);

    callParamCallbacks();
    return tmpHeaderValues;
}

asynStatus Pandabox::extractHeaderData(const xmlTextReaderPtr xmlreader, std::map<std::string, std::string>& values)const
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
          values[name] = value;

          xmlFree(xvalue);
          attribute = attribute->next;
        }
    }
    return asynSuccess;
}

std::string Pandabox::getHeaderValue(const int index, const std::string attribute)const
{
    /*return the value of an attribute of a given element*/
    //first check index (do find on headerValues
    if (headerValues[index].find(attribute) != headerValues[index].end())
    {
        return headerValues[index].find(attribute)->second;
    }
    else
    {
         throw std::out_of_range("attribute not in map");
    }
}

void Pandabox::getAllData(std::vector<char>& inBuffer, const int dataLen, const int buffLen)const
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

    inBuffer.insert(inBuffer.end(), rxBuffer, rxBuffer+nBytesIn);
    if(status)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %d'\n",
                driverName, functionName, status);
    }
}

void Pandabox::parseData(std::vector<char> dataBuffer, const int dataLen){
    int buffLen = dataBuffer.size(); //actual length of received input data stream (could be multiple lines)
    int dataNo = headerValues.size() - 1; //number of received data points (total header fields - 'data' = number of captured fields)
    //check to see if we have read all the data in, and do another read if we haven't
    if(dataLen > buffLen)
    {
        getAllData(dataBuffer, dataLen, buffLen);
    }
    outputData(dataLen, dataNo, dataBuffer);
}

void Pandabox::outputData(const int dataLen, const int dataNo, const std::vector<char> data)
{
    try{
        int linecount = 0; //number of lines of data received and parsed
        //get the length of an individual dataSet
        int setLen = 0;
        for(size_t i = 0; i < headerValues.size()-1; ++i)
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
        const char* ptridx = &data.front();
        int noDataSets = data.size() / setLen; //number of data sets in the received binary data
        std::string dataType;
        //find other possible data types..

        //loop over the data sets in the received data
        for(int j = 0; j < noDataSets; ++j)
        {
            //allocate a frame for each data set
            allocateFrame();
            if(pArray != NULL) {
                //loop over each data point in the data set
                for(int i = 0; i < dataNo; ++i)
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
                                (double*)ptridx);
                            pArray->pAttributeList->add(pAttribute);
                            ((double *)pArray->pData)[i] = *(double*)ptridx;
                            ptridx += sizeof(double);
                        }
                        else if(dataType == "uint32")
                        {
                                            // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                            NDAttribute *pAttribute = new NDAttribute(
                                getHeaderValue(i+1, "name").c_str(),
                                desc.c_str(), sourceType,
                                pSource, NDAttrUInt32,
                                (uint32_t*)ptridx);
                            pArray->pAttributeList->add(pAttribute);
                            uint32_t value = *(uint32_t*)ptridx;
                            std::cout << "VAL: " << value << ", " << (double)value << std::endl;
                            ((double*)pArray->pData)[i] = (double)value;
                            ptridx += sizeof(uint32_t);
                        };
                }
            }
            /* Ship off the NDArray*/
            wrapFrame();

            /* Increment number of lines processed*/
            linecount++;
            callParamCallbacks();
        }
    }
    catch(const std::out_of_range& e){
        //if attribute is not in header map, go back to beginning ?
    }
}

void Pandabox::allocateFrame() {
    // Release the old NDArray if it exists
    if (pArray != NULL) {
        pArray->release();
        pArray = NULL;
    }
    // Allocate a new NDArray
    int arraysize = headerValues.size() -1;
    size_t dims[2];
    int nDims = 2;
    dims[0] = arraysize;
    dims[1] = FRAMEHEIGHT;
    pArray = pNDArrayPool->alloc(nDims, dims, NDFloat64, 0, NULL);
    //clear the attribute list to get rid of previous scans
    if (pArray != NULL) {
        pArray->pAttributeList->clear();
    }
}

void Pandabox::wrapFrame() {
    if(capture)
    {
        this->lock();
        getIntegerParam(NDArrayCounter, &(arrayCounter));
        getIntegerParam(ADNumImagesCounter, &(numImagesCounter));
        // Set the time stamp
        epicsTimeStamp arrayTime;
        epicsTimeGetCurrent(&arrayTime);
        if (pArray != NULL) {
            pArray->timeStamp = arrayTime.secPastEpoch;
            // Save the NDAttributes if there are any
            getAttributes(pArray->pAttributeList);
        }
        // Update statistics
        arrayCounter++;
        numImagesCounter++;
        //send disarm signal if we are in a mode that requires it
        if(imgMode == ADImageSingle &&  arrayCounter == 1)
        {
            capture = false;
        }
        else if(imgMode == ADImageMultiple && numImagesCounter == imgNo)
        {
            capture = false;
        }
        // Set the unique ID
        if (pArray != NULL) {
            pArray->uniqueId = arrayCounter;
        }
        // Update the counters
        setIntegerParam(NDArrayCounter, arrayCounter);
        setIntegerParam(ADNumImagesCounter, numImagesCounter);
        this->unlock();
        if (pArray != NULL) {
            // Ship the array off
            doCallbacksGenericPointer(pArray, NDArrayData, 0);
        }
    }
    if(!capture)
    {
        sendCtrl("*PCAP.DISARM=");
        setIntegerParam(ADAcquire, 0);
        endCapture();
	callParamCallbacks();
    }
}

/* Send helper function
 * called with lock taken
 */
asynStatus Pandabox::sendData(const std::string txBuffer){
    asynStatus status = send(txBuffer, pasynOctet_data, octetPvt_data, pasynUser_data);
    return status;
}

asynStatus Pandabox::sendCtrl(const std::string txBuffer){
    asynStatus status = send(txBuffer, pasynOctet_ctrl, octetPvt_ctrl, pasynUser_ctrl);
    return status;
}

asynStatus Pandabox::send(const std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser) {
    const char *functionName = "send";
    asynStatus status = asynSuccess;
    int connected;
    size_t nBytesOut;
    pasynUser->timeout = TIMEOUT;
    status = pasynOctet->write(octetPvt, pasynUser, txBuffer.c_str(), txBuffer.length(),
            &nBytesOut);
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s:%s: Send: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
    if (status != asynSuccess) {
        // Can't write, port probably not connected
        getIntegerParam(pandaboxIsConnected, &connected);
        if (connected) {
            setIntegerParam(pandaboxIsConnected, 0);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
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

    status = setIntegerParam(param, value);
    //change writing depending on imagemode
    if(param == ADImageMode)
    {
        imgMode = value;
    }

    if(param == ADNumImages)
    {
        imgNo = value;
    }

    if(param == ADAcquire)
    {
        if(value)
        {
            //set the current array number
            capture = true;
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
                    "SEND ARM CMD:\n");
            sendCtrl("*PCAP.ARM=");
            setIntegerParam(ADNumImagesCounter, 0);
        }
        else
        {
            asynPrint(pasynUserSelf, ASYN_TRACEIO_DRIVER,
                    "SEND DISARM CMD:\n");
            sendCtrl("*PCAP.DISARM=");
            endCapture();
            capture = false;
        }
    }

    if(status)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
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
