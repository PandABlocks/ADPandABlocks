// BK: const modifiers on input parameters and methods are missing
// BK: Prefer const references in params instead of pointers where the passed 
//     objects are not changed, references if the memory position of the object
//     is not changed in method (the reference cannot be null), pointers are 
//     usually reserved for output parameters

#include "ADPandABlocks.h"

#include <stdint.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <stdexcept>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <stdlib.h>

#include <libxml/xmlreader.h>

#include <epicsEndian.h>
#include "epicsThread.h"

//#include <unistd.h>
#include <cstdlib>


static void readTaskDataC(void *userPvt) {
    ADPandABlocks *pPvt = (ADPandABlocks *) userPvt;
    pPvt->readTaskData();
}

static void checkPosBusChangesC(void *userPvt) {
    ADPandABlocks *pPvt = (ADPandABlocks *) userPvt;
    pPvt->checkPosBusChanges();
}

typedef int static_assert_endianness[EPICS_BYTE_ORDER != EPICS_ENDIAN_BIG ? 1 : -1];

static const char *driverName = "ADPandABlocks";
static std::map<asynStatus, std::string> errorMsg;

ADPandABlocks::ADPandABlocks(const char* portName, const char* cmdSerialPortName, const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory) :
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

    captureType["No"] = 0;
    captureType["Triggered"] = 1;
    captureType["Difference"] = 2;
    captureType["Average"] = 3;
    captureType["Extended"] = 4;
    captureStrings.push_back("No");
    captureStrings.push_back("Triggered");
    captureStrings.push_back("Difference");
    captureStrings.push_back("Average");
    captureStrings.push_back("Extended");

    const char *functionName = "ADPandABlocks";
    asynStatus status = asynSuccess;
    asynInterface *pasynInterface;

    /* For areaDetector image */
    pArray = NULL;
    arrayCounter = 0;
    numImagesCounter = 0;
    imgMode = ADImageContinuous;
    imgNo = 0;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &ADPandABlocksIsConnected);
    setIntegerParam(ADPandABlocksIsConnected, 0);

    /*Create a parameter to store the header value /*/
    createParam("HEADER", asynParamOctet, &ADPandABlocksHeader);

    /*Create a parameter to store the end of data string */
    createParam("DATAEND", asynParamOctet, &ADPandABlocksDataEnd);
    setStringParam(ADPandABlocksDataEnd, "");

    /* initialise areaDetector parameters */
    setStringParam(ADManufacturer, "Diamond Light Source Ltd.");
    setStringParam(ADModel, "ADPandABlocks");
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

    /*set the receiving format on the data channel*/
    sendData("XML FRAMED SCALED\n");
    /*Get the labels for the bitmask*/
    int numBitMasks = 0;
    for(int n=0; n<4; n++)
    {
        std::stringstream bitmaskCmd;
        bitmaskCmd << "PCAP.BITS"<< n << ".BITS?";
        sendCtrl(bitmaskCmd.str());
        bitMasks.push_back(readFieldNames(&numBitMasks));
    }

    /*Get the POSITION fields*/
    std::stringstream fieldCmd;
    fieldCmd << "*POSITIONS?";
    sendCtrl(fieldCmd.str());
    int numPosFields = 0;
    posFields.push_back(readFieldNames(&numPosFields));

    /*Make params for each of the POSITION fields*/
    char str[NBUFF];
    for(int a = 1; a <numPosFields; a++) //IGNORE THE FIRST VALUE AS IT IS POSITIONS.ZERO
    {
        epicsSnprintf(str, NBUFF, "POSBUS%d", a);
        createParam(str, asynParamOctet, &ADPandABlocksPosFields[a]);
        setStringParam(ADPandABlocksPosFields[a], posFields[0][a].c_str());
    }

    //Initialise the lookup table for posbus values
    int posBusInd = 0;
    for(std::vector<std::string>::iterator it = posFields[0].begin(); it != posFields[0].end(); ++it)
    {
        initLookup(*it, "SCALE", &ADPandABlocksScale[posBusInd], posBusInd);
        initLookup(*it, "OFFSET", &ADPandABlocksOffset[posBusInd], posBusInd);
        initLookup(*it, "UNITS", &ADPandABlocksUnits[posBusInd], posBusInd);
        initLookup(*it, "CAPTURE", &ADPandABlocksCapture[posBusInd], posBusInd);
        posBusInd++;
    }

    /* Create the thread to monitor posbus changes */

    if (epicsThreadCreate("ADPandABlockscheckPosBusChanges", epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) checkPosBusChangesC, this) == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }

    /* Create the thread that reads from the device  */
    if (epicsThreadCreate("ADPandABlocksReadTask2", epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) readTaskDataC, this) == NULL) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }
};

std::string ADPandABlocks::getPosBusField(std::string posbus, const char* paramName){
    std::string field;
    std::stringstream cmdStr;
    cmdStr << posbus << "." << paramName <<"?";
    sendCtrl(cmdStr.str());
    readPosBusValues(&field);
    return field;
}

void ADPandABlocks::createPosBusParam(const char* paramName, asynParamType paramType, int* paramIndex, int paramNo){
    this->lock();
    char str[NBUFF];
    epicsSnprintf(str, NBUFF, "POSBUS%d:%s", paramNo, paramName);
    createParam(str, paramType, paramIndex);
    this->unlock();
}

void ADPandABlocks::initLookup(std::string paramName, std::string paramNameEnd, int* paramInd, int posBusInd)
{
    std::map<std::string, int*> lpMap2;
    if(paramNameEnd == "CAPTURE"){
        createPosBusParam(paramNameEnd.c_str(), asynParamInt32, paramInd, posBusInd);
    }
    else{
        createPosBusParam(paramNameEnd.c_str(), asynParamOctet, paramInd, posBusInd);
    }
    posBusLookup.insert(std::pair<std::string, std::map<std::string, int*> >(paramName, lpMap2));
    posBusLookup[paramName].insert(std::pair<std::string, int*>(paramNameEnd, paramInd));
    std::string paramVal = getPosBusField(paramName, paramNameEnd.c_str());
    asynStatus status = setStringParam(*posBusLookup[paramName][paramNameEnd], paramVal.c_str());
}

/* This is the function that will be run for the read thread */
std::vector<std::string> ADPandABlocks::readFieldNames(int* numFields) {
    this->lock();
    const char *functionName = "readFieldNames";
    char rxBuffer[N_BUFF_CTRL];
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);
    std::vector<std::string> fieldNameStrings;
    pasynUserRead->timeout = 3.0;
    status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
            &nBytesIn, &eomReason);
    int i = 0;
    while(rxBuffer[0] != '.')
    {
        if (strlen(rxBuffer) == 0) break;
        i++;
        if (eomReason & ASYN_EOM_EOS) {
            // Replace the terminator with a null so we can use it as a string
            rxBuffer[nBytesIn] = '\0';
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
        }
        if(rxBuffer[0] == '!')
        {
            char* strippedLabel = rxBuffer + 1;
            fieldNameStrings.push_back(strippedLabel);
        }
        // Push the whole bitmask for 'n' to the vector of vectors
        status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
                &nBytesIn, &eomReason);
    }
   *numFields = i;
   this->unlock();
   return fieldNameStrings;
}

/* This is the function that will be run for the read thread */
asynStatus ADPandABlocks::readPosBusValues(std::string* posBusValue) {
    const char *functionName = "readPosBusValues";
    char rxBuffer[N_BUFF_CTRL];
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);
    //std::string posBusValue;
    pasynUserRead->timeout = 3.0;
    status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, N_BUFF_CTRL - 1,
            &nBytesIn, &eomReason);
    if (eomReason & ASYN_EOM_EOS) {
        // Replace the terminator with a null so we can use it as a string
        rxBuffer[nBytesIn] = '\0';
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
    }

    if(rxBuffer[0] == 'O' && rxBuffer[1] == 'K')
    {
        char* readValue = rxBuffer + 4;
        posBusValue->assign(readValue);
    } else {
        //we didn't recieve OK so something went wrong
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Bad response 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
        status = asynError;
    }
    return status;
}

void ADPandABlocks::checkPosBusChanges(){
    /*  This will check if anything has changed on the panada and update the
        Readback values */
    std::vector<std::vector<std::string> > changedFields;
    std::vector<std::string> changedField;
    //query the pandabox to see if any changes have occured
    std::stringstream cmd;
    cmd << "*CHANGES.ATTR?";
    while(true)
    {
        this->lock();
        int numChanges = 0;
        sendCtrl(cmd.str());
        changedFields.clear();
        changedFields.push_back(readFieldNames(&numChanges));
        for(std::vector<std::string>::iterator it = changedFields[0].begin(); it != changedFields[0].end(); ++it)
        {
            std::vector<std::string> changedFieldsParts = stringSplit(*it, '.');
            std::stringstream  posBusName;
            posBusName << changedFieldsParts[0] << "." << changedFieldsParts[1];
            changedField.clear();
            changedField = stringSplit(changedFieldsParts[2], '=');
            std::string fieldName = changedField[0];
            std::string fieldVal = "";
            if(changedField.size() == 2)
            {
                fieldVal = changedField[1];
            }
            if(posBusLookup.find(posBusName.str()) != posBusLookup.end())
            {
                if(posBusLookup[posBusName.str()].find(fieldName) != posBusLookup[posBusName.str()].end())
                {
                    if(fieldName == "CAPTURE")
                    {
                        setIntegerParam(*posBusLookup[posBusName.str()][fieldName], captureType[fieldVal.c_str()]);
                    }
                    else
                    {
                        setStringParam(*posBusLookup[posBusName.str()][fieldName], fieldVal.c_str());
                    }
                    callParamCallbacks();
                }
            }
        }
        this->unlock();
        epicsThreadSleep(1);
    }
    callParamCallbacks();
}

std::vector<std::string> ADPandABlocks::stringSplit(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

asynStatus ADPandABlocks::readHeaderLine(char* rxBuffer, const size_t buffSize) const {
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: failed on 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
    }
    assert (eomReason == ASYN_EOM_EOS);
    return status;
}

asynStatus ADPandABlocks::readDataBytes(char* rxBuffer, const size_t nBytes) const {
    const char *functionName = "readDataBytes";
    int eomReason;
    size_t nBytesIn = 0;
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
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Only got %d bytes, not %d bytes with EOM reason %d\n",
				driverName, functionName, (int)nBytesIn, nBytes, eomReason);
    }
    assert (nBytes == nBytesIn);
    return status;
}

/*this function reads from the data port*/
void ADPandABlocks::readTaskData() {
    asynStatus status = asynSuccess;
    char rxBuffer[N_BUFF_DATA];
    std::string header;
    try{
        while (true) {
            switch(state) {
                case waitHeaderStart:
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    if (strcmp(rxBuffer, "<header>\0") == 0) {
                        //we have a header so we have started acquiring
                        setIntegerParam(ADAcquire, 1);
                        header.append(rxBuffer);
                        header.append("\n");
                        callParamCallbacks();
                        state = waitHeaderEnd;
                    }
                    break;

                case waitHeaderEnd:
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    /*accumulate the header until we reach the end, then process*/
                    header.append(rxBuffer);
                    header.append("\n");
                    if (strcmp(rxBuffer, "</header>\0") == 0) {
                        headerValues = parseHeader(header);
                        // Read the last line of the header
                        readHeaderLine(rxBuffer, N_BUFF_DATA);

                        //change the input eos as the data isn't terminated with a newline
                        pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "", 0);
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
                        readDataBytes(reinterpret_cast<char *>(&message_length), 4);
                        uint32_t dataLength = message_length - 8; // size of the packet prefix information is 8
                        // read the rest of the packet
                        readDataBytes(rxBuffer, dataLength);
                        std::vector<char> dataPacket;
                        dataPacket.insert(dataPacket.begin(), rxBuffer, rxBuffer + dataLength);
                        parseData(dataPacket, dataLength);
                        state = waitDataStart;
                    }
                    break;

                case dataEnd:
                    //reset the header string
                    header = "";
                    //change the input eos back to newline for the header
                    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
                    //set the acquire light to 0
                    setIntegerParam(ADAcquire, 0);
                    readHeaderLine(rxBuffer, N_BUFF_DATA);
                    setStringParam(ADPandABlocksDataEnd, rxBuffer);
                    callParamCallbacks();
                    state = waitHeaderStart;
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


ADPandABlocks::headerMap ADPandABlocks::parseHeader(const std::string& headerString)
{
/**return a map containing the header data corresponding to each xml node
 * the first map will always be the 'data' node,
 * then each field will be pushed onto the vector sequentially
 */
    std::map<std::string, std::string> tmpValues;
    headerMap tmpHeaderValues;

    //set the header parameter
    setStringParam(ADPandABlocksHeader, headerString.c_str());

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

asynStatus ADPandABlocks::extractHeaderData(const xmlTextReaderPtr xmlreader, std::map<std::string, std::string>& values)const
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

std::string ADPandABlocks::getHeaderValue(const int index, const std::string attribute)const
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

void ADPandABlocks::getAllData(std::vector<char>& inBuffer, const int dataLen, const int buffLen)const
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

void ADPandABlocks::parseData(std::vector<char> dataBuffer, const int dataLen){
    int buffLen = dataBuffer.size(); //actual length of received input data stream (could be multiple lines)
    int dataNo = headerValues.size() - 1; //number of received data points (total header fields - 'data' = number of captured fields)
    //check to see if we have read all the data in, and do another read if we haven't
    if(dataLen > buffLen)
    {
        getAllData(dataBuffer, dataLen, buffLen);
    }
    outputData(dataLen, dataNo, dataBuffer);
}

void ADPandABlocks::outputData(const int dataLen, const int dataNo, const std::vector<char> data)
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
        	// are we still acquiring?
        	int acquiring;
        	getIntegerParam(ADAcquire, &acquiring);
        	if (!acquiring) {
        		return;
        	}

            // allocate a frame for each data set
            allocateFrame();
            if(pArray != NULL) {
                //loop over each data point in the data set
                for(int i = 0; i < dataNo; ++i)
                {
                    idx = (j*dataNo + i);//current data point index in the float array
                    // NDAttributes are used to store the actual captured data
                    std::string desc("sample value");
                    //find out what type the individual point is
                    //from the header and assign the approperiate pointer.
                    dataType = getHeaderValue(i+1, "type");
                    if(dataType == "double")
                    {
                    // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        pArray->pAttributeList->add(
                                getHeaderValue(i+1, "name").c_str(),
                                desc.c_str(),
                                NDAttrFloat64,
                                (double*)ptridx);
                        ((double *)pArray->pData)[i] = *(double*)ptridx;
                        ptridx += sizeof(double);
                    }
                    else if(dataType == "uint32")
                    {
                        // Create the NDAttributes and initialise them with data value (headerValue[0] is the data info)
                        pArray->pAttributeList->add(
                                getHeaderValue(i+1, "name").c_str(),
                                desc.c_str(),
                                NDAttrUInt32,
                                (uint32_t*)ptridx);
                        uint32_t value = *(uint32_t*)ptridx;
                        ((double*)pArray->pData)[i] = (double)value;

                        //Determine if we need to populate the Bit Mask values
                        std::string headerLabel = getHeaderValue(i+1, "name");
                        size_t bitsFound = headerLabel.find("BITS");
                        if(bitsFound != std::string::npos)
                        {
                            int blockNum = atoi(headerLabel.substr(bitsFound + 4, 1).c_str());
                            uint8_t maskPtr;
                            std::string desc("bit mask");
                            if(pArray != NULL) {
                                for(int maski = 0; maski <32; maski++)
                                {
                                    //shift and mask the value and push into individual NDAttrs
                                    maskPtr = (value >> maski) & 0x01;
                                    pArray->pAttributeList->add(
                                            bitMasks[blockNum][maski].c_str(),
                                            desc.c_str(),
                                            NDAttrUInt8,
                                            &maskPtr);
                                }
                            }
                        }
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

void ADPandABlocks::allocateFrame() {
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

void ADPandABlocks::wrapFrame() {
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
	if ((imgMode == ADImageSingle && arrayCounter == 1) ||
		(imgMode == ADImageMultiple && numImagesCounter == imgNo)) {
		sendCtrl("*PCAP.DISARM=");
		setIntegerParam(ADAcquire, 0);
	}
	// Set the unique ID
	if (pArray != NULL) {
		pArray->uniqueId = arrayCounter;
	}
	// Update the counters
	setIntegerParam(NDArrayCounter, arrayCounter);
	setIntegerParam(ADNumImagesCounter, numImagesCounter);
	callParamCallbacks();
	this->unlock();
	if (pArray != NULL) {
		// Ship the array off
		doCallbacksGenericPointer(pArray, NDArrayData, 0);
	}
}

/* Send helper function
 * called with lock taken
 */
asynStatus ADPandABlocks::sendData(const std::string txBuffer){
    asynStatus status = send(txBuffer, pasynOctet_data, octetPvt_data, pasynUser_data);
    return status;
}

asynStatus ADPandABlocks::sendCtrl(const std::string txBuffer){
    asynStatus status = send(txBuffer, pasynOctet_ctrl, octetPvt_ctrl, pasynUser_ctrl);
    return status;
}

asynStatus ADPandABlocks::send(const std::string txBuffer, asynOctet *pasynOctet, void* octetPvt, asynUser* pasynUser) {
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
        getIntegerParam(ADPandABlocksIsConnected, &connected);
        if (connected) {
            setIntegerParam(ADPandABlocksIsConnected, 0);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Can't write to ADPandABlocks: '%.*s'\n", driverName, functionName, (int)txBuffer.length(), txBuffer.c_str());
        }
    }
    return status;
}

/** Called when asyn clients call pasynInt32->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ADPandABlocks::writeInt32(asynUser *pasynUser, epicsInt32 value) {
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
        }
    }
    else //handle the capture menu
    {
        for(std::map<std::string, std::map<std::string, int*> >::iterator it = posBusLookup.begin();
                it != posBusLookup.end(); ++it)
        {
            for(std::map<std::string, int*>::iterator it2 = it->second.begin();it2 != it->second.end(); ++it2)
            {
                if(param == *it2->second)
                {
                    std::stringstream cmdStr;
                    cmdStr << it->first <<"."<< it2->first <<"="<<captureStrings[value];
                    this->lock();
                    sendCtrl(cmdStr.str());
                    std::string field;
                    asynStatus status = readPosBusValues(&field);
                    if(status == asynError)
                    {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
                                driverName, functionName, cmdStr.str());
                    }
                    this->unlock();
                }
            }
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

/** Called when asyn clients call pasynOctet->write().
 * This function performs actions for some parameters
 * For all parameters it sets the value in the parameter library and calls any registered callbacks..
 * \param[in] pasynUser pasynUser structure that encodes the reason and address.
 * \param[in] value Value to write. */
asynStatus ADPandABlocks::writeOctet(asynUser *pasynUser, const char* value, size_t nChars, size_t* nActual) {
    /* This will check if a user has changed a value and attempt to update the
     * panda. It will also update the readback values */
    const char *functionName = "writeOctet";
    asynStatus status = asynError;
    this->lock();
    /* Any work we need to do */
    int param = pasynUser->reason;
    // Before setting any param - send it to the Panda and make sure the
    // response is OK
    // find the correct param
    if(posBusLookup.empty() == false)
    {
        for(std::map<std::string, std::map<std::string, int*> >::iterator it = posBusLookup.begin();
                it != posBusLookup.end(); ++it)
        {
            for(std::map<std::string, int*>::iterator it2 = it->second.begin();it2 != it->second.end(); ++it2)
            {
                if(param == *it2->second)
                {
                    std::stringstream cmdStr;
                    cmdStr << it->first <<"."<< it2->first <<"="<<value;
                    sendCtrl(cmdStr.str());
                    std::string field;
                    asynStatus status = readPosBusValues(&field);
                    if(status == asynError)
                    {
                        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting: %s '\n",
                                driverName, functionName, cmdStr.str());
                    }
                }
            }
        }
    }
    callParamCallbacks();
    this->unlock();
    return status;
}


extern "C" int ADPandABlocksConfig(const char *portName, const char* cmdSerialPortName,
        const char* dataSerialPortName, int maxPts, int maxBuffers, int maxMemory) {
    new ADPandABlocks(portName, cmdSerialPortName, dataSerialPortName, maxPts, maxBuffers, maxMemory);
    return (asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADPandABlocksConfigArg0 = { "Port name", iocshArgString };
static const iocshArg ADPandABlocksConfigArg1 = { "Cmd Serial port name", iocshArgString };
static const iocshArg ADPandABlocksConfigArg2 = { "Data Serial port name", iocshArgString };
static const iocshArg ADPandABlocksConfigArg3 = { "Max number of points to capture in position compare", iocshArgInt };
static const iocshArg ADPandABlocksConfigArg4 = { "maxBuffers for areaDetector", iocshArgInt };
static const iocshArg ADPandABlocksConfigArg5 = { "maxMemory for areaDetector", iocshArgInt };
static const iocshArg* const ADPandABlocksConfigArgs[] = { &ADPandABlocksConfigArg0,
        &ADPandABlocksConfigArg1, &ADPandABlocksConfigArg2, &ADPandABlocksConfigArg3, &ADPandABlocksConfigArg4, &ADPandABlocksConfigArg5  };
static const iocshFuncDef configADPandABlocks = { "ADPandABlocksConfig", 6, ADPandABlocksConfigArgs };
static void configADPandABlocksCallFunc(const iocshArgBuf *args) {
    ADPandABlocksConfig(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}

static void ADPandABlocksRegister(void) {
    iocshRegister(&configADPandABlocks, configADPandABlocksCallFunc);
}

extern "C" {
epicsExportRegistrar(ADPandABlocksRegister);
}
