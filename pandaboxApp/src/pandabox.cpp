
#include <sstream>
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <errno.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdint.h>
#include <epicsString.h>
#include <epicsTime.h>
#include <epicsExport.h>
#include <iocsh.h>

#include <libxml/xmlreader.h>

#include "epicsThread.h"
#include "pandabox.h"


using namespace std;


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
    asynInterface *pasynInterface_data;
//    char buffer[6400]; /* 100 chars per element on sys bus is overkill... */
//    char str[NBUFF];
//    const reg *r;

    /* For areaDetector image */
    this->pArray = NULL;
    this->arrayCounter = 0;
    this->numImagesCounter = 0;
    this->imgMode = ADImageContinuous;
    this->imgNo = 0;
    this->arrayNumberStart = 0;
    this->capture = true;
    this->readBytes = NBUFF2-1;

    /* Connection status */
    createParam("ISCONNECTED", asynParamInt32, &pandaboxIsConnected);
    setIntegerParam(pandaboxIsConnected, 0);

    /*Create a parameter to store the header value */
    createParam("HEADER", asynParamOctet, &pandaboxHeader);

    /*test param to setup a capture experiment*/
    createParam("TESTPARAM", asynParamInt32, &testparam);

    /* initialise areaDetector parameters */
    setStringParam(ADManufacturer, "Diamond Light Source Ltd.");
    setStringParam(ADModel, "Pandabox");
    setIntegerParam(ADMaxSizeX, NARRAYS + 1);
    setIntegerParam(ADMaxSizeY, FRAMEHEIGHT);
//    setIntegerParam(NDDataType, 7);
    setIntegerParam(ADStatus, ADStatusIdle);
    setStringParam(ADStatusMessage, "Idle");

    /* Create a message queue to hold completed messages and interrupts */
    this->msgQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
    this->intQId = epicsMessageQueueCreate(NQUEUE, sizeof(char*));
    if (this->msgQId == NULL || this->intQId == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsMessageQueueCreate failure\n", driverName, functionName);
        return;
    }

    /* Connect to the device port */
    /* Copied from asynOctecSyncIO->connect */
    pasynUser_ctrl = pasynManager->createAsynUser(0, 0);
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
    pasynCommon_ctrl = (asynCommon *) pasynInterface->pinterface;
    pcommonPvt_ctrl = pasynInterface->drvPvt;
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
    pasynUser_data = pasynManager->createAsynUser(0, 0);
    status = pasynManager->connectDevice(pasynUser_data, dataSerialPortName, 0);
    if (status != asynSuccess) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: Connect failed, port=%s, error=%d\n", driverName, functionName, dataSerialPortName, status);
        return;
    }
    pasynInterface_data = pasynManager->findInterface(pasynUser_data, asynCommonType, 1);
    if (!pasynInterface_data) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynCommonType);
        return;
    }
    pasynCommon_data = (asynCommon *) pasynInterface_data->pinterface;
    pcommonPvt_data = pasynInterface_data->drvPvt;
    pasynInterface_data = pasynManager->findInterface(pasynUser_data, asynOctetType, 1);
    if (!pasynInterface_data) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: %s interface not supported", driverName, functionName, asynOctetType);
        return;
    }
    pasynOctet_data = (asynOctet *) pasynInterface_data->pinterface;
    octetPvt_data = pasynInterface_data->drvPvt;

    /* Set EOS and flush */
    pasynOctet_data->flush(octetPvt_data, pasynUser_data);
    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "\n", 1);
    pasynOctet_data->setOutputEos(octetPvt_data, pasynUser_data, "\n", 1);

    /* Create the thread that reads from the device  */
    if (epicsThreadCreate("PandaboxReadTask2", epicsThreadPriorityMedium,
            epicsThreadGetStackSize(epicsThreadStackMedium),
            (EPICSTHREADFUNC) readTaskDataC, this) == NULL) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: epicsThreadCreate failure for reading task\n", driverName, functionName);
        return;
    }

    /*set the receiving format on the data channel*/
    setDataFormat();

};


/* This is the function that will be run for the read thread */
/* This is the function that will be run for the read thread */
void Pandabox::readTaskCtrl() {
    const char *functionName = "readTaskCtrl";
    char *rxBuffer;
    size_t nBytesIn;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_ctrl, 0, 0);

    while (true) {
        pasynUserRead->timeout = LONGWAIT;
        /* Malloc some data to put the reply from pandabox. This is freed if there is an
         * error, otherwise it is put on a queue, and the receiving thread should free it
         */
        rxBuffer = (char *) malloc(NBUFF);
        status = pasynOctet_ctrl->read(octetPvt_ctrl, pasynUserRead, rxBuffer, NBUFF - 1,
                &nBytesIn, &eomReason);
        if (status) {
            epicsThreadSleep(TIMEOUT);
        } else if (eomReason & ASYN_EOM_EOS) {
            // Replace the terminator with a null so we can use it as a string
            rxBuffer[nBytesIn] = '\0';
            asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s:%s: Message: '%s'\n", driverName, functionName, rxBuffer);
            free(rxBuffer);
        } else {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s:%s: Bad message 'bt%.*s'\n", driverName, functionName, (int)nBytesIn, rxBuffer);
            free(rxBuffer);
        }
    }
}

/*this function reads from the data port*/
void Pandabox::readTaskData() {
    const char *functionName = "readTaskData";
    size_t nBytesIn, totalBytesRead;
    int eomReason;
    asynStatus status = asynSuccess;
    asynUser *pasynUserRead = pasynManager->duplicateAsynUser(pasynUser_data, 0, 0);
    int datalength = 0;
    std::vector<char>::iterator it;

    char rxBuffer[NBUFF2];
    while (true) {
        pasynUserRead->timeout = 0;//LONGWAIT;

        status = pasynOctet_data->read(octetPvt_data, pasynUserRead, rxBuffer, readBytes,
                &nBytesIn, &eomReason);
        if(status)
        {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error reading data: %s'\n",
                    driverName, functionName, errorMsg[status].c_str());
        }

        totalBytesRead = nBytesIn;
        cout << "TOTAL BYTES READ: " << totalBytesRead << endl;

        //convert to a string so it's easier to use
        std::vector<char> dataStream(rxBuffer, rxBuffer + nBytesIn);
        std::vector<char> dataPacket(0,0);

        if (status) {
            //printf("Port not connected\n");
            epicsThreadSleep(TIMEOUT);
        }
        switch(state)
        {
            case waitHeaderStart:
                if (std::search(dataStream.begin(), dataStream.end(), "<header>", "<header>" + strlen("<header>")) != dataStream.end())
                {
                    //we have a header so we have started acquiring
                    cout << "ACQUIRING HEADER " << endl;
                    this->setIntegerParam(ADAcquire, 1);
                    header.append(dataStream.begin(),dataStream.end());
                    state = waitHeaderEnd;
                    callParamCallbacks();
                }
                break;

            case waitHeaderEnd:
                cout << "WAITING HEADER END" << endl;
                /*accumulate the header until we reach the end, then process*/
                header.append(dataStream.begin(),dataStream.end());
                header.append("\n");
                if(header.find("</header>") != string::npos && (eomReason & ASYN_EOM_EOS))
                {
                    cout << "FOUND HEADER END" << endl;
                    headerValues = parseHeader(&header);
                    //change the input eos as the data isn't terminated with a newline
                    pasynOctet_data->setInputEos(octetPvt_data, pasynUser_data, "", 0);

                    //read the extra newline at the end of the header
                    status = pasynOctet_data->read(octetPvt_data, pasynUserRead, rxBuffer, 1,
                            &nBytesIn, &eomReason);
                 //   datalength = atoi(getHeaderValue(1, "sample_bytes").c_str());
//                    readBytes = (int)datalength + 8; //length of the data to be read + first 8 bytes + end character
                    readBytes = 4; //first 4 bytes of the input
                    state = waitDataEnd;
                }
                break;

            case waitDataEnd:
                /*when "BIN" is received, start rocessing data*/
                //if (eomReason & ASYN_EOM_CNT)
                //{
                    //search for "BIN " 
                    //get the iterator for the point where this is found
                    //read the next four bytes and put required amount of data
                        //into another buffer
                    //if don't have enough, get the next amount from the next
                    //input stream
                    //pass the full buffer to the function to process the data
                 //   cout << "READING DATA" << ", SIZE: "<< dataStream.size() << endl;
//               //     datalength = atoi(getHeaderValue(1, "sample_bytes").c_str());
                 //   it = std::search(dataStream.begin(), dataStream.end(), "BIN ", "BIN " + strlen("BIN "));
                
                
                 //   while(it!= dataStream.end())
                 //   {
                 //       int dataLen = (int) dataStream[it-dataStream.begin() ]; //get an int* to the data
                 //       cout << "FOUND: " << dataStream[it - dataStream.begin()] << ", " << *it << endl;
                 //       if(dataStream.size() > dataLen)
                 //       {
                 //           vector<char> tmpBuff(it, it + dataLen);
                 //           dataPacket.insert(dataPacket.end(), tmpBuff.begin(), tmpBuff.end());
                 //           cout << "EXPECTED DATA LENGTH: " << datalength << endl;
                 //           cout << "EXPECTED PACKET LENGTH: " << dataLen << endl;
                 //           cout << "PROCESSING SIZE: " << dataPacket.size() << endl;
                 //           parseData(dataPacket);
                 //           dataPacket.clear();
                 //       }
                 //       else if(dataStream.size() < dataLen)
                 //       {
                 //           vector<char> tmpBuff(it, dataStream.end());
                 //           dataPacket = tmpBuff;
                 //           break;
                 //           //we need to read another packet to get the rest of
                 //           //the data
                 //       }
                 //       it = std::search(it + datalength + 8, dataStream.end(), "BIN ", "BIN " + strlen("BIN "));
                 //   }
                //}

  //                if (std::search(dataStream.begin(), dataStream.begin()+4, "BIN ", "BIN " + strlen("BIN ")) != dataStream.begin()+4)
                    if(std::search(dataStream.begin(), dataStream.end(), "BIN ", "BIN " + strlen("BIN ")) != dataStream.end())
                    {

                        cout << "BIN DATA " << endl;
                        datalength = atoi(getHeaderValue(1, "sample_bytes").c_str());
                        //should read the next four bytes here instead of
                        //getting the length from the header..?
                        readBytes = (int)datalength;
                        status = pasynOctet_data->read(octetPvt_data, pasynUserRead, rxBuffer, readBytes,
                                &nBytesIn, &eomReason);
                        std::vector<char> binData(rxBuffer, rxBuffer + nBytesIn);
                        cout << "DATA IN( " << binData.size() << ") " << binData[0] << binData[1] << binData[5] << endl;
                        double* doubleData = (double*) &binData[4];
                        cout << "DOUBLE IN: " << doubleData[0] << ", " << doubleData[6]<< ", " << doubleData[7] <<endl;
                        parseData(binData);
                        //reset number of bytes to be read to 4
                        readBytes = 4;
                    }
                    else if(std::search(dataStream.begin(), dataStream.end(), "END ", "END " + strlen("END ")) != dataStream.end())
                    {
                        cout << "END OF DATA " << endl;
                        endCapture();
                    }
    //            }
                //else if (dataStream.size() > 4)//if (eomReason & ASYN_EOM_END)
                //{
                //    if(std::search(dataStream.begin(), dataStream.begin()+4, "END ", "END " + strlen("END ")) != dataStream.begin()+4)
                //    {
                //        endCapture();
                //    }
                //}
                break;
        }

//        else {
//            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Bad message '%.*s'\n",
//                    driverName, functionName, (int)nBytesIn, rxBuffer);
//        }
    }
    callParamCallbacks();
}

void  Pandabox::endCapture()
{
    //check the next 4 bytes to see if it matches the total arrays read.
    //reset the states
    this->state = waitHeaderStart;
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
    xmlTextReaderPtr xmlreader = NULL;

    xmlreader = xmlReaderForMemory(headerString->c_str(), (int)headerString->length(), NULL, NULL, 0);

    if (xmlreader == NULL){
        //do some error handling
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "ERROR PARSING HEADER\n");
        status = asynError;
    }
    if(status == asynSuccess)
    {
        /*walk the xml nodes*/
        while ((ret = xmlTextReaderRead(xmlreader)) == 1)
        {
            const xmlChar* xmlNodeName = NULL;
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

    callParamCallbacks();
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
          string value((const char*)xvalue);
          string name((const char*)attribute->name);
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
    return headerValues[index].find(attribute)->second;
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

void Pandabox::parseData(std::vector<char> dataBuffer){
/*Return True if the end is found, else return false*/
    const char *functionName = "parseData";
    int * intData = (int*) &dataBuffer.front(); //get an int* to the data
    int dataLen = intData[0] - 8; //second 4 bytes is transmitted length of the frame (data + first 8 bytes)
    int buffLen = dataBuffer.size(); //actual length of received input data stream (could be multiple lines)
    int dataNo = headerValues.size() - 1; //number of received data points (total header fields - 'data' = number of captured fields)
    cout << "dataLen: " << dataLen << ", buffLen: " << buffLen << ", dataNo: " << dataNo << endl;
    //check to see if we have read all the data in, and do another read if we haven't
    if(dataLen > buffLen)
    {
        getAllData(&dataBuffer, dataLen, buffLen);
    }
        outputData(dataLen, dataNo, dataBuffer);
}

//void Pandabox::outputData(int dataLen, int dataNo, double* floatData)
void Pandabox::outputData(int dataLen, int dataNo, vector<char> data)
{
        int linecount = 0; //number of lines of data received and parsed
        //get the length of an individual dataSet
        int setLen = 0;
        for(int i = 0; i < headerValues.size()-1; i++)
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
        int ptridx = 4; //skip the first 8 bytes
        int noDataSets = data.size() / setLen; //noPoints/ dataNo; //number of data sets in the received binary data
        double* doubleData = (double*) &data[ptridx];
        uint32_t* uint32Data = (uint32_t*) &data[ptridx];
        string dataType;
        //find other possible data types..

        //loop over the data sets in the received data
        for(int j = 0; j < noDataSets; j++)
        {
            //allocate a frame for each data set
            allocateFrame();
            if(this->pArray != NULL) {
                //loop over each data point in the data set
                for(int i = 0; i < dataNo; i++)
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
                                &doubleData[0]);
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
                                &uint32Data[0]);
                            this->pArray->pAttributeList->add(pAttribute);
                            ((uint32_t *)this->pArray->pData)[i] = uint32Data[0];
                            ptridx += sizeof(uint32_t);
                        };
                        doubleData = (double*) &data[ptridx];
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
        if(this->imgMode == ADImageSingle &&  this->arrayCounter == this->arrayNumberStart + 1)
        {
            this->capture = false;
        }
        else if(this->imgMode == ADImageMultiple && this->arrayCounter == this->arrayNumberStart + this->imgNo)
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
        string cmdString = "*PCAP.DISARM=";
        sendCtrl(cmdString);
        this->setIntegerParam(ADAcquire, 0);
        endCapture();
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


void Pandabox::setDataFormat() {
    /* set the data header to be in XML, BINARY, SCALED format*/
    string formatString = "XML FRAMED SCALED\n";
    sendData(formatString);
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
            this->arrayNumberStart = this->arrayCounter;
            this->capture = true;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "SEND ARM CMD:\n");
            string cmdString = "*PCAP.ARM=";
            sendCtrl(cmdString);
        }
        else
        {
            this->capture = false;
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                    "SEND DISARM CMD:\n");
            string cmdString = "*PCAP.DISARM=";
            sendCtrl(cmdString);
            endCapture();
        }
    }
    else if (param == testparam)
    {
            setParams();
    }

    if(status)
    {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s:%s: Error setting values'\n",
                driverName, functionName);
    }

    callParamCallbacks();
    return status;
}

//----------------------------------
void Pandabox::setParams(){
/* Temporary function in absence of web interface for setup for testing*/
        std::string cmds[21]= {
                "*PCAP.DISARM=",
                "*CAPTURE=",
                "COUNTER1.STEP=50",
                "COUNTER1.START=100",
                "COUNTER1.ENABLE=SEQ1.ACTIVE",
                "COUNTER1.TRIG=SEQ1.OUTA",
                "COUNTER1.OUT.CAPTURE=Triggered",
                "SEQ1.PRESCALE=0.05",
                "SEQ1.TABLE_CYCLE=1",
                "SEQ1.TABLE<",
                "500",
                "520109824",
                "1",
                "1",
                "",
                "SEQ1.ENABLE=PCAP.ACTIVE",
                "SEQ1.INPA=BITS.ONE",
                "PCAP.ENABLE=SEQ1.ACTIVE",
                "PCAP.FRAME=BITS.ZERO",
                "PCAP.CAPTURE=SEQ1.OUTA",
                "PCAP.CAPTURE_TS.CAPTURE=Trigger"};
        for(int i = 0; i < 21; i ++)
        {
            sendCtrl(cmds[i]);
        }
}
//--------------------------------------


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
