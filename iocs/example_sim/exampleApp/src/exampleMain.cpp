/* This file was automatically generated on Thu 20 Oct 2016 16:12:34 BST from
 * source: /home/fwf58757/git/zebra2/pandabox/etc/makeIocs/example.xml
 * 
 * *** Please do not edit this file: edit the source file instead. ***
 *  */
#include "epicsExit.h"
#include "epicsThread.h"
#include "iocsh.h"

int main(int argc, char *argv[])
{
    if(argc>=2) {
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    iocsh(NULL);
    epicsExit(0);
    return 0;
}

