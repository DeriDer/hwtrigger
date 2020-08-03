#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <chrono>
#include "roboclaw.h"
#include <string>
#include <stdlib.h>
#include "../../lib/Scheduler/Scheduler.h"
#include <fstream>
#include <stdio.h>      // standard input / output functions
#include <thread>
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string.h>
#include <stdlib.h>

using namespace std::chrono;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace std::this_thread;
int exit_key = 0;
char output[200];
int USB1 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

//end serial setup
enum triggerType
{
        SOFTWARE,
        HARDWARE
};
const triggerType chosenTrigger = HARDWARE;

void readUSB()
{
  int n = 0,
    spot = 0;
char buf = '\0';
int USB1 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

/* Whole response*/
char response[200];
memset(response, '\0', sizeof response);
memset(output, '\0', sizeof output);
do {
    n = read( USB1, &buf, 1 );
    sprintf( &response[spot], "%c", buf );

    output[spot] = response[spot];
    spot += n;
} while( buf != '\n' && n > 0);

//cout<<"______"<<response<<"_____"<<endl;

//end
if (n < 0) {
    std::cout << "Error reading: " << strerror(errno) << std::endl;
}
else if (n == 0) {
    std::cout << "Read nothing!" << std::endl;
}
else {
    std::cout << n <<"Response: " << response << std::endl;
}

}
void OpenSerial()
{
  ///serial setup

  int USB1 = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

  struct termios tty;
  struct termios tty_old;
  memset (&tty, 0, sizeof tty);

  /* Error Handling */
  if ( tcgetattr ( USB1, &tty ) != 0 ) {
     std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
  }

  /* Save old tty parameters */
  tty_old = tty;

  /* Set Baud Rate */
  cfsetospeed (&tty, (speed_t)B9600);
  cfsetispeed (&tty, (speed_t)B9600);

  /* Setting other Port Stuff */
  tty.c_cflag     &=  ~PARENB;            // Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;

  tty.c_cflag     &=  ~CRTSCTS;           // no flow control
  tty.c_cc[VMIN]   =  1;                  // read doesn't block
  tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
  tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

  /* Make raw */
  cfmakeraw(&tty);

  /* Flush Port, then applies attributes */
  tcflush( USB1, TCIFLUSH );
  if ( tcsetattr ( USB1, TCSANOW, &tty ) != 0) {
     std::cout << "Error " << errno << " from tcsetattr" << std::endl;
  }

  unsigned char cmd[] = "INIT \r";
  int n_written = 0,
      spot = 0;

  do {
      n_written = write( USB1, &cmd[spot], 1 );
      std::cout << cmd[spot] <<endl;
      spot += n_written;

  } while (cmd[spot-1] != '\r' && n_written > 0);



}
int ConfigureTrigger(INodeMap & nodeMap)
{
        int result = 0;


        cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;
        try
        {
                //
                // Ensure trigger mode off
                //
                // *** NOTES ***
                // The trigger must be disabled in order to configure whether the source
                // hardware.
                //
                CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
                if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
                {
                        cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
                        return -1;
                }

                CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
                if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
                {
                        cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
                        return -1;
                }

                ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

                cout << "Trigger mode disabled..." << endl;
                //
                // Select trigger source
                //
                // *** NOTES ***
                // The trigger source must be set to hardware or software while trigger
                // mode is off.
                //
                CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
                if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
                {
                        cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
                        return -1;
                }

                // Set trigger mode to hardware ('Line0')
                CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line0");
                if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
                {
                        cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
                        return -1;
                }

                ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());

                cout << "Trigger source set to hardware..." << endl;

                //
                // Turn trigger mode on
                //
                // *** LATER ***
                // Once the appropriate trigger source has been set, turn trigger mode
                // on in order to retrieve images using the trigger.
                //
                CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
                if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
                {
                        cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
                        return -1;
                }

                ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

                cout << "Trigger mode turned back on..." << endl << endl;
                //select trigger at frame start
                CEnumerationPtr ptrtriggerSelector = nodeMap.GetNode("TriggerSelector");
                CEnumEntryPtr ptrtriggerFrameStart = ptrtriggerSelector->GetEntryByName("FrameStart");
                if (!IsAvailable(ptrtriggerFrameStart) || !IsReadable(ptrtriggerFrameStart))
                {
                        cout << "Unable to set trigger FrameStart. Aborting..." << endl;
                        return -1;
                }
                ptrtriggerSelector->SetIntValue(ptrtriggerFrameStart->GetValue());

                CEnumerationPtr triggerActivation = nodeMap.GetNode("TriggerActivation");
                CEnumEntryPtr ptrtriggerRisingEdge = triggerActivation->GetEntryByName("RisingEdge");
                if (!IsAvailable(ptrtriggerRisingEdge) || !IsReadable(ptrtriggerRisingEdge))
                {
                        cout << "Unable to set trigger risingEdge. Aborting..." << endl;
                        return -1;
                }
                triggerActivation->SetIntValue(ptrtriggerRisingEdge->GetValue());
                cout << "risiing edge selected"<< endl << endl;

                CEnumerationPtr ptrControlMode = nodeMap.GetNode("TransferControlMode");
                CEnumEntryPtr ptrUserControlled = ptrControlMode->GetEntryByName("UserControlled");
                if (!IsAvailable(ptrUserControlled) || !IsReadable(ptrUserControlled))
                {
                        cout << "Unable to set UserControlled transfer. Aborting..." << endl;
                        return -1;
                }
                ptrControlMode->SetIntValue(ptrUserControlled->GetValue());
                cout << "user controlled transfer" << endl << endl;

                CEnumerationPtr ptrOperationMode = nodeMap.GetNode("TransferOperationMode");
                CEnumEntryPtr ptrMultiBlock = ptrOperationMode->GetEntryByName("MultiBlock");
                if (!IsAvailable(ptrMultiBlock) || !IsReadable(ptrMultiBlock))
                {
                        cout << "Unable to set MultiBlock transfer. Aborting..." << endl;
                        return -1;
                }
                ptrOperationMode->SetIntValue(ptrMultiBlock->GetValue());
                cout << "multiblock" << endl << endl;

                CIntegerPtr ptrBlockCount = nodeMap.GetNode("TransferBlockCount");
                if (IsAvailable(ptrBlockCount) && IsWritable(ptrBlockCount))
                {
                  ptrBlockCount->SetValue(1);
                  cout << "transfer 1 image each time"<< endl;
                }
                else
                {
                  cout << "block count not avaliable"<< endl;
                }

                /*CCommandPtr ptrTransferStop = nodeMap.GetNode("TransferStop");
                if (!IsAvailable(ptrTransferStop) || !IsReadable(ptrTransferStop))
                {
                        cout << "Unable to stop transfer. Aborting..." << endl;
                        return -1;
                }
                ptrTransferStop->Execute();
                cout << "stop camera transfer"<< endl; */
                //cout << "Camera " << i << " serial number  " << SerialNumbers[i] << " triggerconfig done..." << endl;

        }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}
int ResetTrigger(INodeMap & nodeMap)
{
        int result = 0;

        CameraPtr pCam = NULL;
        try
        {
                //
                // Turn trigger mode back off
                //
                // *** NOTES ***
                // Once all images have been captured, turn trigger mode back off to
                // restore the camera to a clean state.
                //

                CEnumerationPtr ptrTriggerMode =nodeMap.GetNode("TriggerMode");
                if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
                {
                        cout << "Unable to disable trigger mode (node retrieval). Non-fatal error..." << endl;
                        return -1;
                }
                CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
                if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
                {
                        cout << "Unable to disable trigger mode (enum entry retrieval). Non-fatal error..." << endl;
                        return -1;
                }
                ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());
                cout << "Trigger mode disabled..." << endl << endl;
          }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}
int AcquireImages(CameraList camList)
{
        int result = 0;
        CameraPtr pCam = NULL;
        cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;
        try
        {
                //
                // Prepare each camera to acquire images
                //
                // *** NOTES ***
                // For pseudo-simultaneous streaming, each camera is prepared as if it
                // were just one, but in a loop. Notice that cameras are selected with
                // an index. We demonstrate pseduo-simultaneous streaming because true
                // simultaneous streaming would require multiple process or threads,
                // which is too complex for an example.
                //
                // Serial numbers are the only persistent objects we gather in this
                // example, which is why a vector is created.
                //
                vector<gcstring> strSerialNumbers(camList.GetSize());
                for (int i = 0; i < camList.GetSize(); i++)
                {
                        // Select camera
                        pCam = camList.GetByIndex(i);
                        // Set acquisition mode to continuous
                        CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
                        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
                        {
                                cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << i << "). Aborting..." << endl << endl;
                                return -1;
                        }
                        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
                        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
                        {
                                cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << i << "). Aborting..." << endl << endl;
                                return -1;
                        }
                        int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
                        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
                        cout << "Camera " << i << " acquisition mode set to continuous..." << endl;
                        // Begin acquiring images
                        pCam->BeginAcquisition();
                        cout << "Camera " << i << " started acquiring images..." << endl;
                        // Retrieve device serial number for filename
                        strSerialNumbers[i] = "";
                        CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
                        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
                        {
                                strSerialNumbers[i] = ptrStringSerial->GetValue();
                                cout << "Camera " << i << " serial number set to " << strSerialNumbers[i] << "..." << endl;
                        }
                        cout << endl;
                }

                //
                // Retrieve, convert, and save images for each camera
                //
                RoboClaw test( "/dev/ttyACM0", 5, 0x80 );

                test.begin( B1152000 );
                test.SpeedM2(250);

                //cout << "motorstart" << endl;

                //const unsigned int k_numImages = 1;
                //for (unsigned int imageCnt = 0; imageCnt < k_numImages; ++imageCnt)
                //int k_numImags = 10;
                int imageCnt = 0;
                int mychar;
                ofstream myfile;
                myfile.open("output.txt");
                int line_num = 1;
                OpenSerial();
                while(exit_key != 1/*mychar !='\n' && imageCnt !=5*/)
                {
                        //cout << "?"<< endl;
                        OpenSerial();
                        readUSB();
                        cout <<"CLSOE USB"<<endl;

                        //close(USB1);
                        bool valid;
                        uint8_t status;
                        uint32_t en1 = test.ReadEncM2(&status,&valid);
                        cout <<"?"<<endl;
                        myfile << line_num <<": "<< output <<std::endl;
                        myfile<<"Enc value: " << en1 << std::endl;
                        test.SetEncM2(0);
                        ++imageCnt;
                        high_resolution_clock::time_point t1 = high_resolution_clock::now();
                        for (int i = 0; i < camList.GetSize(); i++)
                        {

                                try
                                {
                                        //send sth start ua

                                        // Select camera
                                        pCam = camList.GetByIndex(i);
                                        //pCam->BeginAcquisition();
                                        CCommandPtr ptrTransferStart = pCam->GetNodeMap().GetNode("TransferStart");
                                        if (!IsAvailable(ptrTransferStart) || !IsWritable(ptrTransferStart))
                                        {
                                                cout << "Unable to start transfer. Aborting..." << endl;
                                                return -1;
                                        }
                                        ptrTransferStart->Execute();
                                        cout << "start camera transfer thanks GOD no shit happens!!!"<< endl;
                                        // Retrieve next received image and ensure image completion
                                        ImagePtr pResultImage = pCam->GetNextImage();
                                        cout << "after next"<< endl;
                                        if (pResultImage->IsIncomplete())
                                        {
                                                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
                                        }
                                        else
                                        {
                                                // Print image information
                                                cout << "Camera " << i << " grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
                                                // Convert image to mono 8
                                                ImagePtr convertedImage = pResultImage->Convert(PixelFormat_BGR8, HQ_LINEAR);
                                                // Create a unique filename
                                                ostringstream filename;
                                                filename << "cam";
                                                if (strSerialNumbers[i] != "")
                                                {
                                                        filename << i+1;
                                                        //filename << strSerialNumbers[i].c_str();
                                                }
                                                else
                                                {
                                                        filename << i;
                                                }
                                                filename << "-" << imageCnt << ".jpg";
                                                // Save image
                                                convertedImage->Save(filename.str().c_str());
                                                cout << "Image saved at " << filename.str() << endl;
                                        }
                                        // Release image
                                        pResultImage->Release();
                                        cout << endl;
                                }
                                catch (Spinnaker::Exception &e)
                                {
                                        cout << "Error: " << e.what() << endl;
                                        result = -1;
                                }

                        }
                        high_resolution_clock::time_point t2 = high_resolution_clock::now();
                        auto duration =duration_cast<milliseconds>(t2-t1).count();
                        cout << duration <<"  time taken"<< endl<<endl;
                        sleep_until( t1 + seconds(2));
                        line_num++;
                }
                myfile.close();
                //
                // End acquisition for each camera
                //
                // *** NOTES ***
                // Notice that what is usually a one-step process is now two steps
                // because of the additional step of selecting the camera. It is worth
                // repeating that camera selection needs to be done once per loop.
                //
                // It is possible to interact with cameras through the camera list with
                // GetByIndex(); this is an alternative to retrieving cameras as
                // CameraPtr objects that can be quick and easy for small tasks.
                //
                //RoboClaw test( "/dev/ttyACM0", 5, 0x80 );
                test.SpeedM2(0);
                test.end();
                for (int i = 0; i < camList.GetSize(); i++)
                {
                        // End acquisition
                        camList.GetByIndex(i)->EndAcquisition();
                }
        }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}
/*int Saveimages(CameraList camList)
{
  int result = 0;
  CameraPtr pCam = NULL;
  cout << endl << "*** IMAGE SAVE ***" << endl << endl;
  try
  {t1
    int mychar;
    while((mychar = getchar()) !=EOF &&mychar !='\n')
    {
            //send newtrigger
            OpenSerial();
            for (int i = 0; i < camList.GetSize(); i++)
            {
                    high_resolution_clock::time_point t1 = high_resolution_clock::now();
                    try
                    {
                            //send sth start ua

                            // Select camera
                            pCam = camList.GetByIndex(i);
                            //pCam->BeginAcquisition();
                            CCommandPtr ptrTransferStart = pCam->GetNodeMap().GetNode("TransferStart");
                            if (!IsAvailable(ptrTransferStart) || !IsWritable(ptrTransferStart))
                            {
                                    cout << "Unable to start transfer. Aborting..." << endl;
                                    return -1;
                            }
                            ptrTransferStart->Execute();
                            cout << "start camera transfer thanks GOD no shit happens!!!"<< endl;
                            // Retrieve next received image and ensure image completion
                            ImagePtr pResultImage = pCam->GetNextImage();
                            if (pResultImage->IsIncomplete())
                            {
                                    cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
                            }
                            else
                            {
                                    // Print image information
                                    cout << "Camera " << i << " grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl;
                                    // Convert image to mono 8
                                    ImagePtr convertedImage = pResultImage->Convert(PixelFormat_Mono8, HQ_LINEAR);
                                    // Create a unique filename
                                    ostringstream filename;
                                    filename << "multicam-";
                                    if (strSerialNumbers[i] != "")
                                    {
                                            filename << strSerialNumbers[i].c_str();
                                    }
                                    else
                                    {
                                            filename << i;
                                    }
                                    filename << "-" << imageCnt << ".jpg";
                                    // Save image
                                    convertedImage->Save(filename.str().c_str());
                                    cout << "Image saved at " << filename.str() << endl;
                            }
                            // Release image
                            pResultImage->Release();
                            cout << endl;
                    }
                    catch (Spinnaker::Exception &e)
                    {
                            cout << "Error: " << e.what() << endl;
                            result = -1;
                    }
                    high_resolution_clock::time_point t2 = high_resolution_clock::now();
                    auto duration =duration_cast<microseconds>(t2-t1).count();
                    cout << duration <<"  time taken"<< endl<<endl;
            }
    }
  }
  catch (Spinnaker::Exception &e)
  {
          cout << "Error: " << e.what() << endl;
          result = -1;
  }
  return result;
}
*/
int RunMultipleCameras(CameraList camList)
{
        int result = 0;
        CameraPtr pCam = NULL;
        int err1 =0;
        int err2 =0;
        try
        {
                //
                // Retrieve transport layer nodemaps and print device information for
                // each camera

                cout << endl << "*** MULTICAMERA START TO RUN ***" << endl << endl;


                // Initialize each camera

                for (int i = 0; i < camList.GetSize(); i++)
                {
                        // Select camera
                        pCam = camList.GetByIndex(i);
                        pCam->Init();
                        // Configure trigger*
                        INodeMap & nodeMap = pCam->GetNodeMap();
                        err1 = ConfigureTrigger(nodeMap);
                        cout << "config trigger for cam" << i << "done"<< "..."<< endl << endl;
                        if (err1 < 0)
                        {
                                return err1;
                        }
                        // Initialize camera

                }

                //OpenSerial();

                // Acquire images on all cameras
                result = result | AcquireImages(camList);
                //reset trigger

                //
                // Deinitialize each camera

                for (int i = 0; i < camList.GetSize(); i++)
                {
                        // Select camera
                        pCam = camList.GetByIndex(i);

                        // reset trigger*

                        INodeMap & nodeMap = pCam->GetNodeMap();
                        err2 = ResetTrigger(nodeMap);
                        if (err2 < 0)
                        {
                                return err2;
                        }
                        // Deinitialize camera
                        pCam->DeInit();
                }
        }
        catch (Spinnaker::Exception &e)
        {
                cout << "Error: " << e.what() << endl;
                result = -1;
        }
        return result;
}

int main(int /*argc*/, char** /*argv*/)
{
        unsigned int max_n_threads = 3;
        Bosma::Scheduler schedule(max_n_threads);
        schedule.every(std::chrono::milliseconds(10),[&exit_key]()
        {
          int c = getchar();
          if (c !='/n'&& c!=EOF)
          {
            exit_key = 1;
            cout << "exit loop" << endl;
          }
          else
          exit_key = 0;
        });
        int result = 0;
        // Print application build information
        cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;
        // Retrieve singleton reference to system object
        SystemPtr system = System::GetInstance();
        // Retrieve list of cameras from the system
        CameraList camList = system->GetCameras();
        unsigned int numCameras = camList.GetSize();
        cout << "Number of cameras detected: " << numCameras << endl << endl;
        // Finish if there are no cameras
        if (numCameras == 0)
        {
                // Clear camera list before releasing system
                camList.Clear();
                // Release system
                system->ReleaseInstance();
                cout << "Not enough cameras!" << endl;
                cout << "Done! Press Enter to exit..." << endl;
                getchar();
                return -1;
        }

        cout << endl << "Running example for all cameras..." << endl;
        result = RunMultipleCameras(camList);
        cout << "Example complete..." << endl << endl;


        // Clear camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        cout << endl << "Done! don't Press Enter to exit..." << endl;
        //getchar();
        return result;
}
