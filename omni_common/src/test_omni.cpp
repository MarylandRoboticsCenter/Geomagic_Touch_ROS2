#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>

#include<iostream>
#include<unistd.h>

int calibrationStyle;

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration()
{
    int supportedCalibrationStyles;
    HDErrorInfo error;

    hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
    if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET)
    {
        calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
        std::cerr<<"HD_CALIBRATION_ENCODER_RESET.\n";
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL)
    {
        calibrationStyle = HD_CALIBRATION_INKWELL;
        std::cerr<<"HD_CALIBRATION_INKWELL.\n";
    }
    if (supportedCalibrationStyles & HD_CALIBRATION_AUTO)
    {
        calibrationStyle = HD_CALIBRATION_AUTO;
        std::cerr<<"HD_CALIBRATION_AUTO.\n";
    }
    while (hdCheckCalibration() != HD_CALIBRATION_OK)
    {
        usleep(1e6);
        if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
            std::cerr<<"Please place the device into the inkwell for calibration\n";
        else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
        {
          std::cerr<<"Calibration updated successfully\n";
          hdUpdateCalibration(calibrationStyle);
        }
        else if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
          std::cerr<<"Calibration is already OK\n";
        }
        else
          std::cerr<<"Unknown calibration status\n";
    }
}
HDCallbackCode HDCALLBACK frameCallback(void *pUserData)
{
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE)
    {
      std::cerr<<"Updating calibration...\n";
      hdUpdateCalibration(calibrationStyle);
    }
    hdBeginFrame(hdGetCurrentDevice());

    hduVector3Dd angles;
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, angles);
    hduVector3Dd gimbal_angles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
    HDdouble pos[3];
    hdGetDoublev(HD_CURRENT_POSITION,pos);
    std::cerr<<"angles: "<<angles[0]<<" "<<angles[1]<<" "<<angles[2]<<"\n";
    std::cerr<<"gimbal_angles: "<<gimbal_angles[0]<<" "<<gimbal_angles[1]<<" "<<gimbal_angles[2]<<"\n";
    std::cerr<<"pos: "<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

int main(int argc, char **argv)
{
    ////////////////////////////////////////////////////////////////
    // Init Phantom
    ////////////////////////////////////////////////////////////////
    HDErrorInfo error;
    HHD hHD;
    hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        return -1;
    }
    std::cerr<< "Found device "<< hdGetString(HD_DEVICE_MODEL_TYPE);
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
      std::cerr<<"Device error during scheduler init\n";
      return -1;
    }
    HHD_Auto_Calibration();

    hdScheduleAsynchronous(frameCallback, (void*)0,
                           HD_MAX_SCHEDULER_PRIORITY);
   
    while(true) {
     sleep(10);
     std::cerr<<"tick\n";
    } 
    hdStopScheduler();
    hdDisableDevice(hHD);

    return 0;
}
