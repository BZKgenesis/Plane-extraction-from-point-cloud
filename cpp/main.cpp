// On importe les librairies nécessaires
#include <stdio.h> // Gestion des entrées/sorties (dans le terminal)
#include <stdlib.h> // allocation dynamique de mémoire, convertion de chaine de caractères, gestion des processus
#include <signal.h> // gestion des interruptions (par exemple ici on utilise Ctrl+C pour quitter)
#include <string.h> // manipulation de chaines de caractères

#include <vector>

#include "sl_lidar.h"  // Librairie du LIDAR
#include "sl_lidar_driver.h"


#include <Windows.h> // Librairie pour les fonctions de Windows (ici on utilise Sleep)
#define delay(x)   ::Sleep(x) // On défini une fonction delay qui utilise la fonction Sleep de Windows

using namespace sl; // On utilise l'espace de nommage sl pour appelé les fonctions de la librairie (sinon on aurais du faire sl::fonction)

unsigned long int BAUDRATE = 115200; // On défini le baudrate par défaut à 115200
const char* COM_PORT = "\\\\.\\com3"; // On défini le port COM par défaut à COM3



void print_usage(int argc, const char* argv[]) // Fonction qui affiche la notice d'utilisation du programme
{
    printf("Usage:\n"
           " For serial channel\n %s --channel --serial <com port> [baudrate]\n"
           " The baudrate used by different models is as follows:\n"
           "  A1(115200),A2M7(256000),A2M8(115200),A2M12(256000),"
           "A3(256000),S1(256000),S2(1000000),S3(1000000)\n"
		   " For udp channel\n %s --channel --udp <ipaddr> [port NO.]\n"
           " The T1 default ipaddr is 192.168.11.2,and the port NO.is 8089. Please refer to the datasheet for details.\n"
           , argv[0], argv[0]);
}

bool checkSLAMTECLIDARHealth(ILidarDriver* drv) // Fonction qui vérifie l'état de santé du LIDAR
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
		if (healthinfo.status == SL_LIDAR_STATUS_ERROR) { // si l'état de santé est en erreur on affiche un message d'erreur
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

bool ctrl_c_pressed; // on gere l'interruption Ctrl+C
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main() { 
    // on déclare un pointeur sur chaine de caractère
    const char * opt_channel_param_first = NULL;
	sl_u32         opt_channel_param_second = 0;
    sl_result     op_result;

    IChannel* _channel;

    printf("Ultra simple LIDAR data grabber for SLAMTEC LIDAR.\n"
           "Version: %s\n", SL_LIDAR_SDK_VERSION);

    opt_channel_param_second = BAUDRATE;

    // use default com port
    opt_channel_param_first = COM_PORT;

    
    // create the driver instance
	ILidarDriver * drv = *createLidarDriver();

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    sl_lidar_response_device_info_t devinfo;
    bool connectSuccess = false;

    _channel = (*createSerialPortChannel(opt_channel_param_first, opt_channel_param_second));
    if (SL_IS_OK((drv)->connect(_channel))) {
        op_result = drv->getDeviceInfo(devinfo);

        if (SL_IS_OK(op_result)) 
        {
	        connectSuccess = true;
        }
        else{
            delete drv;
			drv = NULL;
        }
    }


    if (!connectSuccess) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n", opt_channel_param_first);
        goto on_finished;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);



    // check health...
    if (!checkSLAMTECLIDARHealth(drv)) {
        goto on_finished;
    }

    signal(SIGINT, ctrlc);
    
    drv->setMotorSpeed();
    // start scan...
    drv->startScan(0,1);

    // fetech result and print it out...
	const int COUNT = 8192;
    sl_lidar_response_measurement_node_hq_t* nodes = new  sl_lidar_response_measurement_node_hq_t[COUNT];
    while (1) {

        size_t   count = COUNT;

        op_result = drv->grabScanDataHq(nodes, count);

        if (SL_IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_z_q14 * 90.f) / 16384.f,
                    nodes[pos].dist_mm_q2/4.0f,
                    nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }

    drv->stop();
	delete[] nodes;
    delay(200);
    drv->setMotorSpeed(0);
    // done!
on_finished:
    if(drv) {
        delete drv;
        drv = NULL;
    }
    return 0;
}

