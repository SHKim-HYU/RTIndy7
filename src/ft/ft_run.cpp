#include "../global_vars.h"
#define DEVICE "/dev/rtdm/pcan1"
#define DEVICE2 "/dev/rtdm/pcan0"
#include <PCANDevice.h>

void rt_ft_run(void* param){

    RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	
	// rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
	PCANDevice can_left;
	// PCANDevice can_right;
	CANDevice::Config_t config;
    config.mode_fd = 0; // 0: can_right.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    
	bool is_left_can_open=1;
	// bool is_right_can_open=1;
    if(!can_left.Open(DEVICE, config, false))
    {
        std::cout << "Unable to open Left CAN Device" << std::endl;
		is_left_can_open=0;
    }
    // if(!can_right.Open(DEVICE2, config, false))
    // {
    //     std::cout << "Unable to open Right CAN Device" << std::endl;
	// 	is_right_can_open=0;
    // }
	can_left.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    // can_right.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear



	int resLeft1, resLeft2,resRight1, resRight2;
    CANDevice::CAN_msg_t TxFrameLeft,TxFrameRight;
    CANDevice::CAN_msg_t RxFrameLeft1;
    CANDevice::CAN_msg_t RxFrameLeft2;
    CANDevice::CAN_msg_t RxFrameRight1;
    CANDevice::CAN_msg_t RxFrameRight2;


    TxFrameLeft.id = 0x102;
    TxFrameLeft.length = 8;
    TxFrameLeft.data[0] = 0x02;
    TxFrameLeft.data[1] = 0x03;
    TxFrameLeft.data[2] = 0x01;
    TxFrameLeft.data[3] = 0x00;
    TxFrameLeft.data[4] = 0x00;
    TxFrameLeft.data[5] = 0x00;
    TxFrameLeft.data[6] = 0x00;
    TxFrameLeft.data[7] = 0x00;
    can_left.Send(TxFrameLeft);
    usleep(10000);


    // TxFrameRight.id = 0x102;
    // TxFrameRight.length = 8;
    // TxFrameRight.data[0] = 0x02;
    // TxFrameRight.data[1] = 0x02;
    // TxFrameRight.data[2] = 0x01;
    // TxFrameRight.data[3] = 0x00;
    // TxFrameRight.data[4] = 0x00;
    // TxFrameRight.data[5] = 0x00;
    // TxFrameRight.data[6] = 0x00;
    // TxFrameRight.data[7] = 0x00;
    // // can_right.Send(TxFrameRight);
    // // usleep(10000);
    // TxFrameRight.data[1] = 0x03;
    // can_right.Send(TxFrameRight);
    usleep(10000);


    RxFrameLeft1.length = 8;
    RxFrameLeft2.length = 8;
    // RxFrameRight1.length = 8;
    // RxFrameRight2.length = 8;
	double ft_array[6]={0};
	double ft_array2[6]={0};
	double filtered_ft_array[6]={0};
	double filtered_ft_array2[6]={0};
	double dt = 0.001;
	double wc = 105;
	double alpha  =(1.0/(1.0+dt*wc));
	resLeft2 = can_left.Receive(RxFrameLeft2); 
	resLeft1 = can_left.Receive(RxFrameLeft1);

	can_left.Send(TxFrameLeft);
	// can_right.Send(TxFrameRight);
	std::cout<<"FT_RUN"<<std::endl;
	while (is_left_can_open)
	{
		rt_task_wait_period(NULL); //wait for next cycle
		resLeft2 = can_left.Receive(RxFrameLeft2); 
		resLeft1 = can_left.Receive(RxFrameLeft1);
		if(resLeft1==1 && resLeft2==1){
			ft_array[0]=(RxFrameLeft1.data[0]*256.0+RxFrameLeft1.data[1])/100.0-300.0;
			ft_array[1]=(RxFrameLeft1.data[2]*256.0+RxFrameLeft1.data[3])/100.0-300.0;
			ft_array[2]=(RxFrameLeft1.data[4]*256.0+RxFrameLeft1.data[5])/100.0-300.0;

			ft_array[3]=(RxFrameLeft2.data[0]*256.0+RxFrameLeft2.data[1])/500.0-50.0;
			ft_array[4]=(RxFrameLeft2.data[2]*256.0+RxFrameLeft2.data[3])/500.0-50.0;
			ft_array[5]=(RxFrameLeft2.data[4]*256.0+RxFrameLeft2.data[5])/500.0-50.0;  
			if(abs(filtered_ft_array[i]-ft_array[i])>40){
					ft_array[i] = 0;
					info.act.F = Vector6d::Zero();
					break;
			}
			for(int i =0;i<6;i++){
				filtered_ft_array[i] = alpha*filtered_ft_array[i]+(1-alpha)*ft_array[i];
				info.act.F[i] = filtered_ft_array[i];
			}
		}
         rt_printf("Id 01  %.3f : %.3f , %.3f, %.3f, %.3f, %.3f, %.3f  \n", 0.01,ft_array[0], ft_array[1], ft_array[2], ft_array[3], ft_array[4],ft_array[5]);

		// // can_left.Send(TxFrameRight);
		// resRight2 = can_right.Receive(RxFrameRight2); 
		// resRight1 = can_right.Receive(RxFrameRight1);
		// if(resRight1==1 && resRight2==1){
		// 	ft_array2[0]=(RxFrameRight1.data[0]*256.0+RxFrameRight1.data[1])/100.0-300.0;
		// 	ft_array2[1]=(RxFrameRight1.data[2]*256.0+RxFrameRight1.data[3])/100.0-300.0;
		// 	ft_array2[2]=(RxFrameRight1.data[4]*256.0+RxFrameRight1.data[5])/100.0-300.0;

		// 	ft_array2[3]=(RxFrameRight2.data[0]*256.0+RxFrameRight2.data[1])/500.0-50.0;
		// 	ft_array2[4]=(RxFrameRight2.data[2]*256.0+RxFrameRight2.data[3])/500.0-50.0;
		// 	ft_array2[5]=(RxFrameRight2.data[4]*256.0+RxFrameRight2.data[5])/500.0-50.0;
		// 	for(int i =0;i<6;i++){
		// 		filtered_ft_array2[i] = alpha*filtered_ft_array2[i]+(1-alpha)*ft_array2[i];
		// 		info.act.F_ext_r[i] = filtered_ft_array2[i];
		// 	}
		// }
		// rt_printf("Id 01  %.3f : %.3f , %.3f, %.3f, %.3f, %.3f, %.3f  \n", gt,ft_array[0], ft_array[1], ft_array[2], ft_array[3], ft_array[4],ft_array[5]);
		// rt_printf("Id 02  %.3f : %.3f , %.3f, %.3f, %.3f, %.3f, %.3f  \n", gt,ft_array2[0], ft_array2[1], ft_array2[2], ft_array2[3], ft_array2[4],ft_array2[5]);

	}
}