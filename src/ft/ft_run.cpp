#include "ft_run.h"

#define DEVICE "/dev/rtdm/pcan1"
double a =1.0;

PCANDevice can;

unsigned int cycle_ns = 1000000; // 1 ms

unsigned char data_field[16];

short raw_data[6] = { 0 };
unsigned short temp;
unsigned DF=50, DT=2000;
double ft_array[6];
double RET_frq = 1000 ;
double frq_cut_0ff = 150;
double alpha = (frq_cut_0ff * ( 1/ RET_frq))/(1 + frq_cut_0ff * (1/ RET_frq));
double force[6];
double ft_array_prev[6];
double sensor_array[6];
double sensor_array_prev[6];

Matrix3d symmetric(double a, double b, double c) {
    Matrix3d symmetric_m(3, 3);
    symmetric_m <<  0.0, -c, b,
                    c, 0.0, -a,
                    -b, a, 0.0; 
     
    return symmetric_m;
}

Vector6d FT_sensor ;
void rt_ft_run(void *arg)
{	
	// CAN Setup
    CANDevice::Config_t config;
    config.mode_fd = 0; // 0: CAN2.0 Mode, 1: CAN-FD Mode
    config.bitrate = 1e6; //1mbps
    config.d_bitrate = 2e6; //2mbps
    config.sample_point = .875; //87.5% 
    config.d_sample_point = 0.6; //60%
    config.clock_freq = 80e6; // 80mhz // Read from driver?  
    

	std::vector<double> input_fx, input_fy, input_fz, input_mx, input_my, input_mz;
    double filtered_value_fx, filtered_value_fy, filtered_value_fz;
    double filtered_value_mx, filtered_value_my, filtered_value_mz;
    Vector3d filter_f(3);
    Vector3d filter_t(3);
    Vector3d filter_force(3);
    Vector3d filter_torque(3);
    Vector3d force_bias(3);
    Vector3d torque_bias(3);
	Vector6d filter_Wrench(6);


	int count = 0;
    int window_size = 500;
    double mass = 0.075;
    double gravity = -9.8;
    Eigen::Vector3d gravity_vec(0.0, 0.0, gravity);
    Eigen::Vector3d pos_Vec(0.0, 0.0, 0.05);
    Eigen::Matrix3d R1;
	R1 <<  1,0,0,
			0,1,0,
			0,0,1;
    if(!can.Open(DEVICE, config, false))
    {
        std::cout << "Unable to open CAN Device" << std::endl;
        // exit(-2);

    }

    // Setup Filters
    can.ClearFilters(); // Clear Existing/Reset.  Filters are saved on the device hardware.  Must make sure to clear
    can.AddFilter(1, 2); // Only Listen to messages on id 0x01, 0x02.  
	
    int res1, res2;
    CANDevice::CAN_msg_t TxFrame;
    CANDevice::CAN_msg_t RxFrame1;
    CANDevice::CAN_msg_t RxFrame2;


    TxFrame.id = 0x64;
    TxFrame.length = 8;
    TxFrame.data[0] = 0x0A;
    TxFrame.data[1] = 0x01;
    TxFrame.data[2] = 0x01;
    TxFrame.data[3] = 0x01;
    TxFrame.data[4] = 0x01;
    TxFrame.data[5] = 0x01;
    TxFrame.data[6] = 0x01;
    TxFrame.data[7] = 0x01;

    RxFrame1.length = 8;
    RxFrame2.length = 8;

    can.Status();

    can.Send(TxFrame);

    rt_task_set_periodic(NULL, TM_NOW, cycle_ns);
    while (1) {
        rt_task_wait_period(NULL); //wait for next cycle
        
        res2 = can.Receive(RxFrame2);
        res1 = can.Receive(RxFrame1);

        can.Send(TxFrame);

        if (res1 == 1 && res2 == 1)
        {
            //CANbus data to Torque data
            for(int i = 0; i<6; i++)
            {
                data_field[i] = (unsigned char) RxFrame1.data[i];
                data_field[i+8] = (unsigned char) RxFrame2.data[i];
            }
            
            for(int idx = 0; idx<6; idx++)
            {
                temp = data_field[2*idx+1]*256;
                temp += data_field[2*idx+2];

                raw_data[idx] = (signed short) temp;
            }

            // Set Force/Torque Original
            for(int n = 0; n<3; n++)
            {
                ft_array[n] = ((float)raw_data[n]) / DF;
                ft_array[n+3] = ((float)raw_data[n+3]) / DT;
            }
			for (int i = 0; i < 6; i++)
            {
                force[i] = alpha * ft_array[i] + (1 - alpha) * ft_array_prev[i] ;
                ft_array_prev[i] = force[i] ;
            }
			
			force_bias << force[0], force[1], force[2];
			torque_bias << force[3], force[4], force[5];
			
			if (count < 502) {
				input_fx.push_back(force[0]);
				input_fy.push_back(force[1]);
				input_fz.push_back(force[2]);
				input_mx.push_back(force[3]);
				input_my.push_back(force[4]);
				input_mz.push_back(force[5]);

				if (input_fx.size() >= window_size) {
					double sum_fx = std::accumulate(input_fx.end() - window_size, input_fx.end(), 0.0);
					double sum_fy = std::accumulate(input_fy.end() - window_size, input_fy.end(), 0.0);
					double sum_fz = std::accumulate(input_fz.end() - window_size, input_fz.end(), 0.0);
					double sum_mx = std::accumulate(input_mx.end() - window_size, input_mx.end(), 0.0);
					double sum_my = std::accumulate(input_my.end() - window_size, input_my.end(), 0.0);
					double sum_mz = std::accumulate(input_mz.end() - window_size, input_mz.end(), 0.0);

					filtered_value_fx = sum_fx / window_size;
					filtered_value_fy = sum_fy / window_size;
					filtered_value_fz = sum_fz / window_size;
					filtered_value_mx = sum_mx / window_size;
					filtered_value_my = sum_my / window_size;
					filtered_value_mz = sum_mz / window_size;
				} else {
					printf(" Please wait : %d \n", (499-count));
				}
			}
			else{
				filter_f << filtered_value_fx, filtered_value_fy, filtered_value_fz;
				filter_t << filtered_value_mx, filtered_value_my, filtered_value_mz;
				filter_force =  force_bias - (filter_f - (mass * R1.transpose() * gravity_vec));
				filter_torque =  torque_bias - (filter_t - (symmetric(pos_Vec[0], pos_Vec[1], pos_Vec[2])*mass * R1.transpose() * gravity_vec));
			    filter_Wrench << filter_force[0],filter_force[1],filter_force[2],filter_torque[0],filter_torque[1],filter_torque[2];
				std::cout << "filter_f :  \n" << filter_f << std::endl;
			//     body_Wrench << (mass * Rg[0]),(mass * Rg[1]),(mass * Rg[2]),0,0,0;
			//     filter_Wrench = (TSb*filter_Wrench);
			}
				info.F_sensor = filter_Wrench;
				count +=1;
			}
			
    }
    can.Close();
}