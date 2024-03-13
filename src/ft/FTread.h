#include <canlib.h>
#include <canstat.h>
#include <iostream>
#include <string>
#include <csignal>
#include <cerrno>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <Eigen/Dense>
#include <iomanip>

#include <canlib.h>
#include <canstat.h>
#include <iostream>
#include <string>
#include <csignal>
#include <cerrno>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#define READ_WAIT_INFINITE (unsigned long)(-1)
static unsigned int msgCounter = 0;

using namespace Eigen;
using namespace std;
class FTread
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;   
    canHandle hnd;
    canStatus stat;
    int channel;
    int canId;
    struct sigaction sigact;    

    public:
        Vector6d FT;
        Vector6d init_FT;
        Vector6d filtered_FT;
        Vector6d prev_filtered_FT;
        Vector6d prev_FT;
        int init_flag;
        double wc;  //cut-off-frequency for LPF
        double dt;  //Sampling Time
        double Fs;  //Sampling Freq
        
        FTread(int channel, int canId);
        void readData();
        void setBias();
        void setCutOffFreq(double wc);
        void clearBias();
        void initialize();
        void print(Vector6d vec);
};