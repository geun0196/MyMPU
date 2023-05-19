#include <iostream>
#include <string.h>

#include "../inc/i2c.h"
#include "../inc/timer.h"
#include "../inc/accel.h"

int main(int argc, char* argv[])
{
    if (gpioInitialise()<0) //initialises pigpio.h
    {
        //if pigpio initialisation failed
        std::cout<<"pigpio.h initialisation failed\n";
        return -1;
    }

    //variables commonly used in main()
    int device_handle, wake_handle;
    std::string current_time;
    float accel_x, accel_y, accel_z;

    // gravity calibration
    float gx = 0, gy = 0, gz = 0; 
    const int SAMPLES = 100; // number of samples
    float samples[SAMPLES][3]; // sample data
    
    //attempt to open I2C devices
    device_handle = i2cOpen(I2C_DEVICE,I2C_DEVICE_ADDR,I2C_FLAGS);
    if (device_handle < 0)
    {
        std::string err = "Failed to open i2c communication to IMU\n";
        std::cout<<err<<"\n";
        return -1;
    }
    
    //attempt to disable sleep modes
    wake_handle = i2cWriteByteData(device_handle,PWR_MGMT_1_ADDR,PWR_MGMT_1_VAL);
    if (wake_handle<0)
    {
        std::string err = "Failed to wake IMU device\n";
        std::cout<<err<<"\n";
        return -2;
    }

    device_wait(100); //to allow device to be ready to take readings
    std::cout << "Device is ready to take readings\n\n";

    //attempt to set scales
    set_accel(device_handle,2);
    
    bool cali_flag = true;
    
    //infinite loop to get readings from device
    while (true)
    {
        if(cali_flag){
        
            // Calculate the value of maintaining gravity
            for (int i = 0; i < SAMPLES; i++) {
                samples[i][0] = get_accel_x(device_handle);
                samples[i][1] = get_accel_y(device_handle);
                samples[i][2] = get_accel_z(device_handle);
                gx += samples[i][0];
                gy += samples[i][1];
                gz += samples[i][2];
                device_wait(10);
            }

            printf("calibracn finish!!\n");
        
            gx /= SAMPLES;
            gy /= SAMPLES;
            gz /= SAMPLES;
                        
            cali_flag = false;
        }
        
        //get readings
        accel_x = get_accel_x(device_handle) - gx;
        accel_y = get_accel_y(device_handle) - gy;
        accel_z = get_accel_z(device_handle) - gz;
        
        if(abs(accel_x) < 1) {
            accel_x = 0;
        }
        
        //output readings
        std::cout << "Accel:  x: "<<accel_x<<"  y: "<<accel_y<<"  z: "<<accel_z<<"  [m/s^2]\n";
        std::cout <<"\n";
        
        device_wait(100); 
    }
    
    return 0;
}
