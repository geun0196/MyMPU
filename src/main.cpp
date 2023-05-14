#include <iostream>
#include <string.h>

#include "../inc/i2c.h"
#include "../inc/timer.h"
#include "../inc/stats.h"
#include "../inc/accel.h"
#include "../inc/gyro.h"
#include "../inc/mag.h"


int main(int argc, char* argv[])
{
    if (gpioInitialise()<0) //initialises pigpio.h
    {
        //if pigpio initialisation failed
        std::cout<<"pigpio.h initialisation failed\n";
        return -1;
    }

    //variables commonly used in main()
    int device_handle, mag_device_handle, wake_handle;
    std::string current_time;
    double accel_x, accel_y, accel_z;
    double gyro_x, gyro_y, gyro_z;

    //attempt to open I2C devices
    device_handle = i2cOpen(I2C_DEVICE,I2C_DEVICE_ADDR,I2C_FLAGS);
    if (device_handle < 0)
    {
        std::string err = "Failed to open i2c communication to IMU\n";
        std::cout<<err<<"\n";
        return -2;
    }
    mag_device_handle = i2cOpen(I2C_DEVICE,I2C_MAGDEVICE_ADDR,I2C_FLAGS);
    if (mag_device_handle < 0)
    {
        std::string err = "Failed to open i2c communication to mag\n";
        std::cout<<err<<"\n";
        return -3;
    }

    //attempt to disable sleep modes
    wake_handle = i2cWriteByteData(device_handle,PWR_MGMT_1_ADDR,PWR_MGMT_1_VAL);
    if (wake_handle<0)
    {
        std::string err = "Failed to wake IMU device\n";
        std::cout<<err<<"\n";
        return -4;
    }

    device_wait(100); //to allow device to be ready to take readings
    std::cout << "Device is ready to take readings\n\n";

    //attempt to set scales
    set_gyro(device_handle,250);
    set_accel(device_handle,2);
    
    double velocity_x = 0.0; 
    double velocity_y = 0.0; 
    double velocity_z = 0.0;
    double time_interval = 0.1; // unit : sec
    
    //infinite loop to get readings from device
    while (true)
    {
        //get readings
        accel_x = -(get_accel_x(device_handle));
        accel_y = -(get_accel_y(device_handle));
        accel_z = -(get_accel_z(device_handle));
        gyro_x = get_gyro_x(device_handle);
        gyro_y = get_gyro_y(device_handle);
        gyro_z = get_gyro_z(device_handle);
        
        // velocity
        velocity_x += accel_x * time_interval;
        velocity_y += accel_y * time_interval;
        velocity_z += accel_z * time_interval;
    
        //output readings
        std::cout << "Accel:  x: "<<accel_x<<"  y: "<<accel_y<<"  z: "<<accel_z<<"  [m/s^2]\n";
        std::cout << "Gyro:  x: "<<gyro_x<<"  y: "<<gyro_y<<"  z: "<<gyro_z<<"  [deg/sec]\n";
        std::cout << "Velocity:  x: " << velocity_x << "  y: " << velocity_y << "  z: " << velocity_z << "  [m/s]\n";
        std::cout <<"\n";
        
        get_stats();
        
        device_wait(100); 
    }
    
    return 0;
}
