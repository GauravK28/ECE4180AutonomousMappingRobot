
#include "mbed.h"
#include "rtos.h"
#include "LSM9DS1.h"
#include "Motor.h"

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#define dist_per_wheel_rot 0.65


// IMU
LSM9DS1 imu(p9, p10, 0xD6, 0x3C);
Serial pc(USBTX, USBRX);
Mutex imu_mutex;

/* Robot Pose:

    float x
    float y
    float theta

*/
volatile float x, y, theta;
bool isTurning = false;

// MotorA
Motor rightA(p22, p6, p5);


// MotorB
Motor leftB(p21, p7, p8);


// Sonar
DigitalOut trigger(p12);
DigitalIn  echo(p11);
volatile int sonar_distance = 43;
int correction = 0;
Timer sonar;

//Lidar
Serial lidar(p28, p27);
volatile int lidar_distance = 40;

//Thread
static int TEST_THREAD_SPEED = 2000;
static int Production_THREAD_SPEED = 50;

//Interrupts
volatile int motorACount = 0;
volatile int motorBCount = 0;
InterruptIn motorAEncoder(p30);
InterruptIn motorBEncoder(p29);

//Pi Connection
RawSerial pi(USBTX, USBRX);

// Bluetooth
RawSerial blue(p13, p14);

//Debug
DigitalOut led(LED1);




void setup_sonar() {
    sonar.reset();
    sonar.start();
    while (echo==2) {};
    sonar.stop();
    correction = sonar.read_us();
}

void setup_pi() {
    pi.baud(9600);
}

void setup_imu() {
    imu.begin();
    imu.calibrate();
}

void setup_lidar() {
    lidar.baud(115200);
}



void motorAInc() {
    motorACount += isTurning ? 0 : 1;
}

void motorBInc() {
    motorBCount += isTurning ? 0 : 1;
}

void setup_encoders() {
    motorAEncoder.mode(PullUp);
    Thread::wait(0.1);
    motorAEncoder.rise(&motorAInc);
    motorBEncoder.mode(PullUp);
    Thread::wait(0.1);
    motorBEncoder.rise(&motorBInc);
}

void sonar_thread(void const* args) {
    while (1) {
        //Trigger sonar to send a ping
        trigger = 1;
        sonar.reset();
        trigger = 0;
        //Wait for echo high
        while (echo==0) {};
        sonar.start();
        //Wait for echo low
        while (echo==1) {};

        //Stop timer and read value
        sonar.stop();
        
        //Subtract software overhead timer delay and scale to cm
        sonar_distance = (sonar.read_us()-correction)/58.0;
        Thread::wait(200);
    }
    
}
float SPEED = -.5;
void stop_motors() {
    rightA.speed(0);
    leftB.speed(0);
}
void move_forward() {
    rightA.speed(SPEED);
    leftB.speed(SPEED);
}
void turn_right() {
    rightA.speed(-SPEED);
    leftB.speed(SPEED);
}

void turn_left() {
    rightA.speed(SPEED);
    leftB.speed(-SPEED);
}

float get_current_dps() {
    imu_mutex.lock();
    while(!imu.gyroAvailable());
    imu.readGyro();
    float dps = imu.calcGyro(imu.gz);
    imu_mutex.unlock();
    return dps;
}

float get_current_accel() {
    imu_mutex.lock();
    while(!imu.accelAvailable());
    imu.readAccel();
    float a = imu.calcAccel(imu.ax) * 9.8066;
    imu_mutex.unlock();
    return a;
}

void rotate(int target_degrees) {
    isTurning = true;
        led = 1;
        Timer rot_time;
        
        float degrees;
        Thread::wait(500);

        rot_time.start();
        int rightOrLeft = rand() % 2;
        if (rightOrLeft == 0) {
            turn_right();

        } else {
            turn_left();
        }
        
        float prev_dps = abs(get_current_dps());
        unsigned prev_t = rot_time.read();
        
        while(degrees < target_degrees) {
            float cur_dps = abs(get_current_dps());
            unsigned cur_t = rot_time.read();
            degrees += (((cur_dps + prev_dps)/2.0)*(cur_t - prev_t));
            prev_dps = cur_dps;
            prev_t = cur_t;
            Thread::wait(50);
        }
    

        rot_time.stop();
        isTurning = false;

}

int stop_distance = 30;
void random_walk() {
    if (sonar_distance <= stop_distance || lidar_distance <= 30) { // 15cm = 6in
        stop_motors();
        
        rotate(90);
        
        stop_motors();
        Thread::wait(1000);
        move_forward();
        led = 0;
    }
    
}

void full_rotation() {
    printf("full rotation \n\r");
    rotate(300);
    printf(" rotation comlete \n\r");
}

void motors_thread(void const* args) {
    Timer full_rotation_timer;
    full_rotation_timer.start();
    float prev = full_rotation_timer.read();
    
    move_forward();
    while(1) {
        random_walk();
        
        float cur = full_rotation_timer.read();
        if (cur - prev >= 12) {
            full_rotation();
            full_rotation_timer.reset();
            prev = 0;
            
        }
        
        Thread::wait(50);
    }


}

void imu_lidar_thread(void const* args) {
    while (1) {
        //LIDAR DATA
        char frame[10] = {0};
        int checksum = 0;
        for(int i = 1; i < 10; i++) {
            if(lidar.readable()) {
                frame[i] = lidar.getc();
            }
            if (i < 9) {
                checksum += frame[i];
            }
        }
        
        int strength = (frame[6] << 8 | frame[5]);
        int quality = frame[8];
        
        int lidar_in_cm = (frame[4] << 8 | frame[3]);
        lidar_distance = lidar_in_cm;
        printf("Distance %d\n\r", lidar_in_cm);

        if (lidar_in_cm > 3000) {
            Thread::wait(1000);
            continue;
        }

        float lidar_x = lidar_in_cm * cos(theta / 180 * M_PI);
        float lidar_y = lidar_in_cm * sin(theta / 180 * M_PI);
        
        float x_coord_send = (x * 100) + lidar_x;
        float y_coord_send = (y * 100) + lidar_y;
        
        int x_coord_send_int = (int)x_coord_send;
        int y_coord_send_int = (int)y_coord_send;
        printf("Test: %d, %d \n\r", x_coord_send_int, y_coord_send_int);
        char x_data[64] = "";
        sprintf(x_data,"%d", x_coord_send_int);
        for (int i = 0 ; i < 64; i++) {
               if (x_data == 0) break;
               blue.putc(x_data[i]);
        }
        blue.putc(',');
        char y_data[64] = "";
        sprintf(y_data,"%d", y_coord_send_int);
        for (int i = 0 ; i < 64; i++) {
               if (y_data == 0) break;
               blue.putc(y_data[i]);
        }
        blue.putc('\n');
        Thread::wait(500);
        
    }
        
}



void pose_calculator() {
    Timer time;
    time.start();
    
    float prev_dps = get_current_dps();
    float prev_a = 0;
    unsigned prev_t = time.read();
    unsigned prev_motor_a = motorACount;
    unsigned prev_motor_b = motorBCount;
    while (1) {
        unsigned cur_t = time.read();
        
        //Theta
        float cur_dps = get_current_dps();
        float delta_t = cur_t - prev_t;
        theta += (((cur_dps + prev_dps)/2.0)*(delta_t));
        if (theta < 0) {
            theta = 360 + theta;
        } else if (theta > 360) {
            theta = theta - 360;
        }
        prev_dps = cur_dps;
        
        float a = get_current_accel();


        float motor_dist = ((float)((motorACount - prev_motor_a) + (motorBCount - prev_motor_b)) / 2) / 180 * dist_per_wheel_rot;

        // X
        x += motor_dist *  cos(theta / 180 * M_PI);

        // Y
        y += motor_dist *  sin(theta / 180 * M_PI);
      
        prev_t = cur_t;
        prev_a = a;
        prev_motor_a = motorACount;
        prev_motor_b = motorBCount;
        
        Thread::wait(25);
        
    }
    
}

void setup_blue() {
    blue.baud(9600);    
}

int main() {
    setup_blue();
    setup_sonar();
    setup_imu();
    setup_encoders();
    setup_lidar();
    setup_pi();
    Thread t2(sonar_thread);
    Thread t3(imu_lidar_thread);
    Thread t4(motors_thread);
    pose_calculator();    
}


