#include <TimerOne.h>
#include <Servo.h>

TimerOne timer;

Servo arm;
Servo flip;
Servo catchR;  
Servo catchL;
int tirette =3;
int couleur =11;

int right_pinA = 21;
int right_pinB = 20;

int left_pinA = 18;
int left_pinB = 19;

int right_motorF = 6;
int right_motorB = 7;

int left_motorF = 4;
int left_motorB = 5;

int finCourseD = 49;
int finCourseG = 47;

int pompe1 = 33;
int pompe2 = 35;

volatile long int current_right_ticks = 0;
volatile long int current_left_ticks = 0;
volatile long int previous_right_ticks = 0;
volatile long int previous_left_ticks = 0;
volatile long int d_right_ticks = 0;
volatile long int d_left_ticks = 0;

volatile long int right_ticks = 0;
volatile long int left_ticks = 0;

volatile float orientation_rad = 0;
volatile float orientation_deg = 0;

volatile float dR = 0;
volatile float dL = 0;
volatile float dC = 0;
volatile float dTheta = 0;

volatile float dR_total = 0;
volatile float dL_total = 0;
volatile float dC_total = 0;

volatile float dR_speed = 0;
volatile float dL_speed = 0;
volatile float dTheta_speed = 0;

volatile float dX = 0;
volatile float dY = 0;

volatile float X = -470;
volatile float Y = 0;

volatile float right_encoder_speed = 0;
volatile float left_encoder_speed = 0;
volatile float theta_encoder_speed = 0;

float PWM_R = 0;
float PWM_L = 0;
float PWM_MIN = 35;
float PWM_MAX = 180;

float right_erreur = 0;
float left_erreur = 0;
float i_right_erreur = 0;
float i_left_erreur = 0;

float orientation = 0;
float i_orientation = 0;
float orientation_erreur = 0;
float i_orientation_erreur = 0;
float Theta_correction = 0;
float position_erreur = 0;

float kp = 0.3;
float kp_dour = 12;
float ki = 0;
float kTheta = 4;
float k_position =0.1;

int sens = 1;

volatile long int t = 0;
int T = 10;
int precision = 4;
float resoultion = 700;
float radius_r = 37.02;
float radius_l = 37.05;
float entreAxe = 264.65;

float rotation = 0;

float ticks2Distance(long int ticks, float radius, float resolution, int precision)
{
    return ticks * 2 * PI * radius / (resolution * precision);
}

float rad2Deg(float rad)
{
    return rad * 180 / PI;
}

void iniiit()
{
    dR_total = 0;
    dL_total = 0;
    dC_total = 0;
    i_right_erreur = 0;
    i_left_erreur = 0;
    right_erreur = 0;
    left_erreur = 0;
    position_erreur = 0;
    orientation_erreur = 0;
}

float erreur(float PWM, float min, float max)
{
    if (PWM < min)
    {
        PWM = min;
    }
    else if (PWM > max)
    {
        PWM = max;
    }
    return PWM;
}

void run()
{

    if (PWM_R > 0)
    {
        // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
        analogWrite(right_motorF, PWM_R);
        analogWrite(right_motorB, 0);
    }
    else
    {
        // PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);
        analogWrite(right_motorF, 0);
        analogWrite(right_motorB, -PWM_R);
    }
    if (PWM_L > 0)
    {
        // PWM_L=erreur(PWM_L,PWM_MIN,PWM_MAX);

        analogWrite(left_motorF, PWM_L);
        analogWrite(left_motorB, 0);
    }
    else
    {
        // PWM_L=erreur(PWM_L,-PWM_MAX,-PWM_MIN);

        analogWrite(left_motorF, 0);
        analogWrite(left_motorB, -PWM_L);
    }
}
void STOP()
{
    analogWrite(right_motorF, 0);
    analogWrite(right_motorB, 0);
    analogWrite(left_motorF, 0);
    analogWrite(left_motorB, 0);
}

float acceleration(float speed, float distance, float accel, float decel)
{
    float current_speed;

    if (abs(dC_total) < accel)
    {
        current_speed = (speed / (accel)) * abs(dC_total);
    }
    else if (distance - abs(dC_total) < decel)
    {
        current_speed = (speed / -decel) * abs(dC_total) + speed - ((distance - decel) * (speed / -decel));
    }
    else
    {
        current_speed = speed;
    }
    return current_speed;
}

float acceleration_dour(float speed, float distance, float accel, float decel)
{
    float current_speed;

    if ((dR_total - dL_total) < accel)
    {
        current_speed = (speed / (accel)) * (dR_total - dL_total);
    }
    else if (distance - (dR_total - dL_total) < decel)
    {
        current_speed = (speed / -decel) * (dR_total - dL_total) + speed - ((distance - decel) * (speed / -decel));
    }
    else
    {
        current_speed = speed;
    }

    current_speed = erreur(current_speed, PWM_MIN, speed);
    return current_speed;
}

void dour(float angle, float speed)
{
    iniiit();

    float distance = angle * PI * entreAxe / 180;

    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs((abs(dL_total - dR_total) - abs(distance)))>0.5)
    {
        if (((dR_total - dL_total) - distance) < 0)
            sens = 1;
        else
            sens = -1;
        /*Serial.print(distance);
  Serial.print("   ");
        Serial.print(sens);
  Serial.print("   ");*/
        float current_speed = sens * acceleration_dour(speed, abs(distance), abs(accel), abs(decel));

        // right pid
        right_erreur = current_speed - right_encoder_speed;
        i_right_erreur += right_erreur;
        PWM_R = kp_dour * right_erreur + ki * i_right_erreur;

        //  PWM_R=erreur(PWM_R,PWM_MIN,PWM_MAX);

        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }

        /*Serial.print(PWM_R);
          Serial.print("     ");*/
        // left pid
        left_erreur = -current_speed - left_encoder_speed;
        i_left_erreur += left_erreur;
        PWM_L = kp_dour * left_erreur + ki * i_left_erreur;
        // PWM_L=erreur(PWM_L,-PWM_MAX,-PWM_MIN);
        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        else
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        // Serial.println(PWM_L);
        // position pid
        position_erreur = k_position * (dR_total + dL_total);
        // Theta_correction=kTheta*orientation_erreur;

        PWM_R += position_erreur;
        PWM_L -= position_erreur;

        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        else
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }

        run();
    }
    STOP();
    delay(500);
}

void move(float distance, float speed)
{
    iniiit();
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs(dC_total - distance) > 5)
    {
        if ((dC_total - distance) < 0)
            sens = 1;
        else
            sens = -1;
        Serial.print(sens);
        Serial.print("      ");

        float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
        // right pid
        right_erreur = current_speed - right_encoder_speed;
        i_right_erreur += right_erreur;
        PWM_R = kp * right_erreur + ki * i_right_erreur;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        // left pid
        left_erreur = current_speed - left_encoder_speed;
        i_left_erreur += left_erreur;
        PWM_L = kp * left_erreur + ki * i_left_erreur;

        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        Serial.print(PWM_R);
        Serial.print("      ");
        Serial.print(dC_total);
        Serial.print("      ");
        Serial.print(current_speed);
        Serial.println("      ");
        /*Serial.print(right_erreur);
        Serial.print("    ");
        Serial.print(current_speed);
        Serial.print("   ");
        Serial.println(PWM_R);*/

        // orientation pid
        orientation_erreur = dR_total - dL_total;
        Theta_correction = kTheta * orientation_erreur;

        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        run();
    }
    STOP();
    Serial.print(dC_total);
}
void asta3(float distance, float speed)
{
    iniiit();
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs(dC_total - distance) > 1 && !(digitalRead(finCourseD) == 0 && digitalRead(finCourseG) == 0))
    {
        if ((dC_total - distance) < 0)
            sens = 1;
        else
            sens = -1;
        Serial.print(sens);
        Serial.print("      ");

        float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
        // right pid
        right_erreur = current_speed - right_encoder_speed;
        i_right_erreur += right_erreur;
        PWM_R = kp * right_erreur + ki * i_right_erreur;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        // left pid
        left_erreur = current_speed - left_encoder_speed;
        i_left_erreur += left_erreur;
        PWM_L = kp * left_erreur + ki * i_left_erreur;

        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        Serial.print(PWM_R);
        Serial.print("      ");
        Serial.print(dC_total);
        Serial.print("      ");
        Serial.print(current_speed);
        Serial.println("      ");
        /*Serial.print(right_erreur);
        Serial.print("    ");
        Serial.print(current_speed);
        Serial.print("   ");
        Serial.println(PWM_R);*/

        // orientation pid
        orientation_erreur = dR_total - dL_total;
        Theta_correction = kTheta * orientation_erreur;

        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        run();
    }
    STOP();
    Serial.print(dC_total);
}

void asta3Wa7daBark(float distance, float speed)
{
    iniiit();
    float accel = 0.25 * distance;
    float decel = 0.5 * distance;

    while (abs(dC_total - distance) > 1 && !(digitalRead(finCourseD) == 0 || digitalRead(finCourseG) == 0))
    {
        if ((dC_total - distance) < 0)
            sens = 1;
        else
            sens = -1;
        Serial.print(sens);
        Serial.print("      ");

        float current_speed = sens * acceleration(speed, abs(distance), abs(accel), abs(decel));
        // right pid
        right_erreur = current_speed - right_encoder_speed;
        i_right_erreur += right_erreur;
        PWM_R = kp * right_erreur + ki * i_right_erreur;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        // left pid
        left_erreur = current_speed - left_encoder_speed;
        i_left_erreur += left_erreur;
        PWM_L = kp * left_erreur + ki * i_left_erreur;

        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        Serial.print(PWM_R);
        Serial.print("      ");
        Serial.print(dC_total);
        Serial.print("      ");
        Serial.print(current_speed);
        Serial.println("      ");
        /*Serial.print(right_erreur);
        Serial.print("    ");
        Serial.print(current_speed);
        Serial.print("   ");
        Serial.println(PWM_R);*/

        // orientation pid
        orientation_erreur = dR_total - dL_total;
        Theta_correction = kTheta * orientation_erreur;

        PWM_R -= Theta_correction;
        PWM_L += Theta_correction;
        if (sens == 1)
        {
            PWM_R = erreur(PWM_R, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_R = erreur(PWM_R, -PWM_MAX, -PWM_MIN);
        }
        if (sens == 1)
        {
            PWM_L = erreur(PWM_L, PWM_MIN, PWM_MAX);
        }
        else
        {
            PWM_L = erreur(PWM_L, -PWM_MAX, -PWM_MIN);
        }
        run();
    }
    STOP();
    Serial.print(dC_total);
}

void orienter(float orientation, float speed)
{
    float target_angle = orientation - orientation_deg;
    if (target_angle > 180)
        target_angle -= 360;
    if (target_angle < -180)
        target_angle += 360;
    dour(target_angle, speed);
}

void doEncodeA0()
{
    if (digitalRead(right_pinA) != digitalRead(right_pinB))
    {
        right_ticks++;
    }
    else
    {
        right_ticks--;
    }
}
void habbetCatch(){
  timer.stop();
  catchR.attach(A8);      
    catchL.attach(A9);
    catchR.write(50);
  catchL.write(120);
  
delay(1200);
    catchL.detach();
    catchR.detach();
timer.start();
}
void talla3Catch(){
  timer.stop();
  catchR.attach(A8);      
    catchL.attach(A9);
    catchR.write(145);
  catchL.write(40);
  
delay(1200);
    catchL.detach();
    catchR.detach();
    timer.start();
}
void doEncodeB0()
{
    if (digitalRead(right_pinA) == digitalRead(right_pinB))
    {
        right_ticks++;
    }
    else
    {
        right_ticks--;
    }
}
void doEncodeA1()
{
    if (digitalRead(19) != digitalRead(18))
    {
        left_ticks++;
    }
    else
    {
        left_ticks--;
    }
}
void doEncodeB1()
{
    if (digitalRead(18) == digitalRead(19))
    {
        left_ticks++;
    }
    else
    {
        left_ticks--;
    }
}

void updatePosition()
{

    previous_right_ticks = current_right_ticks;
    previous_left_ticks = current_left_ticks;
    current_right_ticks = right_ticks;
    current_left_ticks = left_ticks;

    d_right_ticks = current_right_ticks - previous_right_ticks;
    d_left_ticks = current_left_ticks - previous_left_ticks;

    dR = ticks2Distance(d_right_ticks, radius_r, resoultion, precision);
    dL = ticks2Distance(d_left_ticks, radius_l, resoultion, precision);

    dR_total += dR;
    dR_speed += dR;
    dL_total += dL;
    dL_speed += dL;

    dC = (dR + dL) / 2;
    dC_total += dC;

    dTheta = (dR - dL) / entreAxe;

    dL_speed += dTheta;
    orientation_rad += dTheta;

    while (orientation_rad > PI)
    {
        orientation_rad -= 2 * PI;
    }

    while (orientation_rad < -PI)
    {
        orientation_rad += 2 * PI;
    }

    orientation_deg = rad2Deg(orientation_rad);

    dX = dC * cos(orientation_rad);
    dY = dC * sin(orientation_rad);

    X += dX;
    Y += dY;
    // Serial.println(dC_total);
}

void updateSpeed()
{

    right_encoder_speed = dR_speed * 1000 / (T * 5);
    dR_speed = 0;

    left_encoder_speed = dL_speed * 1000 / (T * 5);
    dL_speed = 0;

    theta_encoder_speed = dTheta_speed * 1000 / (T * 5);
    dTheta_speed = 0;
}

void callback()
{
    t++;
    updatePosition();

    if (t % T == 0)
    {
        updateSpeed();
    }
}

void go(float x, float y, float speedf, float speedr)
{
    int sens = (asinf((y - Y) / sqrtf((X - x) * (X - x) + (Y - y) * (Y - y))) > 0) ? 1 : -1;
    float target_angle = sens * rad2Deg(acosf((x - X) / sqrtf((X - x) * (X - x) + (Y - y) * (Y - y))));
    orienter(target_angle, speedr);
    float goal_distance = sqrtf((X - x) * (X - x) + (Y - y) * (Y - y));
    move(goal_distance, speedf);
    // Serial.print(target_angle);
    // Serial.println("   ");
}

void mauve()
{

    timer.stop();
    // initialisation

    flip.attach(9);
    flip.write(5);
    arm.attach(8);
    arm.write(80);
    delay(2000);
    arm.detach();
    flip.detach();

    timer.start();

    go(520, -200, 800, 100);
    //go(550, -200, 100, 100);
    delay(500);
    orienter(45, 100);
    //orienter(45, 100);
    //orienter(45, 100);
    asta3Wa7daBark(1000,300);
  
    timer.stop();

    // 5adem pompe replique
    digitalWrite(pompe2, HIGH);
    // servo arm yatla3
    arm.attach(8);
    arm.write(90);
    delay(1000);

    // 5adem pompe statuette
    digitalWrite(pompe1, HIGH);

    // servo arm yahbet
    arm.write(45);
    delay(1000);

    // servo arm fi chtar
    arm.write(50);
    //delay(1000);

    // servo flip yeflippi

    flip.attach(9);
    flip.write(159);
    delay(1000);
    flip.detach();

    // servo arm yahbet
    arm.write(45);
    delay(1000);

    // sakker pompe replique
    digitalWrite(pompe2, LOW);
    delay(1000);

    // servo arm fi chtar
    arm.write(55);
    delay(1000);
    arm.detach();
    habbetCatch();
    
       timer.start();
       
       move(-400,500);
       timer.stop();
       talla3Catch();
       

       
       delay(1000);
       
       go(-600,10,800,100);
       delay(1000);
       //dour(30,100);
       orienter(180,100);
       
       delay(1000);
       orienter(180,100);
       asta3(700,400);
       X=125;
       orientation_rad=PI;
       timer.stop();
       delay(1000);
  
 timer.stop();
    // flip servo yflippi
    flip.attach(9);
    flip.write(5);
    delay(2000);
    flip.detach();

    // servo arm yahbet
    arm.attach(8);
    arm.write(45);
    delay(800);
    arm.detach();

    // pompe statuette tet7al
    digitalWrite(pompe1, LOW);
    delay(1000);

    // servo arm fi chtar
    arm.attach(8);
    arm.write(80);
    delay(1000);
    arm.detach();

    timer.start();
    move(-300, 500);
    orienter(90,200);
    asta3(500,400);
    Y=-125;
    orientation_rad=PI/2;
    habbetCatch();
move(-400, 500);
    talla3Catch();


    
    
    delay(500);
    go(1700, -720, 600, 100);
    orienter(0,100);
   
    timer.stop();
    arm.attach(8);
    arm.write(40);
    delay(1000);
    arm.detach();

    timer.start();
    asta3(1000,200);
    move(-300, 300);
    go(500, -400, 600, 100);
}






void asfar()
{

    timer.stop();
    // initialisation

    flip.attach(9);
    flip.write(5);
    arm.attach(8);
    arm.write(80);
    delay(2000);
    arm.detach();
    flip.detach();

    timer.start();

    go(570, 200, 800, 100);
    //go(550, -200, 100, 100);
    delay(500);
    orienter(-45, 100);
    //orienter(45, 100);
    //orienter(45, 100);
    asta3Wa7daBark(1000,300);
  
    timer.stop();

    // 5adem pompe replique
    digitalWrite(pompe2, HIGH);
    // servo arm yatla3
    arm.attach(8);
    arm.write(90);
    delay(1000);

    // 5adem pompe statuette
    digitalWrite(pompe1, HIGH);

    // servo arm yahbet
    arm.write(45);
    delay(1000);

    // servo arm fi chtar
    arm.write(50);
    //delay(1000);

    // servo flip yeflippi

    flip.attach(9);
    flip.write(159);
    delay(1000);
    flip.detach();

    // servo arm yahbet
    arm.write(45);
    delay(1000);

    // sakker pompe replique
    digitalWrite(pompe2, LOW);
    delay(1000);

    // servo arm fi chtar
    arm.write(55);
    delay(1000);
    arm.detach();
    habbetCatch();
    
       timer.start();
       
       move(-400,500);
       timer.stop();
       talla3Catch();
       

       
       delay(1000);
       
       go(-600,-30,800,100);
       delay(1000);
       //dour(30,100);
       orienter(-180,100);
       
       delay(1000);
       orienter(-180,100);
       asta3(700,400);
       X=-125;
       orientation_rad=PI;
       timer.stop();
       delay(1000);
  
 timer.stop();
    // flip servo yflippi
    flip.attach(9);
    flip.write(5);
    delay(2000);
    flip.detach();

    // servo arm yahbet
    arm.attach(8);
    arm.write(45);
    delay(800);
    arm.detach();

    // pompe statuette tet7al
    digitalWrite(pompe1, LOW);
    delay(1000);

    // servo arm fi chtar
    arm.attach(8);
    arm.write(80);
    delay(1000);
    arm.detach();

    timer.start();
    move(-300, 500);
    orienter(-90,200);
    asta3(500,400);
    Y=125;
    orientation_rad=-PI/2;
    habbetCatch();
move(-400, 500);
    talla3Catch();


    
    
    delay(500);
    go(1500, 900, 600, 100);
    orienter(0,100);
   
    timer.stop();
    arm.attach(8);
    arm.write(40);
    delay(1000);
    arm.detach();

    timer.start();
    asta3(1000,200);
    move(-300, 300);
    go(500, 400, 600, 100);
}
void setup()

{
    timer.initialize(5000);
    timer.attachInterrupt(callback);
    // put your setup code here, to run once:
    pinMode(tirette,INPUT);
    pinMode(couleur,INPUT_PULLUP);
    pinMode(right_pinA, INPUT_PULLUP);
    pinMode(right_pinB, INPUT_PULLUP);
    pinMode(left_pinA, INPUT_PULLUP);
    pinMode(left_pinB, INPUT_PULLUP);
    pinMode(right_motorF, OUTPUT);
    pinMode(left_motorB, OUTPUT);
    pinMode(right_motorF, OUTPUT);
    pinMode(left_motorB, OUTPUT);
    pinMode(pompe1, OUTPUT);
    pinMode(pompe2, OUTPUT);
    pinMode(finCourseD, INPUT_PULLUP);
    pinMode(finCourseG, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(right_pinA), doEncodeA0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(right_pinB), doEncodeB0, CHANGE);
    attachInterrupt(digitalPinToInterrupt(left_pinA), doEncodeA1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(left_pinB), doEncodeB1, CHANGE);
    Serial.begin(9600);
    
    talla3Catch();


if (digitalRead(couleur)==HIGH)
{
    mauve();
}
else{
    asfar();
}



}

void loop()
{
  

}
