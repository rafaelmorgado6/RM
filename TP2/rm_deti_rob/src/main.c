//pcompile main.c rm-mr32.c
//ldpic32 -w main.hex
//pterm

#include "rm-mr32.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

void rotateRel_basic(int speed, double deltaAngle);
void rotateRel(int maxVel, double deltaAngle);
bool checkForFinalLine(void); 
void adjustSensors2(void);
void adjustSensors1(void);

typedef struct {
    int tries;         // nº de caminhos que tentou naquela interseção
    int m_tries;      // nº máximo de caminhos naquela interseção
} Node; 

Node path[256];     // stack

// Função para imprimir a lista de caminhos por interseção
void printPathList(int intersection) {
    int i = 0;
    for (i = 0; i < intersection; i++) {
        printf("Interseção %d - Caminho escolhido: %d\n", i + 1, path[i].tries);
    }
}

int main(void)
{
    int groundSensor;

    initPIC32();    // Inicializa o PIC32
    closedLoopControl( true );  // Liga O controlo de velocidade (closed-Loop)
    setVel2(0, 0); // Define a velocidade de paragem (0,0)

    int intersection = 0;
    bool right = false;
    bool right_before = false;

    while(1)
    {
        printf("Press start to continue\n");
        while(!startButton());  // Espera que o START seja pressionado

        do
        {
            waitTick20ms();
            readAnalogSensors();
            groundSensor = readLineSensors(20);
            // printInt(groundSensor, 2 | 5 << 16);
            // printf("\n");
            printf("intersection %d: tries %d, max tries %d \n", intersection, path[intersection].tries, path[intersection].m_tries);
            if (intersection > 0){
                printf("previous intersection %d: tries %d, max tries %d \n", intersection-1, path[intersection-1].tries, path[intersection-1].m_tries);
            }

            int s4 = (groundSensor >> 4) & 1; 
            int s3 = (groundSensor >> 3) & 1; 
            int s2 = (groundSensor >> 2) & 1; 
            int s1 = (groundSensor >> 1) & 1; 
            int s0 = (groundSensor >> 0) & 1; 

            int baseSpeed = 30;
            int leftSpeed = baseSpeed;
            int rightSpeed = baseSpeed;

            right = s0 == 1;
       
            if (checkForFinalLine()) {
                setVel2(0, 0);  // Para o robô
                led(0,1);
                led(1,1);
                led(2,1);
                led(3,1);
                path[intersection].tries = 5;
                break;  // Sai do loop
            }

            else if (s4 == 1) // curva esquerda
            {   
                waitTick40ms();
                groundSensor = readLineSensors(20);
                int temp_s0 = (groundSensor >> 0) & 1; 

                adjustSensors1();
                
                groundSensor = readLineSensors(20);
                int s3 = (groundSensor >> 3) & 1; 
                int s2 = (groundSensor >> 2) & 1; 
                int s1 = (groundSensor >> 1) & 1; 

                if (path[intersection].tries == 0) {
                    path[intersection].tries = 1;
                    path[intersection].m_tries = 1 + temp_s0 + (s3 || s2 || s1);
                    intersection++;
            
                }
                else {
                    path[intersection].tries++;
                    if (path[intersection].tries > path[intersection].m_tries) {
                        path[intersection].tries = 0;
                        intersection--;
                    }
                    else intersection++;
                }


                rotateRel(50, M_PI / 2);

                right = false;
                
            }
            else if (s4 == 0 && s3 == 0 && s2 == 0 && s1 == 0 && s0 == 1) // curva direita
            {
                adjustSensors2();
                if (path[intersection].tries == 0) {
                    path[intersection].tries = 1;
                    path[intersection].m_tries = 1;
                    intersection++;
                }
                else {
                    path[intersection].tries++;
                    if (path[intersection].tries > path[intersection].m_tries) {
                        path[intersection].tries = 0;
                        intersection--;
                    }
                    else intersection++;
                }
                rotateRel(50, -M_PI / 2);
                right= false;

            }
            else if ((s3 == 1  || s2 == 1 || s1 == 1 ) && s0 == 0 && right_before) // em frente com curva a direita
            {
                if (path[intersection].tries == 0) {
                    path[intersection].tries = 1;
                    path[intersection].m_tries = 2;
                    intersection++;
                }
                else {
                    path[intersection].tries++;
                    if (path[intersection].tries > path[intersection].m_tries) {
                        path[intersection].tries = 0;
                        intersection--;
                    }
                    else intersection++;
                }
            }

            else if (s4 == 0 && s3 == 0 && s2 == 0 && s1 == 0 && s0 == 0) // volta para trás
            {
                waitTick20ms();
                
                groundSensor = readLineSensors(20);
                int s4 = (groundSensor >> 4) & 1; 
                int s3 = (groundSensor >> 3) & 1; 
                int s2 = (groundSensor >> 2) & 1; 
                int s1 = (groundSensor >> 1) & 1;
                int s0 = (groundSensor >> 0) & 1;
               
                if((s4 == 0 && s3 == 0 && s2 == 0 && s1 == 0 && s0 == 0)) {
                    rotateRel(50, M_PI); // Volta de 180º
                    intersection--;
                }
                
            }


            // Correções suaves para alinhamento
            if (s3 == 1 && s2 == 1 && s1 == 1)
            {
                leftSpeed = baseSpeed;
                rightSpeed = baseSpeed;
            }
            else if (s1 == 0)
            {
                leftSpeed = baseSpeed * 0.9;
                rightSpeed = baseSpeed * 1.0;
            }
            else if (s3 == 0)
            {
                leftSpeed = baseSpeed * 1.0;
                rightSpeed = baseSpeed * 0.9;
            }

            setVel2(leftSpeed, rightSpeed);
            led(0,0);
            led(1,0);
            led(2,0);
            led(3,0);

            right_before = right;

        } while (!stopButton());

        printPathList(intersection); 


        // OTIMIZAÇÃO
        while(1) {
            while (!startButton());  // Espera que o START seja pressionado
            
            int i=0;
            do {
                waitTick20ms();
                readAnalogSensors();
                groundSensor = readLineSensors(20);

                int s4 = (groundSensor >> 4) & 1; 
                int s3 = (groundSensor >> 3) & 1; 
                int s2 = (groundSensor >> 2) & 1; 
                int s1 = (groundSensor >> 1) & 1; 
                int s0 = (groundSensor >> 0) & 1; 

                int baseSpeed = 30;
                int leftSpeed = baseSpeed;
                int rightSpeed = baseSpeed;

                // Verifica se o robô encontrou a linha final
                if (checkForFinalLine()) {
                    setVel2(0, 0);  // Para o robô
                    led(0,1);
                    led(1,1);
                    led(2,1);
                    led(3,1);
                    break;  // Sai do loop
                }

                else if (s4 == 1 && s0 == 0) {
                    adjustSensors2();
                    groundSensor = readLineSensors(20);  
                    s3 = (groundSensor >> 3) & 1; 
                    s2 = (groundSensor >> 2) & 1; 
                    s1 = (groundSensor >> 1) & 1; 

                    if (s3 == 0 && s2 == 0 && s1 == 0) {
                        rotateRel(50, M_PI / 2);
                        i++;
                    }
                    else if(s3 == 1 || s2 == 1 || s1 == 1) {
                        if (path[i].tries == 1) {
                            rotateRel(50, M_PI / 2);
                            i++;
                        }
                        else if (path[i].tries == 2) {
                            i++;
                            continue;
                        }
                    }
                    //i++;
                }

                else if (s4 == 0 && s0 == 1) {
                    adjustSensors2();
                    groundSensor = readLineSensors(20); 
                    s3 = (groundSensor >> 3) & 1; 
                    s2 = (groundSensor >> 2) & 1; 
                    s1 = (groundSensor >> 1) & 1; 

                    if (s3 == 0 && s2 == 0 && s1 == 0) {
                        rotateRel(50, -M_PI / 2);
                        i++;
                    }
                    else if(s3 == 1 || s2 == 1 || s1 == 1) {
                        if (path[i].tries == 1) {
                            i++;
                            continue;
                        }
                        else if (path[i].tries == 2) {
                            rotateRel(50, -M_PI / 2);
                            i++;
                        }
                    }
                    //i++; 
                }

                else if (s4 == 1 && s0 == 1) {
                    adjustSensors2();
                    groundSensor = readLineSensors(20); 
                    s3 = (groundSensor >> 3) & 1; 
                    s2 = (groundSensor >> 2) & 1; 
                    s1 = (groundSensor >> 1) & 1; 

                    if (s3 == 0 && s2 == 0 && s1 == 0) {
                        
                        if (path[i].tries == 1) {
                            rotateRel(50, M_PI / 2);
                            i++;
                        }
                        else if (path[i].tries == 2) {
                            rotateRel(50, -M_PI / 2);
                            i++;
                        }
                        //i++; 
                    }
                    else if (s3 == 1 || s2 == 1 || s1 == 1) {
                        if (path[i].tries == 1) {
                            rotateRel(50, M_PI / 2);
                            i++;
                        } 
                        else if (path[i].tries == 2) {
                            i++;
                            continue;
                        }
                        else if (path[i].tries == 3){
                            rotateRel(50, -M_PI / 2);
                            i++;
                        }
                        //i++; 
                    }
                }

                // Correções suaves para alinhar o robô
                if (s3 == 1 && s2 == 1 && s1 == 1) {
                    leftSpeed = baseSpeed;
                    rightSpeed = baseSpeed;
                } else if (s1 == 0) {
                    leftSpeed = baseSpeed * 0.9;
                    rightSpeed = baseSpeed * 1.0;
                } else if (s3 == 0) {
                    leftSpeed = baseSpeed * 1.0;
                    rightSpeed = baseSpeed * 0.9;
                }

                // Atualiza a velocidade do robô
                setVel2(leftSpeed, rightSpeed);

            } while (!stopButton());
        }

    }

    return 0;
}



#define KP_ROT	40
#define KI_ROT	5


// Função para verificar se todos os sensores estão a 1 por 200ms
bool checkForFinalLine(void) {
    int groundSensor;
    int s4, s3, s2, s1, s0;
    int count = 0;
    // Verifica por 200ms
    while (count < 7) {
        waitTick20ms();
        count++;

        groundSensor = readLineSensors(20);
        s4 = (groundSensor >> 4) & 1;
        s3 = (groundSensor >> 3) & 1;
        s2 = (groundSensor >> 2) & 1;
        s1 = (groundSensor >> 1) & 1;
        s0 = (groundSensor >> 0) & 1;

        // Verifica se todos os sensores estão a 1
        if (!(s4 == 1 && s3 == 1 && s2 == 1 && s1 == 1 && s0 == 1)) {
            return false; 
        }
    }

    return true;  // Se todos os sensores estiverem a 1 por 200ms, retorna verdadeiro
}


void adjustSensors1()
{
    int groundSensor;
    int s4, s3, s2, s1, s0;
    int count=0;

    while (count<18) {
        waitTick20ms();  
        count++;
        
        readAnalogSensors();
        groundSensor = readLineSensors(20);
        
        s4 = (groundSensor >> 4) & 1;
        s3 = (groundSensor >> 3) & 1;
        s2 = (groundSensor >> 2) & 1;
        s1 = (groundSensor >> 1) & 1;
        s0 = (groundSensor >> 0) & 1;

        int baseSpeed = 30;
        int leftSpeed = baseSpeed;
        int rightSpeed = baseSpeed;

        // Correções suaves para alinhamento
        if (s3 == 1 && s2 == 1 && s1 == 1)
        {
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed;
        }
        else if (s1 == 0)
        {
            leftSpeed = baseSpeed * 0.95; 
            rightSpeed = baseSpeed * 1.0;
        }
        else if (s3 == 0)
        {
            leftSpeed = baseSpeed * 1.05;
            rightSpeed = baseSpeed * 0.95;
        }

        setVel2(leftSpeed, rightSpeed);
    }
}

void adjustSensors2()
{
    int groundSensor;
    int s4, s3, s2, s1, s0;
    int count=0;

    while (count<18) {
        waitTick20ms(); 
        count++;
        
        readAnalogSensors();
        groundSensor = readLineSensors(20);
        
        s4 = (groundSensor >> 4) & 1; 
        s3 = (groundSensor >> 3) & 1; 
        s2 = (groundSensor >> 2) & 1; 
        s1 = (groundSensor >> 1) & 1; 
        s0 = (groundSensor >> 0) & 1; 

        int baseSpeed = 30;
        int leftSpeed = baseSpeed;
        int rightSpeed = baseSpeed;

        if (s3 == 1 && s2 == 1 && s1 == 1)
        {
            leftSpeed = baseSpeed;
            rightSpeed = baseSpeed;
        }
        else if (s1 == 0)
        {
            leftSpeed = baseSpeed * 0.95; 
            rightSpeed = baseSpeed * 1.0;
        }
        else if (s3 == 0)
        {

            leftSpeed = baseSpeed * 1.05;
            rightSpeed = baseSpeed * 0.95;
        }

        setVel2(leftSpeed, rightSpeed);
    }
}


// deltaAngle in radians
void rotateRel(int maxVel, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    double integral = 0;
    int cmdVel;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    do
    {
        waitTick40ms();
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);

        integral += error;
        integral = integral > PI / 2 ? PI / 2: integral;
        integral = integral < -PI / 2 ? -PI / 2: integral;

        cmdVel = (int)((KP_ROT * error) + (integral * KI_ROT));
        cmdVel = cmdVel > maxVel ? maxVel : cmdVel;
        cmdVel = cmdVel < -maxVel ? -maxVel : cmdVel;

        setVel2(-cmdVel, cmdVel);
    } while (fabs(error) > 0.01);
    setVel2(0, 0);
}


void rotateRel_basic(int speed, double deltaAngle)
{
    double x, y, t;
    double targetAngle;
    double error;
    int cmdVel, errorSignOri;

    getRobotPos(&x, &y, &t);
    targetAngle = normalizeAngle(t + deltaAngle);
    error = normalizeAngle(targetAngle - t);
    errorSignOri = error < 0 ? -1 : 1;

    cmdVel = error < 0 ? -speed : speed;
    setVel2(-cmdVel, cmdVel);

    do
    {
        getRobotPos(&x, &y, &t);
        error = normalizeAngle(targetAngle - t);
    } while (fabs(error) > 0.01 && errorSignOri * error > 0);
    setVel2(0, 0);
}
