#ifndef PHYSMODEL_H
#define PHYSMODEL_H
#include "drone.h"


class PhysModel
{
public:
    PhysModel();
    void setParameters (double mass, double lagging, double coeff1, double coeff2, double force1, double force2,
                        double speed);
    void setBegValuesDrones ();
    void createPulDrones ();
    double powerOfSlaveDrone (int j, double K3);
    void setDisturbances();

    int numbDrones;     //Количество дронов
    int tailSize;       //Размеры следа каждого дрона (одинаковы для всех). В каждом дроне для удобства повторяется
    double Mdrones;     //масса каждого дрона (кг)
    double h;           //Запаздывание времени
    double deltaPhysT;  // Квант времении для расчета очередного шага в Эйлере или Гюне
    double drones_interval; //Заданное расстояние между дронами
    double distLider;       //Расстояние, которое должен пролететь головной дрон за h секунд
    double speedOfLider;    //заданная скорость лидера
    double StartPosition;   // Стартовая позиция головного дрона
    double K1, K2;          // Коэффициенты в системе дифференциально-разностных уравнений
    double vLiderDisturbance;            // Возмущение для скоростилидера
    double vSlave1Disturbance;            // Возмущение скорости для первого ведомого

    Drone drone[5];       // Массив дронов
    void setCoefficientK1(double KK1); // Установка коэффициента K1 системы уравнений
    void setCoefficientK2(double KK2); // Установка коэффициента K2 системы уравнений

};

#endif // PHYSMODEL_H
