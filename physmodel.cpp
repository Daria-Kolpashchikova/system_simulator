//************************************************************************************
//  Движение  дронов по задаче ВКР
//   Файл содержит модель движения
//   и задание начальных параметров
//  Санкт-Петербург  2022, май
//************************************************************************************
#include "physmodel.h"
#include <math.h>

PhysModel::PhysModel()
{

}
/**************************************************************
 * ЭТА ФУНКЦИЯ ДОЛЖНА РАБОТАТЬ ПЕРВОЙ
 Установка параметров
 * *************************************************************/
void PhysModel::setParameters (double mass, double lagging, double coeff1, double coeff2, double force1, double force2,
                               double speed)
{
    Mdrones = mass;   //масса каждого дрона (кг)
    h = lagging;
    deltaPhysT = 0.001;    // При постоянно отведенной памяти не делать значение меньше 0.0001
                            // tailSize, deltaPhysT и h должны быть связаны соотношением
                        // h = tailSize*deltaPhysT; Примечание:
                        // Соотношение tailSize = h/deltaPhysT даёт неправильное округление (в нашем случае 299)
    tailSize = (int) ((h+0.000001)/deltaPhysT);   // Количество дискретов в h

    // Вариант для измененного варианта системы
    numbDrones = 4;         // Количество дронов. При постоянно отведенной памяти максимум 5
    drones_interval = 200.;      //Заданное расстояние между дронами
    speedOfLider = speed;     //Скорость лидера в м/сек
    StartPosition = 600.;   // Стартовая позиция головного дрона
    K1 = coeff1;           // Этот коэффициент влияет на выравнивание скорости головного
    K2 = coeff2;           // Этот коэффициент влияет на выравнивание интервалов между дронами
    vLiderDisturbance = force1;            // Возмущение для скоростилидера
    vSlave1Disturbance= force2;            // Возмущение скорости для первого ведомого

    distLider = h*speedOfLider; // Дистанция пролета лидера за h сек
}

/**************************************************************
// Установка коэффициента системы уравнений K1
// Пришлось вынести, чтобы иметь возможность менять в цикле вычисления
// переходного процесса с перебором K1 и K2
**************************************************************/
void PhysModel::setCoefficientK1(double KK1)
{
    K1 = KK1;
}
/**************************************************************
// Установка коэффициента системы уравнений K2
// Пришлось вынести, чтобы иметь возможность менять в цикле вычисления
// переходного процесса с перебором K1 и K2
**************************************************************/
void PhysModel::setCoefficientK2(double KK2)
{
    K2 = KK2;
}

/**************************************************************
 * ЭТА ФУНКЦИЯ ДОЛЖНА РАБОТАТЬ ВТОРОЙ
 * Присваивание значений и начальных значений пулу дронов
 * и параметрам физической модели
 *  Здесь просто заполняемая часть
 * *************************************************************/
void PhysModel::setBegValuesDrones ()
{
    int i, j;

    createPulDrones ();

    // Зададим массы дронов
    for (i = 0; i<numbDrones; i++)
        drone[i].m = Mdrones;     //(кг)

    // Зададим положения и скорости дронов. Ускорения нулевые
    for (i = 0; i<numbDrones; i++)
    {
        drone[i].x = StartPosition - i*drones_interval;
        drone[i].y = 50.0;  // Чтобы задать высоту полета
        drone[i].R = 5;      // Для размеров дрона - квадрокоптера
        drone[i].vX = speedOfLider;
        drone[i].aX = 0.0;
    }

    // Заполняем след лидера, как будто он h секунд летел с постоянной скоростью
    for (j=0; j<tailSize; j++)
        drone[0].pushCurrent(drone[0].x - (tailSize-j-1)*deltaPhysT*drone[0].vX);

    // Заполняем след остальных, как будто они h секунд летели с постоянной скоростью
    for (i = 1; i<numbDrones; i++)
    {
        for (j=0; j<tailSize; j++)
            drone[i].pushCurrent(drone[i].x - (tailSize-j-1)*deltaPhysT*drone[i].vX);
    }
    return;
}

/**************************************************************
// Здесь вносим возмущения (отличие скорости от заданной)
**************************************************************/
void PhysModel::setDisturbances()
{
    drone[0].vX = drone[0].vX + vLiderDisturbance;            // Возмущение для скорости лидера
    drone[1].vX = drone[1].vX + vSlave1Disturbance;            // Возмущение для скорости первого ведомого
}

/**************************************************************
 * Создание пула дронов
 * numb - количество дронов
 * tailSize - размер следа каждого дрона (все одинаковы)
 * begValues - каким значением забить сначала
 * Устанавливает в каждом дроне начальные значения счетчиков записи в хвост и чтения
 * последнего элемена хвоста (должно соответсвовать (t-h)
 *
 * *************************************************************/
void PhysModel::createPulDrones ()
{
    int j;
    // Создадим массивы следов в дронах
    for (int i=0; i<numbDrones; i++)
        drone[i].createTail(tailSize);
}
/****************************************************
 * Вычисление необходимой силы для ведомого дрона j > 0
 * согласно
 *
 * m xj“(t) =   k1 (xn-1 (t) - xn (t)) - k2 (xn-1 (t-h) - xn (t- h))
 * реально делаем
 * m xj“(t) =   k3 ((xn-1 (t) - xn (t)) - (xn-1 (t-h) - xn (t- h))))
   но силу еще нужно тоже умножать на какой нибудь коэффициент K0
 * *************************************************/
double PhysModel::powerOfSlaveDrone (int j, double K2)
{
    //Для варианта стремления к установленной скорости лидера
    return (K2* ((drone[j-1].x - drone[j].x) -
                 (drone[j-1].getLagging() - drone[j].getLagging())
                 ));

}



