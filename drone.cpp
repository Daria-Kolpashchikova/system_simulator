//************************************************************************************
//  Движение  дронов по задаче ВКР
//   Файл содержит класс объекта - дрона, вкючая параметры и основные методы
//  Санкт-Петербург  2022, май
//************************************************************************************


#include "drone.h"
/****************************************************
 * Создание массива для следа (хвоста)
 *  После выполнения функции putCurrentXIndex должен указывать на последний записанный элемент
 * Это нужно для корректной работы getLagging
 * *************************************************/
void Drone::createTail(int tailS)
{
    int j;      //TEST
    putCurrentXIndex = -1;    //Сбрасываем индекс размещения текущего положения дрона
    tailSize = tailS;
}
/****************************************************
 * Помещение значения в след  (хвост)
 *  После выполнения функции putCurrentXIndex должен указывать на последний записанный элемент
 * Это нужно для корректной работы getLagging
 * *************************************************/
void Drone::pushCurrent(double X)
{
    putCurrentXIndex++;
    if (putCurrentXIndex == tailSize)
        putCurrentXIndex = 0;
    tail[putCurrentXIndex] = X;
    return;
}
/****************************************************
 * Получение значения следа  (хвоста)
 * *************************************************/
double Drone::getLagging()
{
    int getInd;
    getInd = putCurrentXIndex+1;
    if (getInd == tailSize)
        getInd = 0;
    return (tail[getInd]);
}
/****************************************************
 * Вычисление следующего положения дрона по 2му закону Ньютона
 * Параметр - значение приложенной силы F
 * за время deltaPhysT
 * method = 0   - методом Эйлера, method = 1 (и любом другом)   - методом Гюна
 Засылает вновь вычисленное положение в хвост
 * *************************************************/
void Drone::calcNextPosition(double F, double deltaPhysT, int method)
{
    //Вычисляем кандидата следующего положения (как бы метод Эйлера)
    xNext = x + deltaPhysT*vX;
    vXNext = vX + deltaPhysT*aX;
    aXNext = F/m;

    // Присваиваем следующее положение
    if (method == 0)   // метод Эйлера
    {
        x = xNext;
        vX = vXNext;
        aX = aXNext;
    }    else       // Метод Гюна
    {
        x  += deltaPhysT*(vX + vXNext)/2;
        vX += deltaPhysT*(aX + aXNext)/2;
        aX = (aX + aXNext)/2;
    }
    pushCurrent(x);
}
/****************************************************
 * Вычисление необходимой силы для первого дрона
 * согласно
 *
   m x1“(t) = V - (K1*x1 (t) - K2*x1(t- h),
 * *************************************************/
double Drone::powerOfLider (double V,  double K0, double K1, double K2)
{
    lagg = getLagging();        // Это x(t- h). Запомним для некоторых внешних программ
    return (K1*(V- (x - lagg)));
}
