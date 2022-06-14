#ifndef DRONE_H
#define DRONE_H


class Drone
{
public:

    void createTail(int tailS);      // Выделение массива для "хвоста", размещение первого элемента
                                                // и дублирование его во всём хвосте, установка значений
                                                // индекса разиещения и индекса последнего элемента
    void pushCurrent(double X);                 // Запоминание в "хвосте" координаты положения дрона
    double getLagging();            // Извлечение запаздвающего положениядрона, соответствует (t-h)
    void calcNextPosition(double F, double deltaPhysT, int method);
    double powerOfLider (double V, double K0, double K1, double K2);

    int putCurrentXIndex;      // индекс размещения текущего положения дрона
    double x, y, vX, vY, aX,  aY, m, R, xF, yF, yNext, xNext, yVNext, vXNext, aXNext, yANext;
    double tail[20000];       //массив для запоминания "следа, хвоста", перечня последних X координат дрона
    int tailSize;       // Значение размера "хвоста"
    double lagg;        // Положение дрона в момент t-h
};

#endif // DRONE_H
