#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <QtGui>
#include <math.h>
#include <QWidget>
#include <QPixmap>
#include "physmodel.h"

class Analysis : public QWidget
{
    Q_OBJECT

public:
    Analysis(double m, double l,
             double k1, double k2,
             double f1, double f2,
             double v) {mass = m; lagging = l; coeff1 = k1;
                        coeff2 = k2; force1 = f1; force2 = f2; speed = v;}
    Analysis(QWidget *parent = 0);
    ~Analysis();
    void paintEvent(QPaintEvent *);

    void setParamsForGraphics ();
    void drawOneDroneAxes (int droneNumb);
    int physTimeToWindowXCoords(double crTime);
    int physSpeedToWindowYCoords(int droneNumb, double droneSpeed);
    void countMaxOfDronesSpeed ();
    void paintOneDroneGraph ();
    void CoeffCalc();

    int sizeWinX,sizeWinY;          // Размер окна компьютерного окна
    double currTime;
    double maxExersTime;  // предельное время эксперимента, секунд
    double zeroPixOffset;      // На столько пикселей отодвигаем вправо начало координат графика
    double rightPixOffset;      // На столько пикселей отодвигаем вправо начало координат графика
    double kffX;            // Коэффициент преобразования времени в экранные координаты
    double kffY;            // Коэффициент преобразования скорости в часть экранных координат
    int topYOffset;         // Отступ графиков сверху, и от предыдущего графика
    int oneGraphHeight;     // Общая высота каждого графика
    int graphTopSize;       // Высота положительной части графика
    double maxDronesSpeed;  // Вычисленная максимально достигнутая скорость всеми дронами
    double bigSpeedValue;   //Макстимально отображаемая на нрафиках скорость
    double sizeGrapicsY;    // Отведенный размер по высоте для графико
    double K1;              // Для задания коэффициентов
    double K2;              // системы уравнений
    bool param_set;

    PhysModel phmodel;
private:
    double mass, lagging, coeff1, coeff2, force1, force2, speed;


};

#endif // MODEL_H
