#ifndef SIMULATOR_H
#define SIMULATOR_H
#include <QtGui>
#include <math.h>
#include <QWidget>
#include "physmodel.h"

class Simulator : public QWidget
{
    Q_OBJECT

public:
    Simulator(QWidget *parent = 0);
    ~Simulator();
    void paintEvent(QPaintEvent *);
    void drawDrone(QPainter &paint, int xC, int yC, int R);
    void drawDroneQuadro(QPainter &paint, int xC, int yC, int R);
    void drawPillars(QPainter &paint);
    int physXToWindowXCoords(double Xcoord);
    int physYToWindowYCoords(double Ycoord);
    void clear(QPainter &paint);
    void timerEvent(QTimerEvent *);
    void recountDronePositions();
    void calcFball ();
    void drawSpeedGraphPoint(QPainter &paint, int i);
    void Simulator::beginSimulating(double mass, double lagging, double coeff_k1, double coeff_k2, double force1, double force2,
                                    double speed);

    PhysModel phmodel;

private:
    int vpw, vph;

    int minSizeV, idx, idy;

    int i_sqScreenSize, i_bordSize;
    double  physSpaceSize, kff;
    int ikRP;   //масштаб времени
    int numbMSec; // Через сколько микросекунд перерисовывать окно на экране

    double realTStart, realT, deltaPhysT, PhysT;   // реальный отсчет времени
    double dPhisT, PhysTCurr, realTNext, PhysTNext, PhysTPrev, stepPhysT;  // Пока не все используем

    double X_coord_Win;     // Координата перемещения окна, в которое видим дрон
    double VX_coord_Win;     // Скорость перемещения окна, в которое видим дрон (м/сек) это же средняя скорость головного самолета
    double dist_Pillar;            // Расстояние между столбами
    double Pillars [300];        // Массив указателей столбов
    int numb_of_Pillars;    // Количество отображаемых столбов
    double drones_interval; //Заданное расстояние между дронами
     double zeroOffset;

// Скорость перемещения окна  м/сек, это же средняя скорость головного самолета
};
typedef struct
{
    double X;
    double Y;
} vVect;

#endif // SIMULATOR_H
