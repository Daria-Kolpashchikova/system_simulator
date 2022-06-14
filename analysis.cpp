#include "analysis.h"
#include "physmodel.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

Analysis::~Analysis()
{
}
Analysis::Analysis(QWidget *parent)
    : QWidget(parent)
{

}
void Analysis::paintEvent(QPaintEvent *)
{
    QPainter paint(this);

    sizeWinX = 1000;
    sizeWinY = 600;
    resize(sizeWinX,sizeWinY);

    setParamsForGraphics ();    //Установим начальные параметры графиков
    countMaxOfDronesSpeed ();   //Вычислим максимальную скорость всех дронов, нужна много где
    CoeffCalc();

    // Отрисуем оси
    for (int j=0; j<phmodel.numbDrones; j++ )
        drawOneDroneAxes(j);

    // Отрисуем графики
    paintOneDroneGraph ();

}
/*************************************************************
 * Вычисление коэффициентов отрисовки объектов дронов
 *
 ************************************************************/
void Analysis::CoeffCalc()
{
    // Коэффициент преобразования времени в экранные координаты
    kffX = (double)(sizeWinX-zeroPixOffset-rightPixOffset)/maxExersTime;
    kffY = (double) (graphTopSize/maxDronesSpeed);
    //высота отдельного графика
    oneGraphHeight = (sizeGrapicsY - phmodel.numbDrones*topYOffset)/phmodel.numbDrones;
    graphTopSize = (int)(0.66* oneGraphHeight); //Для верхней части графиков отведем 2 третьих
}

/*************************************************************
 * Отрисовка осей для одного дрона
 *
 ************************************************************/
void Analysis::drawOneDroneAxes (int droneNumb)
{
    int XC, YC;

    QPainter paint(this);
    QString value;
    QFont font = paint.font();
    int fontsize = 12;
    font.setPixelSize(fontsize);
    paint.setFont(font);
    paint.setPen(Qt::black);

    int xAxesBeg = physTimeToWindowXCoords(0.0);
    int xAxesEnd = physTimeToWindowXCoords(maxExersTime);
    paint.setPen(QPen(Qt::black,1,Qt::SolidLine));

    YC = physSpeedToWindowYCoords(droneNumb, 0.);
    paint.drawLine (xAxesBeg,YC, xAxesEnd,YC);
    //Стрелочки в конце
    paint.drawLine (xAxesEnd,YC, xAxesEnd-20,YC-4);
    paint.drawLine (xAxesEnd,YC, xAxesEnd-20,YC+4);

    // Ось ординат
    paint.drawLine (physTimeToWindowXCoords(0.0), physSpeedToWindowYCoords (droneNumb, maxDronesSpeed),
                    physTimeToWindowXCoords(0.0), physSpeedToWindowYCoords (droneNumb, -maxDronesSpeed/2));

    //Нарисуем риски времени
    int stepTimeRis = 50;
    int timeRisVal = 0;
    while ((double)timeRisVal < maxExersTime)
    {
        XC = physTimeToWindowXCoords((double)timeRisVal);
        YC = physSpeedToWindowYCoords (droneNumb, 0.0);

        // Подписи времени
        paint.drawLine (XC, YC, XC, YC+10);

        value.setNum (timeRisVal);
        paint.drawText (XC+2, YC+fontsize+10, value);
        timeRisVal = timeRisVal + stepTimeRis;
    }

    YC = physSpeedToWindowYCoords(droneNumb, phmodel.speedOfLider);
    // Отрисуем еще и линии заданной скорости
    paint.setPen(QPen(Qt::magenta,1,Qt::SolidLine));
    paint.drawLine (xAxesBeg,physSpeedToWindowYCoords(droneNumb, phmodel.speedOfLider),
                    xAxesEnd,physSpeedToWindowYCoords(droneNumb, phmodel.speedOfLider));
    // Подпишем значение заданной скорости
    value.setNum ((int)phmodel.speedOfLider);
    paint.drawText (xAxesBeg - 2*fontsize, YC, value);


    return;
}

//**********************************************************
// перевод физических координат графика в экранные
// Перевод времени
//**********************************************************
int Analysis::physTimeToWindowXCoords(double crTime)
{
    return(zeroPixOffset + kffX*crTime);
}
//**********************************************************
// перевод физических координат графика в экранные
// Перевод скорости в экранный Y с учетом номера дрона и соответствующего сдвига графика
// этого дрона
//**********************************************************
int Analysis::physSpeedToWindowYCoords(int droneNumb, double droneSpeed)
{
    return((droneNumb+1)*(oneGraphHeight+topYOffset)-oneGraphHeight + graphTopSize - kffY*droneSpeed);
}

//**********************************************************
// Вычисление максимальной скорости у всех дронов
//**********************************************************
void Analysis::countMaxOfDronesSpeed ()
{
    int j;
    double F;

    //Зададим параметры
    phmodel.setParameters (mass, lagging, coeff1, coeff2, force1, force2, speed);
    //Зададим некоторые начальные данные дронов
    phmodel.setBegValuesDrones ();
    //Зададим возмущения
    phmodel.setDisturbances();
    phmodel.setCoefficientK1(K1);
    phmodel.setCoefficientK2(K2);

    currTime = 0.0;
    while (currTime < maxExersTime)
    {
        for (j=0; j<phmodel.numbDrones; j++ )
        {
            if (j==0)       //Для дрона-лидера формула другая
                F = phmodel.drone[0].powerOfLider (phmodel.speedOfLider*phmodel.h, phmodel.K1, phmodel.K1, phmodel.K2);
            else
                F = phmodel.powerOfSlaveDrone (j, phmodel.K2);
            // Вычисляем очередное состояние дрона
            phmodel.drone[j].calcNextPosition(F, phmodel.deltaPhysT, 1);

            if (maxDronesSpeed < phmodel.drone[j].vX)  //Подсчет максимально
                maxDronesSpeed = phmodel.drone[j].vX;
        }
        currTime = currTime+phmodel.deltaPhysT;   // Изменение текущего физического времени эксперимента
    }

    bigSpeedValue = phmodel.speedOfLider*3;   //Макстимально отображаемая на нрафиках скорость
    if (maxDronesSpeed > bigSpeedValue)
        maxDronesSpeed = bigSpeedValue;
}
//**********************************************************
// Отрисовка графика скоростей дронов
//**********************************************************
void Analysis::paintOneDroneGraph ()
{
    int j;
    double F;
    QPainter paint(this);
    paint.setPen(QPen(QBrush(QColor(Qt::red)), 2 ));
    double stepPaintTime = MAX(phmodel.deltaPhysT, 0.001);  //Рисовать через такие промежутки
    double nextTime;

    nextTime = 0.0;

    //Зададим параметры
    phmodel.setParameters (mass, lagging, coeff1, coeff2, force1, force2, speed);
    //Зададим некоторые начальные данные дронов
    phmodel.setBegValuesDrones ();
    //Зададим возмущения
    phmodel.setDisturbances();
    phmodel.setCoefficientK1(coeff1);
    phmodel.setCoefficientK2(coeff2);

    currTime = 0.0;
    while (currTime < maxExersTime)
    {
        for (j=0; j<phmodel.numbDrones; j++ )
        {
            if (j==0)       //Для дрона-лидера формула другая
                F = phmodel.drone[0].powerOfLider (phmodel.speedOfLider*phmodel.h, phmodel.K1, phmodel.K1, phmodel.K2);
            else
                F = phmodel.powerOfSlaveDrone (j, phmodel.K2);
            // Вычисляем очередное состояние дрона
            phmodel.drone[j].calcNextPosition(F, phmodel.deltaPhysT, 1);


            if (currTime >= nextTime)
            {
                int YC = physSpeedToWindowYCoords(j,phmodel.drone[j].vX);

                // Если точка в пределах обасти графика - отрисовываем
                if (YC <= physSpeedToWindowYCoords (j, -maxDronesSpeed/2)
                        &&  YC >= physSpeedToWindowYCoords (j, maxDronesSpeed))
                        paint.drawPoint( physTimeToWindowXCoords(currTime), YC);

                if (j== phmodel.numbDrones-1)   // Если точку отрисовали для последнего дрона
                        nextTime += stepPaintTime;
            }
        }
        currTime = currTime+phmodel.deltaPhysT;   // Изменение текущего физического времени эксперимента
    }
}
//**********************************************************
// Установка параметров процедур графики
//**********************************************************
void Analysis::setParamsForGraphics ()
{
    maxExersTime = 200.0;
    zeroPixOffset  = 40;        //Отступ нуля осей графика от края
    rightPixOffset = 15;        //Отступ права графика от края
    topYOffset = 20;        //Отступ сверху графиков от края
    sizeGrapicsY = sizeWinY;    // Отведенный размер по высоте для графиков, сделаем во всю экрана

    //Коэффициенты для демонстрации крайних вариантов
    K1= 1400;        //Коэффициенты
    K2 = 1400;       //системы уравнений

}


