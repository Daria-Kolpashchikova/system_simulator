#include "simulator.h"
#include <QPair>
#include <time.h>
#include "drone.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

Simulator::~Simulator()
{
}
Simulator::Simulator(QWidget *parent)
    : QWidget(parent)
{

}

void Simulator::beginSimulating(double mass, double lagging, double coeff_k1, double coeff_k2, double force1, double force2,
                                double speed){
    //Зададим параметры
    phmodel.setParameters(mass, lagging, coeff_k1, coeff_k2, force1, force2, speed);
    //Зададим некоторые начальные данные дронов
    phmodel.setBegValuesDrones ();
    //Зададм возмущения
    phmodel.setDisturbances();

    X_coord_Win = 0.0;      // Начальное положение окна, которое перемещается
    VX_coord_Win = phmodel.speedOfLider;     // Скорость перемещения окна  м/сек, это же средняя скорость головного самолета
    dist_Pillar = 200;       // Расстояние между столбами
    drones_interval = 110;      //Заданное расстояние между дронами

    resize(800,600);        // Сделам размер окна на экране

    // Ширина и высота открытого экранного окна
    vpw = width();
    vph = height();

    minSizeV = MIN(vpw, vph);   // Минимальная сторона прямоугольника-окна
    idx = (vpw-minSizeV)/2;
    idy = (vph-minSizeV)/2;
    i_sqScreenSize = minSizeV;  // Размер стороны окна, в которое мы отображаем квадрат физического пр-ва

    physSpaceSize = 1000.0;    // Размер отображаемого квадрата в физическом пространстве (в м)
    zeroOffset = 100.0;         // На столько метров отодвигаем вправо окно в начальный момент вправо
                                 //чтобы точка 0 попала на экран

    numb_of_Pillars = (int)((physSpaceSize*1.5)/dist_Pillar);     // Количество отображаемых столбов (чтобы заведомо заполняли окно)
    // Первоначальные координаты столбов
    for (int i=0;i<numb_of_Pillars; i++)
        Pillars[i] = i*dist_Pillar;

    // Вычисляем параметры пересчета физических координат в экранные
    kff = double(i_sqScreenSize)/(physSpaceSize);

    numbMSec = 10;    // Через сколько микросекунд прерывания
    PhysT = 0.0;
    PhysTPrev = 0.0;

    ikRP = 3; //масштаб времени
    deltaPhysT = phmodel.deltaPhysT;     // Дробность времени для перерасчетов. Сейчас можно было сделать равным времени перерисовки,
                            // но эту схему зарезервируем для реализации физической модели по диф. уравнениям, это фактически
                            // задает точность для методов Эйлера и Гюна, еужно между отрисовками (которы 50 или 100 раз в секунду)
                            // делать еще итерации по методу Эйлера или Гюна, раз 10 или больше.
                            // deltaPhysT должно быть в разы меньше, чем numbMSec (в секундах, то есть deltaPhysT <<numbMSec*1000)
    startTimer(numbMSec);    //  Запускаем регулярное прерывание таймером, чтобы перерисовывать (при numbMSec = 10) 100 раз в секунду
    realTStart = double(clock())/CLOCKS_PER_SEC; // Начальное время в секундах
}

//**********************************************************
// перевод физических координат в экранные
// С учетом скорости движения подвижного окна
//**********************************************************
int Simulator::physXToWindowXCoords(double Xcoord)
{
    return(kff*(zeroOffset + Xcoord-VX_coord_Win*PhysT));        //Сначала сдвигаем X координату влево на расстояние, которое прошло подвижное окно
}
int Simulator::physYToWindowYCoords(double Ycoord)
{
    return(kff*(physSpaceSize-Ycoord)+idy-3*i_sqScreenSize/4);
}
//**********************************************************
// Пересчет положения дронов
//**********************************************************
void Simulator::recountDronePositions()
{
    // PhysTCurr   // Текущее физическое время
    // realTNext   //  Ожидаемый момент прерывания реального времени
    // PhysTNext   // Ожидаемый момент прерывания физического времени

    double F;    // Сила, действующая на дрон


    while (PhysT <= PhysTCurr)  // Вычисления между моментами отрисовки экрана
    {
        F = phmodel.drone[0].powerOfLider (phmodel.speedOfLider*phmodel.h, phmodel.K1, phmodel.K1, phmodel.K2);
        phmodel.drone[0].calcNextPosition(F, phmodel.deltaPhysT, 1);
        // Положение остальных дронов
        for (int i=1; i<phmodel.numbDrones; i++){
            F = phmodel.powerOfSlaveDrone (i, phmodel.K2);
            phmodel.drone[i].calcNextPosition(F, phmodel.deltaPhysT, 1);
        }

        PhysT = PhysT+deltaPhysT;  // Вычислим следующий момент времени
    }

    // Вычислим новое положение столбов
    if (Pillars[0] < VX_coord_Win*PhysT-zeroOffset)
    {
        for (int j=0; j<numb_of_Pillars-1; j++)
            Pillars[j] = Pillars[j+1];

        Pillars[numb_of_Pillars-1] = Pillars[numb_of_Pillars-2] + dist_Pillar;
    }
}

void Simulator::timerEvent(QTimerEvent *)
{
    realT = double(clock())/CLOCKS_PER_SEC - realTStart; // Реальное время в секундах от начала движения снаряда
    PhysTCurr = realT*ikRP;   // Текущее физическое время от начала движения снаряда
    recountDronePositions();

    update();       //Перерисовать экран
}

//**********************************************************
// Воспроизведение графики при прерываниях от таймера
//**********************************************************
void Simulator::paintEvent(QPaintEvent *) // Головной блок воспроизведения графики
{
    QPainter paint(this);

    // Отрисуем дроны

    int xC, yC;
    for (int i=0; i<phmodel.numbDrones; i++){  // Преобразуем физические координаты ядра в оконные
        xC = physXToWindowXCoords(phmodel.drone[i].x);
        yC = physYToWindowYCoords(phmodel.drone[i].y);
        drawDroneQuadro(paint, xC, yC, phmodel.drone[i].R);     // Отрисовка дрона-квадрокоптера
    }
    drawPillars(paint); // Отрисуем столбики
    //Вывод текущего времени в секундах

    // Выводим значение времени
    QString value;
    int xText = 20;
    int yText = 40;
    paint.drawText (xText, yText, "Time = ");
    value.setNum ((int) (PhysTCurr));
    paint.drawText (xText+58, yText, value); //Выводим значение времени, в целых
    paint.drawText (xText+90, yText, "sec");

}
//**********************************************************
// Отрисовка одного дрона - самолета
//**********************************************************
void Simulator::drawDrone(QPainter &paint, int xC, int yC, int R)
{
    double dist;
    paint.setPen(QPen(Qt::red,2,Qt::SolidLine));
    paint.drawLine (xC-R,yC,xC+R,yC);       //Фюзеляж
    paint.drawLine (xC-R,yC-R,xC,yC);       // Крыло левое
    paint.drawLine (xC-R,yC+R,xC,yC);       // Крыло правое
    paint.drawLine (xC-R-0.6*R,yC-0.6*R,xC-R,yC);   // Киль
}
//**********************************************************
// Отрисовка одного дрона - квадрокоптера
//**********************************************************
void Simulator::drawDroneQuadro(QPainter &paint, int xC, int yC, int R)
{
    double dist;
    int Razm;
    Razm = 2*R;
    paint.setPen(QPen(Qt::blue,2,Qt::SolidLine));
    paint.drawLine (xC-Razm,yC,xC+Razm,yC);       //Фюзеляж
    paint.drawLine (xC-Razm,yC-Razm,xC,yC);       // Крыло левое
    paint.drawLine (xC-Razm,yC+Razm,xC,yC);       // Крыло правое
    paint.drawLine (xC-Razm-0.6*Razm,yC-0.6*Razm,xC-Razm,yC);   // Киль
}

//**********************************************************
// Рисуем столбы
//**********************************************************
void Simulator::drawPillars(QPainter &paint)
{
    int x_Pillars;
    int y_Pillars;
    y_Pillars = physYToWindowYCoords(0.0);
    paint.setPen(QPen(Qt::black,2,Qt::SolidLine));
    for(int i=0; i<numb_of_Pillars-1; i++)
    {
        x_Pillars = physXToWindowXCoords(Pillars[i]);
        paint.drawLine (x_Pillars, y_Pillars, x_Pillars, y_Pillars+10);
        paint.drawLine (physXToWindowXCoords(VX_coord_Win*PhysT-zeroOffset), physYToWindowYCoords(0.0),
                        physXToWindowXCoords(2*physSpaceSize+VX_coord_Win*PhysT), physYToWindowYCoords(0.0));
    }
}

//**********************************************************
// Отображаем точку графика отличия скорости дрона i от заданной для дрона-лидера
//**********************************************************
void Simulator::drawSpeedGraphPoint(QPainter &paint, int i)
{
    double VxGraph; //Условное отображение значения скорости в физическом окне
    double kffSpeed = 1;  // КАоэффициент отображения скорости
    double abcissLevel;     // Уровень линии абциссы для графика
    int xxC;
    int yyC;

    abcissLevel = -(i+1)* 200.;
    VxGraph = abcissLevel + (phmodel.drone[i].vX -phmodel.speedOfLider)* kffSpeed;
    xxC = physXToWindowXCoords(700.+VX_coord_Win*PhysT);
    yyC = physYToWindowYCoords(VxGraph);
    paint.setPen(QPen(Qt::red));
    paint.drawPoint (xxC, yyC);
    paint.drawPoint (xxC, yyC+1);
    paint.drawPoint (xxC+1, yyC+1);
}

//**********************************************************
// Очиска экрана, но не понадобилось,  мы используем update
//**********************************************************
void Simulator::clear(QPainter &paint)
{
    paint.eraseRect(0,0,200,200);
}
