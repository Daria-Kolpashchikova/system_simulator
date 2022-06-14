#include "widget.h"
#include "ui_widget.h"

#include <QPair>
#include <QLabel>
#include <time.h>
#include <QLCDNumber>
#include "drone.h"
#include "simulator.h"
#include "drone.h"
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define PI 3.141592653589793238462643

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    QObject::connect(ui->horizontalSlider_m, &QSlider::valueChanged, this, [=] () {
        ui->label_m->setText(QString::number(ui->horizontalSlider_m->value()));
    });

    QObject::connect(ui->horizontalSlider_h, &QSlider::valueChanged, this, [=] () {
        ui->label_h->setText(QString::number(ui->horizontalSlider_h->value()));
    });

    QObject::connect(ui->horizontalSlider_V, &QSlider::valueChanged, this, [=] () {
        ui->label_s->setText(QString::number(ui->horizontalSlider_V->value()));
    });

    was_begin = 0;

}


void Widget::on_pushButton_calc_clicked()
{
   double lagging = ui->horizontalSlider_h->value(),
          mass = ui->horizontalSlider_m->value();
   double border;
   lagging = lagging/1000;
   border = (mass*PI*PI)/(1-cos(PI*lagging));
   QString valueAsString = QString::number(border);
   ui->label_calc->setText("(0; " + valueAsString + ")");
}

void Widget::on_push_start_clicked()
{
        if (was_begin)
            return;
        QString k1_str,
                k2_str,
                force_str;
        double force1 = 0, force2 = 0, lagging = ui->horizontalSlider_h->value(),
               mass = ui->horizontalSlider_m->value(),
               speed = ui->horizontalSlider_V->value();

        if (ui->lineEdit_k1->text().isEmpty()){
          ui->label_warning_k1->setText("Введите значение!");
          return;
        }else{
          ui->label_warning_k1->clear();
          k1_str = ui->lineEdit_k1->text();
        }

        if (ui->lineEdit_k2->text().isEmpty()){
          ui->label_warning_k2->setText("Введите значение!");
          return;
        }else{
          ui->label_warning_k2->clear();
          k2_str = ui->lineEdit_k2->text();
        }

        if (!ui->lineEdit_F->text().isEmpty()){
          force_str = ui->lineEdit_F->text();
          force1 = force_str.split(" ")[0].toInt();
        }

        if (!ui->lineEdit_F->text().isEmpty()){
          force_str = ui->lineEdit_F_2->text();
          force2 = force_str.split(" ")[0].toInt();
        }
        w = new Simulator;
        ui->gridLayout->addWidget(w);
        lagging = lagging/1000;

        w->beginSimulating(mass, lagging, k1_str.split(" ")[0].toInt(), k2_str.split(" ")[0].toInt(), force1, force2, speed);
        was_begin = 1;

        w2 = new Analysis(mass, lagging, k1_str.split(" ")[0].toInt(), k2_str.split(" ")[0].toInt(), force1, force2, speed);
        ui->gridLayout_analysis->addWidget(w2);


}

void Widget::on_push_stop_clicked()
{
    if (was_begin){
        was_begin = 0;
        delete w;
        delete w2;
    }
}


void Widget::on_tabWidget_currentChanged(int index)
{
}

Widget::~Widget()
{
    delete ui;
}

