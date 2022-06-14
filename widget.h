#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtGui>
#include <math.h>
#include "physmodel.h"
#include "simulator.h"
#include "analysis.h"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);

    ~Widget();

private slots:

    void on_push_start_clicked();

    void on_push_stop_clicked();

    void on_pushButton_calc_clicked();


    void on_tabWidget_currentChanged(int index);

private:
    Ui::Widget *ui;
    Simulator *w;
    Analysis *w2;
    bool was_begin, curr_tab;
// Скорость перемещения окна  м/сек, это же средняя скорость головного самолета
};

#endif // WIDGET_H
