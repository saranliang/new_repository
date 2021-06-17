#ifndef MYTHREAD_H
#define MYTHREAD_H
#include <QObject>
#include<QThread>
#include <QtWidgets/QMainWindow>

Mythread::Mythread(QObject *parent) : QThread(parent)
{
     Global=0;
}

#endif // MYTHREAD_H
