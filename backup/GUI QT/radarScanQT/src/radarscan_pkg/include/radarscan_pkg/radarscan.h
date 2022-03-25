#ifndef RADARSCAN_H
#define RADARSCAN_H

#include <QWidget>
#include <QApplication>
#include <QTimer>

#include <vector>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
using namespace std;

#include <QPixmap>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QDebug>

struct cordinate
{
    float x;
    float y;
};

struct paramGraphic
{
    cordinate sizeInPixel;
    cordinate sizeInMetre;
    cordinate sizeMarker;
    cordinate posRadar;
    float ratioX = 1;
    float ratioY = 1;
    float offset = 0;
};

struct Test
{
    int isObj;
    int numObj;
    vector<int> IdObj;
    vector<int> isApproach;
    vector<float> alpha;
    vector<float> posX;
    vector<float> posY;
    vector<float> distance;
    vector<float> velocity;
    vector<float> ttc;
    vector<QString> nameCar;
};

namespace Ui {
class radarScan;
}

class radarScan : public QWidget
{
  Q_OBJECT

public:
  explicit radarScan(QWidget *parent = nullptr);
  ~radarScan();

private slots:
    void initTimer();
    void initGraphicRadar();
    void addPoint(double x, double y);
    void clearPoint();
    void plotPoint();
    void on_btnAdd_clicked();
    void on_btnClr_clicked();

    void on_btnFindxy_clicked();
    void plotDetectObj();
    float x2p(float x);
    float y2p(float y);
    void clearVector();
    QString carColor(QString name);
    float nRound(float num, int n);

private:
  Ui::radarScan *ui;
  vector<double> qvX, qvY;
  QVector<double> idObj, dis, vel, isappr, alpha, posX, posY;
  float count = 0;
  QTimer *timerRadar;
  QImage imageRaw;
  paramGraphic paramGraphicRadar;
  Test test;

  // graphic
  QGraphicsScene *graphicRadar;
  QGraphicsPixmapItem *radarScanBG, *markerObj;
  vector<QGraphicsPixmapItem*> vMarkerObj;
  QGraphicsTextItem *ttcObj;
  vector<QGraphicsTextItem*> vttcObj;

};

#endif // RADARSCAN_H
