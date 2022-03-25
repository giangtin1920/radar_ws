#include "radarscan.h"
#include "ui_radarscan.h"

radarScan::radarScan(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::radarScan)
{
  ui->setupUi(this);
  initTimer();
}

radarScan::~radarScan()
{
  delete ui;
}

void radarScan::initTimer()
{
    timerRadar = new QTimer(this);
    connect(timerRadar, SIGNAL(timeout()), this, SLOT(plotPoint()));

}

void radarScan::initGraphicRadar()
{
    // parameter Background Radar Scan
    paramGraphicRadar.sizeInPixel.x = 178;
    paramGraphicRadar.sizeInPixel.y = 350;
    paramGraphicRadar.sizeInMetre.x = 8;
    paramGraphicRadar.sizeInMetre.y = 15;
    paramGraphicRadar.sizeMarker.x = 30;
    paramGraphicRadar.sizeMarker.y = 30;
    paramGraphicRadar.posRadar.x = 119;
    paramGraphicRadar.posRadar.y = 417;
    paramGraphicRadar.ratioX = paramGraphicRadar.sizeInPixel.x / paramGraphicRadar.sizeInMetre.x;
    paramGraphicRadar.ratioY = paramGraphicRadar.sizeInPixel.y / paramGraphicRadar.sizeInMetre.y;
    paramGraphicRadar.offset = paramGraphicRadar.sizeMarker.x / 2;

    // show image
    graphicRadar = new QGraphicsScene();
    ui->graphicsView_radarScan->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_radarScan->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    radarScanBG =  new QGraphicsPixmapItem(QPixmap(":/radarScanBG/RadarScanBG.png").scaled(225, 450, Qt::KeepAspectRatio));
    graphicRadar->addItem(radarScanBG);
    ui->graphicsView_radarScan->setScene(graphicRadar);

    // read image
    imageRaw = QImage("RadarScanBGfindxy.png");
    for (int i = 0; i < test.numObj; i++)
    {
        markerObj = new QGraphicsPixmapItem(QPixmap(":/car/"+test.nameCar[i]+".png"));
        ttcObj = new QGraphicsTextItem(QString::number(nRound(test.ttc[i], 1)));
        vMarkerObj.push_back(markerObj);
        vttcObj.push_back(ttcObj);
    }
}

void radarScan::addPoint(double x, double y)
{
    qvX.push_back(x);
    qvY.push_back(y);
}

void radarScan::clearPoint()
{
    posX.clear();
    posY.clear();
}

void radarScan::plotPoint()
{
    clearVector();

    for (int i = 0; i < 2; i++)
    {
        test.isObj = 1;
        test.numObj +=1;
        test.IdObj.push_back(i);
        test.isApproach.push_back(i);
        test.posX.push_back(count - 4 +2*i);
        test.posY.push_back(sin(count)+5 +2*i);
        test.distance.push_back( sqrt(pow(test.posX[i],2) + pow(test.posY[i],2) )) ;
        test.velocity.push_back(sin(count)+2 +2*i);
        test.ttc.push_back(sin(count)+5 + 2*i);  //  4<ttc<6  6 <ttc<7

        if(test.ttc[i] < 5)
            test.nameCar.push_back("carAccidence");
        else if(test.ttc[i] < 5.5)
            test.nameCar.push_back("carWarning");
        else
            test.nameCar.push_back("carNormal");
    }
    count+=3.14156/36;
    if(count > 8) count = 0;
    initGraphicRadar();
    plotDetectObj();
}


// private slots in GUI ----------------------------

void radarScan::on_btnAdd_clicked()
{
    timerRadar->start(100);

}

void radarScan::on_btnClr_clicked()
{
    clearPoint();
    plotPoint();
}

void radarScan::on_btnFindxy_clicked()
{

//    clrCurrent.red();
//    clrCurrent.green();
//    clrCurrent.blue();

//    qInfo() << clrCurrent.red();
//    qInfo() << clrCurrent.green();
//    qInfo() << clrCurrent.blue();

}

void radarScan::plotDetectObj()
{
    float offset = paramGraphicRadar.offset;
    for (int i = 0; i < test.numObj; i++)
    {
        graphicRadar->addItem(vMarkerObj[i]);
        graphicRadar->addItem(vttcObj[i]);
        vMarkerObj[i]->setPos(  x2p(test.posX[i])-offset, y2p(test.posY[i])-offset  );
        vttcObj[i]->setPos(  x2p(test.posX[i])- offset+2, y2p(test.posY[i])-offset+4  );
        vttcObj[i]->setFont(QFont("Helvetica", 9, QFont::Bold));
        vttcObj[i]->setDefaultTextColor(carColor(test.nameCar[i]));
    }
    ui->graphicsView_radarScan->setScene(graphicRadar);
}

float radarScan::x2p(float x)
{
    return paramGraphicRadar.posRadar.x + paramGraphicRadar.ratioX * x;
}

float radarScan::y2p(float y)
{
    return paramGraphicRadar.posRadar.y - paramGraphicRadar.ratioX * y;
}

void radarScan::clearVector()
{
    vMarkerObj.clear();
    vttcObj.clear();
    test.isObj = 0;
    test.numObj = 0;
    test.IdObj.clear();
    test.isApproach.clear();
    test.posX.clear();
    test.posY.clear();
    test.distance.clear();
    test.velocity.clear();
    test.ttc.clear();
    test.nameCar.clear();

}

QString radarScan::carColor(QString name)
{
    if (name == "carNormal")
        return "white";
    else if (name == "carWarning")
        return "yellow";
    else
        return "red";
}

float radarScan::nRound(float num, int n)
{
    int base = pow(10,n);
    return round(num * base) /base;
}
