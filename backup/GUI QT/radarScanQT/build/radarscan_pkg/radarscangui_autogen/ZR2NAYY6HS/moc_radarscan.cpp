/****************************************************************************
** Meta object code from reading C++ file 'radarscan.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/radarscan_pkg/include/radarscan_pkg/radarscan.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'radarscan.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_radarScan_t {
    QByteArrayData data[21];
    char stringdata0[190];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_radarScan_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_radarScan_t qt_meta_stringdata_radarScan = {
    {
QT_MOC_LITERAL(0, 0, 9), // "radarScan"
QT_MOC_LITERAL(1, 10, 9), // "initTimer"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 16), // "initGraphicRadar"
QT_MOC_LITERAL(4, 38, 8), // "addPoint"
QT_MOC_LITERAL(5, 47, 1), // "x"
QT_MOC_LITERAL(6, 49, 1), // "y"
QT_MOC_LITERAL(7, 51, 10), // "clearPoint"
QT_MOC_LITERAL(8, 62, 9), // "plotPoint"
QT_MOC_LITERAL(9, 72, 17), // "on_btnAdd_clicked"
QT_MOC_LITERAL(10, 90, 17), // "on_btnClr_clicked"
QT_MOC_LITERAL(11, 108, 20), // "on_btnFindxy_clicked"
QT_MOC_LITERAL(12, 129, 13), // "plotDetectObj"
QT_MOC_LITERAL(13, 143, 3), // "x2p"
QT_MOC_LITERAL(14, 147, 3), // "y2p"
QT_MOC_LITERAL(15, 151, 11), // "clearVector"
QT_MOC_LITERAL(16, 163, 8), // "carColor"
QT_MOC_LITERAL(17, 172, 4), // "name"
QT_MOC_LITERAL(18, 177, 6), // "nRound"
QT_MOC_LITERAL(19, 184, 3), // "num"
QT_MOC_LITERAL(20, 188, 1) // "n"

    },
    "radarScan\0initTimer\0\0initGraphicRadar\0"
    "addPoint\0x\0y\0clearPoint\0plotPoint\0"
    "on_btnAdd_clicked\0on_btnClr_clicked\0"
    "on_btnFindxy_clicked\0plotDetectObj\0"
    "x2p\0y2p\0clearVector\0carColor\0name\0"
    "nRound\0num\0n"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_radarScan[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   84,    2, 0x08 /* Private */,
       3,    0,   85,    2, 0x08 /* Private */,
       4,    2,   86,    2, 0x08 /* Private */,
       7,    0,   91,    2, 0x08 /* Private */,
       8,    0,   92,    2, 0x08 /* Private */,
       9,    0,   93,    2, 0x08 /* Private */,
      10,    0,   94,    2, 0x08 /* Private */,
      11,    0,   95,    2, 0x08 /* Private */,
      12,    0,   96,    2, 0x08 /* Private */,
      13,    1,   97,    2, 0x08 /* Private */,
      14,    1,  100,    2, 0x08 /* Private */,
      15,    0,  103,    2, 0x08 /* Private */,
      16,    1,  104,    2, 0x08 /* Private */,
      18,    2,  107,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    5,    6,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Float, QMetaType::Float,    5,
    QMetaType::Float, QMetaType::Float,    6,
    QMetaType::Void,
    QMetaType::QString, QMetaType::QString,   17,
    QMetaType::Float, QMetaType::Float, QMetaType::Int,   19,   20,

       0        // eod
};

void radarScan::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        radarScan *_t = static_cast<radarScan *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->initTimer(); break;
        case 1: _t->initGraphicRadar(); break;
        case 2: _t->addPoint((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 3: _t->clearPoint(); break;
        case 4: _t->plotPoint(); break;
        case 5: _t->on_btnAdd_clicked(); break;
        case 6: _t->on_btnClr_clicked(); break;
        case 7: _t->on_btnFindxy_clicked(); break;
        case 8: _t->plotDetectObj(); break;
        case 9: { float _r = _t->x2p((*reinterpret_cast< float(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        case 10: { float _r = _t->y2p((*reinterpret_cast< float(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        case 11: _t->clearVector(); break;
        case 12: { QString _r = _t->carColor((*reinterpret_cast< QString(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 13: { float _r = _t->nRound((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = std::move(_r); }  break;
        default: ;
        }
    }
}

const QMetaObject radarScan::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_radarScan.data,
      qt_meta_data_radarScan,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *radarScan::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *radarScan::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_radarScan.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int radarScan::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
