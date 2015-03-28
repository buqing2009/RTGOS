/****************************************************************************
** Meta object code from reading C++ file 'SegmentBuilder.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../SegmentBuilder.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SegmentBuilder.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SegmentBuilder[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   52,   57,   57, 0x08,
      58,   97,   57,   57, 0x08,
     103,  166,  172,   57, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SegmentBuilder[] = {
    "SegmentBuilder\0processOdometry(rtabmap::SensorData)\0"
    "data\0\0processStatistics(rtabmap::Statistics)\0"
    "stats\0"
    "getSupervoxelClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)\0"
    "cloud\0pcl::PointCloud<pcl::PointXYZRGB>::Ptr\0"
};

void SegmentBuilder::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SegmentBuilder *_t = static_cast<SegmentBuilder *>(_o);
        switch (_id) {
        case 0: _t->processOdometry((*reinterpret_cast< const rtabmap::SensorData(*)>(_a[1]))); break;
        case 1: _t->processStatistics((*reinterpret_cast< const rtabmap::Statistics(*)>(_a[1]))); break;
        case 2: { pcl::PointCloud<pcl::PointXYZRGB>::Ptr _r = _t->getSupervoxelClusters((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< pcl::PointCloud<pcl::PointXYZRGB>::Ptr*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SegmentBuilder::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SegmentBuilder::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_SegmentBuilder,
      qt_meta_data_SegmentBuilder, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SegmentBuilder::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SegmentBuilder::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SegmentBuilder::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SegmentBuilder))
        return static_cast<void*>(const_cast< SegmentBuilder*>(this));
    if (!strcmp(_clname, "UEventsHandler"))
        return static_cast< UEventsHandler*>(const_cast< SegmentBuilder*>(this));
    return QWidget::qt_metacast(_clname);
}

int SegmentBuilder::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
