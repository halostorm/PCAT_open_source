/****************************************************************************
** Meta object code from reading C++ file 'rviz_cloud_annotation_plugin.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/rviz_cloud_annotation/src/rviz_cloud_annotation_plugin.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'rviz_cloud_annotation_plugin.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation_t {
    QByteArrayData data[64];
    char stringdata0[909];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation_t qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation = {
    {
QT_MOC_LITERAL(0, 0, 43), // "rviz_cloud_annotation::QRVizC..."
QT_MOC_LITERAL(1, 44, 13), // "onSetEditMode"
QT_MOC_LITERAL(2, 58, 0), // ""
QT_MOC_LITERAL(3, 59, 4), // "mode"
QT_MOC_LITERAL(4, 64, 13), // "onSetToolType"
QT_MOC_LITERAL(5, 78, 4), // "type"
QT_MOC_LITERAL(6, 83, 19), // "onSetAnnotationType"
QT_MOC_LITERAL(7, 103, 6), // "uint32"
QT_MOC_LITERAL(8, 110, 15), // "annotation_type"
QT_MOC_LITERAL(9, 126, 21), // "onLabelButtonSelected"
QT_MOC_LITERAL(10, 148, 2), // "id"
QT_MOC_LITERAL(11, 151, 11), // "onPlusLabel"
QT_MOC_LITERAL(12, 163, 12), // "onMinusLabel"
QT_MOC_LITERAL(13, 176, 8), // "onPageUp"
QT_MOC_LITERAL(14, 185, 10), // "onPageDown"
QT_MOC_LITERAL(15, 196, 6), // "onSave"
QT_MOC_LITERAL(16, 203, 9), // "onRestore"
QT_MOC_LITERAL(17, 213, 7), // "onClear"
QT_MOC_LITERAL(18, 221, 5), // "onNew"
QT_MOC_LITERAL(19, 227, 14), // "onClearCurrent"
QT_MOC_LITERAL(20, 242, 6), // "onUndo"
QT_MOC_LITERAL(21, 249, 6), // "onRedo"
QT_MOC_LITERAL(22, 256, 10), // "onSendName"
QT_MOC_LITERAL(23, 267, 18), // "onViewCloudToggled"
QT_MOC_LITERAL(24, 286, 7), // "checked"
QT_MOC_LITERAL(25, 294, 26), // "onViewControlPointsToggled"
QT_MOC_LITERAL(26, 321, 19), // "onViewLabelsToggled"
QT_MOC_LITERAL(27, 341, 17), // "onGotoFirstUnused"
QT_MOC_LITERAL(28, 359, 16), // "onGotoLastUnused"
QT_MOC_LITERAL(29, 376, 11), // "onGotoFirst"
QT_MOC_LITERAL(30, 388, 16), // "onGotoNextUnused"
QT_MOC_LITERAL(31, 405, 15), // "onSmallerPoints"
QT_MOC_LITERAL(32, 421, 14), // "onBiggerPoints"
QT_MOC_LITERAL(33, 436, 17), // "onResetPointsSize"
QT_MOC_LITERAL(34, 454, 23), // "onControlYawSliderMoved"
QT_MOC_LITERAL(35, 478, 9), // "new_value"
QT_MOC_LITERAL(36, 488, 21), // "onControlYawSliderSet"
QT_MOC_LITERAL(37, 510, 18), // "onSetControlMaxYaw"
QT_MOC_LITERAL(38, 529, 15), // "std_msgs::Int32"
QT_MOC_LITERAL(39, 545, 3), // "msg"
QT_MOC_LITERAL(40, 549, 18), // "onSetControlMinYaw"
QT_MOC_LITERAL(41, 568, 8), // "onYawInc"
QT_MOC_LITERAL(42, 577, 8), // "onYawDec"
QT_MOC_LITERAL(43, 586, 8), // "onBiasX1"
QT_MOC_LITERAL(44, 595, 8), // "onBiasX2"
QT_MOC_LITERAL(45, 604, 8), // "onBiasX3"
QT_MOC_LITERAL(46, 613, 8), // "onBiasX4"
QT_MOC_LITERAL(47, 622, 8), // "onBiasY1"
QT_MOC_LITERAL(48, 631, 8), // "onBiasY2"
QT_MOC_LITERAL(49, 640, 8), // "onBiasY3"
QT_MOC_LITERAL(50, 649, 8), // "onBiasY4"
QT_MOC_LITERAL(51, 658, 8), // "onBiasZ1"
QT_MOC_LITERAL(52, 667, 8), // "onBiasZ2"
QT_MOC_LITERAL(53, 676, 8), // "onBiasZ3"
QT_MOC_LITERAL(54, 685, 8), // "onBiasZ4"
QT_MOC_LITERAL(55, 694, 13), // "onSetBiasZero"
QT_MOC_LITERAL(56, 708, 15), // "std_msgs::Empty"
QT_MOC_LITERAL(57, 724, 31), // "onControlPointWeightSliderMoved"
QT_MOC_LITERAL(58, 756, 29), // "onControlPointWeightSliderSet"
QT_MOC_LITERAL(59, 786, 23), // "onControlPointWeightInc"
QT_MOC_LITERAL(60, 810, 23), // "onControlPointWeightDec"
QT_MOC_LITERAL(61, 834, 23), // "onControlPointWeightMax"
QT_MOC_LITERAL(62, 858, 23), // "onControlPointWeightMin"
QT_MOC_LITERAL(63, 882, 26) // "onSetControlPointMaxWeight"

    },
    "rviz_cloud_annotation::QRVizCloudAnnotation\0"
    "onSetEditMode\0\0mode\0onSetToolType\0"
    "type\0onSetAnnotationType\0uint32\0"
    "annotation_type\0onLabelButtonSelected\0"
    "id\0onPlusLabel\0onMinusLabel\0onPageUp\0"
    "onPageDown\0onSave\0onRestore\0onClear\0"
    "onNew\0onClearCurrent\0onUndo\0onRedo\0"
    "onSendName\0onViewCloudToggled\0checked\0"
    "onViewControlPointsToggled\0"
    "onViewLabelsToggled\0onGotoFirstUnused\0"
    "onGotoLastUnused\0onGotoFirst\0"
    "onGotoNextUnused\0onSmallerPoints\0"
    "onBiggerPoints\0onResetPointsSize\0"
    "onControlYawSliderMoved\0new_value\0"
    "onControlYawSliderSet\0onSetControlMaxYaw\0"
    "std_msgs::Int32\0msg\0onSetControlMinYaw\0"
    "onYawInc\0onYawDec\0onBiasX1\0onBiasX2\0"
    "onBiasX3\0onBiasX4\0onBiasY1\0onBiasY2\0"
    "onBiasY3\0onBiasY4\0onBiasZ1\0onBiasZ2\0"
    "onBiasZ3\0onBiasZ4\0onSetBiasZero\0"
    "std_msgs::Empty\0onControlPointWeightSliderMoved\0"
    "onControlPointWeightSliderSet\0"
    "onControlPointWeightInc\0onControlPointWeightDec\0"
    "onControlPointWeightMax\0onControlPointWeightMin\0"
    "onSetControlPointMaxWeight"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz_cloud_annotation__QRVizCloudAnnotation[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      52,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,  274,    2, 0x08 /* Private */,
       4,    1,  277,    2, 0x08 /* Private */,
       6,    1,  280,    2, 0x08 /* Private */,
       9,    1,  283,    2, 0x08 /* Private */,
      11,    0,  286,    2, 0x08 /* Private */,
      12,    0,  287,    2, 0x08 /* Private */,
      13,    0,  288,    2, 0x08 /* Private */,
      14,    0,  289,    2, 0x08 /* Private */,
      15,    0,  290,    2, 0x08 /* Private */,
      16,    0,  291,    2, 0x08 /* Private */,
      17,    0,  292,    2, 0x08 /* Private */,
      18,    0,  293,    2, 0x08 /* Private */,
      19,    0,  294,    2, 0x08 /* Private */,
      20,    0,  295,    2, 0x08 /* Private */,
      21,    0,  296,    2, 0x08 /* Private */,
      22,    0,  297,    2, 0x08 /* Private */,
      23,    1,  298,    2, 0x08 /* Private */,
      25,    1,  301,    2, 0x08 /* Private */,
      26,    1,  304,    2, 0x08 /* Private */,
      27,    0,  307,    2, 0x08 /* Private */,
      28,    0,  308,    2, 0x08 /* Private */,
      29,    0,  309,    2, 0x08 /* Private */,
      30,    0,  310,    2, 0x08 /* Private */,
      31,    0,  311,    2, 0x08 /* Private */,
      32,    0,  312,    2, 0x08 /* Private */,
      33,    0,  313,    2, 0x08 /* Private */,
      34,    1,  314,    2, 0x08 /* Private */,
      36,    1,  317,    2, 0x08 /* Private */,
      37,    1,  320,    2, 0x08 /* Private */,
      40,    1,  323,    2, 0x08 /* Private */,
      41,    0,  326,    2, 0x08 /* Private */,
      42,    0,  327,    2, 0x08 /* Private */,
      43,    0,  328,    2, 0x08 /* Private */,
      44,    0,  329,    2, 0x08 /* Private */,
      45,    0,  330,    2, 0x08 /* Private */,
      46,    0,  331,    2, 0x08 /* Private */,
      47,    0,  332,    2, 0x08 /* Private */,
      48,    0,  333,    2, 0x08 /* Private */,
      49,    0,  334,    2, 0x08 /* Private */,
      50,    0,  335,    2, 0x08 /* Private */,
      51,    0,  336,    2, 0x08 /* Private */,
      52,    0,  337,    2, 0x08 /* Private */,
      53,    0,  338,    2, 0x08 /* Private */,
      54,    0,  339,    2, 0x08 /* Private */,
      55,    1,  340,    2, 0x08 /* Private */,
      57,    1,  343,    2, 0x08 /* Private */,
      58,    1,  346,    2, 0x08 /* Private */,
      59,    0,  349,    2, 0x08 /* Private */,
      60,    0,  350,    2, 0x08 /* Private */,
      61,    0,  351,    2, 0x08 /* Private */,
      62,    0,  352,    2, 0x08 /* Private */,
      63,    1,  353,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, 0x80000000 | 7,    8,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   24,
    QMetaType::Void, QMetaType::Bool,   24,
    QMetaType::Void, QMetaType::Bool,   24,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   35,
    QMetaType::Void, QMetaType::Int,   35,
    QMetaType::Void, 0x80000000 | 38,   39,
    QMetaType::Void, 0x80000000 | 38,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 56,   39,
    QMetaType::Void, QMetaType::Int,   35,
    QMetaType::Void, QMetaType::Int,   35,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 38,   39,

       0        // eod
};

void rviz_cloud_annotation::QRVizCloudAnnotation::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        QRVizCloudAnnotation *_t = static_cast<QRVizCloudAnnotation *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->onSetEditMode((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->onSetToolType((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->onSetAnnotationType((*reinterpret_cast< uint32(*)>(_a[1]))); break;
        case 3: _t->onLabelButtonSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->onPlusLabel(); break;
        case 5: _t->onMinusLabel(); break;
        case 6: _t->onPageUp(); break;
        case 7: _t->onPageDown(); break;
        case 8: _t->onSave(); break;
        case 9: _t->onRestore(); break;
        case 10: _t->onClear(); break;
        case 11: _t->onNew(); break;
        case 12: _t->onClearCurrent(); break;
        case 13: _t->onUndo(); break;
        case 14: _t->onRedo(); break;
        case 15: _t->onSendName(); break;
        case 16: _t->onViewCloudToggled((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        case 17: _t->onViewControlPointsToggled((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        case 18: _t->onViewLabelsToggled((*reinterpret_cast< const bool(*)>(_a[1]))); break;
        case 19: _t->onGotoFirstUnused(); break;
        case 20: _t->onGotoLastUnused(); break;
        case 21: _t->onGotoFirst(); break;
        case 22: _t->onGotoNextUnused(); break;
        case 23: _t->onSmallerPoints(); break;
        case 24: _t->onBiggerPoints(); break;
        case 25: _t->onResetPointsSize(); break;
        case 26: _t->onControlYawSliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 27: _t->onControlYawSliderSet((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 28: _t->onSetControlMaxYaw((*reinterpret_cast< const std_msgs::Int32(*)>(_a[1]))); break;
        case 29: _t->onSetControlMinYaw((*reinterpret_cast< const std_msgs::Int32(*)>(_a[1]))); break;
        case 30: _t->onYawInc(); break;
        case 31: _t->onYawDec(); break;
        case 32: _t->onBiasX1(); break;
        case 33: _t->onBiasX2(); break;
        case 34: _t->onBiasX3(); break;
        case 35: _t->onBiasX4(); break;
        case 36: _t->onBiasY1(); break;
        case 37: _t->onBiasY2(); break;
        case 38: _t->onBiasY3(); break;
        case 39: _t->onBiasY4(); break;
        case 40: _t->onBiasZ1(); break;
        case 41: _t->onBiasZ2(); break;
        case 42: _t->onBiasZ3(); break;
        case 43: _t->onBiasZ4(); break;
        case 44: _t->onSetBiasZero((*reinterpret_cast< const std_msgs::Empty(*)>(_a[1]))); break;
        case 45: _t->onControlPointWeightSliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 46: _t->onControlPointWeightSliderSet((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 47: _t->onControlPointWeightInc(); break;
        case 48: _t->onControlPointWeightDec(); break;
        case 49: _t->onControlPointWeightMax(); break;
        case 50: _t->onControlPointWeightMin(); break;
        case 51: _t->onSetControlPointMaxWeight((*reinterpret_cast< const std_msgs::Int32(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject rviz_cloud_annotation::QRVizCloudAnnotation::staticMetaObject = {
    { &rviz::Panel::staticMetaObject, qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation.data,
      qt_meta_data_rviz_cloud_annotation__QRVizCloudAnnotation,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *rviz_cloud_annotation::QRVizCloudAnnotation::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz_cloud_annotation::QRVizCloudAnnotation::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_rviz_cloud_annotation__QRVizCloudAnnotation.stringdata0))
        return static_cast<void*>(const_cast< QRVizCloudAnnotation*>(this));
    return rviz::Panel::qt_metacast(_clname);
}

int rviz_cloud_annotation::QRVizCloudAnnotation::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 52)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 52;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 52)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 52;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
