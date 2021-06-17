/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[17];
    char stringdata0[330];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 12), // "slot_clicked"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 10), // "vtkObject*"
QT_MOC_LITERAL(4, 36, 22), // "on_open_source_clicked"
QT_MOC_LITERAL(5, 59, 23), // "on_change_point_clicked"
QT_MOC_LITERAL(6, 83, 19), // "on_open_txt_clicked"
QT_MOC_LITERAL(7, 103, 43), // "on_Translation_Estimate_curre..."
QT_MOC_LITERAL(8, 147, 4), // "arg1"
QT_MOC_LITERAL(9, 152, 16), // "on_Start_clicked"
QT_MOC_LITERAL(10, 169, 19), // "on_save_txt_clicked"
QT_MOC_LITERAL(11, 189, 23), // "on_MODE_ADD_TWO_clicked"
QT_MOC_LITERAL(12, 213, 23), // "on_Mode_Add_ONE_clicked"
QT_MOC_LITERAL(13, 237, 23), // "on_insert_point_clicked"
QT_MOC_LITERAL(14, 261, 23), // "on_select_point_clicked"
QT_MOC_LITERAL(15, 285, 23), // "on_delete_point_clicked"
QT_MOC_LITERAL(16, 309, 20) // "on_add_point_clicked"

    },
    "MainWindow\0slot_clicked\0\0vtkObject*\0"
    "on_open_source_clicked\0on_change_point_clicked\0"
    "on_open_txt_clicked\0"
    "on_Translation_Estimate_currentIndexChanged\0"
    "arg1\0on_Start_clicked\0on_save_txt_clicked\0"
    "on_MODE_ADD_TWO_clicked\0on_Mode_Add_ONE_clicked\0"
    "on_insert_point_clicked\0on_select_point_clicked\0"
    "on_delete_point_clicked\0on_add_point_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    4,   79,    2, 0x0a /* Public */,
       4,    0,   88,    2, 0x08 /* Private */,
       5,    0,   89,    2, 0x08 /* Private */,
       6,    0,   90,    2, 0x08 /* Private */,
       7,    1,   91,    2, 0x08 /* Private */,
       9,    0,   94,    2, 0x08 /* Private */,
      10,    0,   95,    2, 0x08 /* Private */,
      11,    0,   96,    2, 0x08 /* Private */,
      12,    0,   97,    2, 0x08 /* Private */,
      13,    0,   98,    2, 0x08 /* Private */,
      14,    0,   99,    2, 0x08 /* Private */,
      15,    0,  100,    2, 0x08 /* Private */,
      16,    0,  101,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, QMetaType::ULong, QMetaType::VoidStar, QMetaType::VoidStar,    2,    2,    2,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    8,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->slot_clicked((*reinterpret_cast< vtkObject*(*)>(_a[1])),(*reinterpret_cast< ulong(*)>(_a[2])),(*reinterpret_cast< void*(*)>(_a[3])),(*reinterpret_cast< void*(*)>(_a[4]))); break;
        case 1: _t->on_open_source_clicked(); break;
        case 2: _t->on_change_point_clicked(); break;
        case 3: _t->on_open_txt_clicked(); break;
        case 4: _t->on_Translation_Estimate_currentIndexChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->on_Start_clicked(); break;
        case 6: _t->on_save_txt_clicked(); break;
        case 7: _t->on_MODE_ADD_TWO_clicked(); break;
        case 8: _t->on_Mode_Add_ONE_clicked(); break;
        case 9: _t->on_insert_point_clicked(); break;
        case 10: _t->on_select_point_clicked(); break;
        case 11: _t->on_delete_point_clicked(); break;
        case 12: _t->on_add_point_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    &QMainWindow::staticMetaObject,
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
