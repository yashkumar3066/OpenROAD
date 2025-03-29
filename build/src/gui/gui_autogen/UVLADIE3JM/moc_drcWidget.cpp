/****************************************************************************
** Meta object code from reading C++ file 'drcWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/gui/src/drcWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'drcWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gui__DRCWidget_t {
    QByteArrayData data[24];
    char stringdata0[249];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__DRCWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__DRCWidget_t qt_meta_stringdata_gui__DRCWidget = {
    {
QT_MOC_LITERAL(0, 0, 14), // "gui::DRCWidget"
QT_MOC_LITERAL(1, 15, 9), // "selectDRC"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 8), // "Selected"
QT_MOC_LITERAL(4, 35, 8), // "selected"
QT_MOC_LITERAL(5, 44, 5), // "focus"
QT_MOC_LITERAL(6, 50, 10), // "loadReport"
QT_MOC_LITERAL(7, 61, 8), // "filename"
QT_MOC_LITERAL(8, 70, 8), // "setBlock"
QT_MOC_LITERAL(9, 79, 13), // "odb::dbBlock*"
QT_MOC_LITERAL(10, 93, 5), // "block"
QT_MOC_LITERAL(11, 99, 7), // "clicked"
QT_MOC_LITERAL(12, 107, 11), // "QModelIndex"
QT_MOC_LITERAL(13, 119, 5), // "index"
QT_MOC_LITERAL(14, 125, 12), // "selectReport"
QT_MOC_LITERAL(15, 138, 14), // "toggleRenderer"
QT_MOC_LITERAL(16, 153, 7), // "visible"
QT_MOC_LITERAL(17, 161, 15), // "updateSelection"
QT_MOC_LITERAL(18, 177, 9), // "selection"
QT_MOC_LITERAL(19, 187, 16), // "selectionChanged"
QT_MOC_LITERAL(20, 204, 14), // "QItemSelection"
QT_MOC_LITERAL(21, 219, 10), // "deselected"
QT_MOC_LITERAL(22, 230, 10), // "focusIndex"
QT_MOC_LITERAL(23, 241, 7) // "defocus"

    },
    "gui::DRCWidget\0selectDRC\0\0Selected\0"
    "selected\0focus\0loadReport\0filename\0"
    "setBlock\0odb::dbBlock*\0block\0clicked\0"
    "QModelIndex\0index\0selectReport\0"
    "toggleRenderer\0visible\0updateSelection\0"
    "selection\0selectionChanged\0QItemSelection\0"
    "deselected\0focusIndex\0defocus"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__DRCWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       5,    1,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   75,    2, 0x0a /* Public */,
       8,    1,   78,    2, 0x0a /* Public */,
      11,    1,   81,    2, 0x0a /* Public */,
      14,    0,   84,    2, 0x0a /* Public */,
      15,    1,   85,    2, 0x0a /* Public */,
      17,    1,   88,    2, 0x0a /* Public */,
      19,    2,   91,    2, 0x0a /* Public */,
      22,    1,   96,    2, 0x08 /* Private */,
      23,    0,   99,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   16,
    QMetaType::Void, 0x80000000 | 3,   18,
    QMetaType::Void, 0x80000000 | 20, 0x80000000 | 20,    4,   21,
    QMetaType::Void, 0x80000000 | 12,   13,
    QMetaType::Void,

       0        // eod
};

void gui::DRCWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DRCWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->selectDRC((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 1: _t->focus((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 2: _t->loadReport((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->setBlock((*reinterpret_cast< odb::dbBlock*(*)>(_a[1]))); break;
        case 4: _t->clicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 5: _t->selectReport(); break;
        case 6: _t->toggleRenderer((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->updateSelection((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 8: _t->selectionChanged((*reinterpret_cast< const QItemSelection(*)>(_a[1])),(*reinterpret_cast< const QItemSelection(*)>(_a[2]))); break;
        case 9: _t->focusIndex((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 10: _t->defocus(); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 8:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 1:
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QItemSelection >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DRCWidget::*)(const Selected & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DRCWidget::selectDRC)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DRCWidget::*)(const Selected & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DRCWidget::focus)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject gui::DRCWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QDockWidget::staticMetaObject>(),
    qt_meta_stringdata_gui__DRCWidget.data,
    qt_meta_data_gui__DRCWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::DRCWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::DRCWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__DRCWidget.stringdata0))
        return static_cast<void*>(this);
    return QDockWidget::qt_metacast(_clname);
}

int gui::DRCWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void gui::DRCWidget::selectDRC(const Selected & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void gui::DRCWidget::focus(const Selected & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
