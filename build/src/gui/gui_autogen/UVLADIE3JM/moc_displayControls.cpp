/****************************************************************************
** Meta object code from reading C++ file 'displayControls.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/gui/src/displayControls.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'displayControls.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gui__PatternButton_t {
    QByteArrayData data[1];
    char stringdata0[19];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__PatternButton_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__PatternButton_t qt_meta_stringdata_gui__PatternButton = {
    {
QT_MOC_LITERAL(0, 0, 18) // "gui::PatternButton"

    },
    "gui::PatternButton"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__PatternButton[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void gui::PatternButton::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject gui::PatternButton::staticMetaObject = { {
    QMetaObject::SuperData::link<QRadioButton::staticMetaObject>(),
    qt_meta_stringdata_gui__PatternButton.data,
    qt_meta_data_gui__PatternButton,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::PatternButton::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::PatternButton::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__PatternButton.stringdata0))
        return static_cast<void*>(this);
    return QRadioButton::qt_metacast(_clname);
}

int gui::PatternButton::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QRadioButton::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_gui__DisplayColorDialog_t {
    QByteArrayData data[4];
    char stringdata0[51];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__DisplayColorDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__DisplayColorDialog_t qt_meta_stringdata_gui__DisplayColorDialog = {
    {
QT_MOC_LITERAL(0, 0, 23), // "gui::DisplayColorDialog"
QT_MOC_LITERAL(1, 24, 12), // "acceptDialog"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 12) // "rejectDialog"

    },
    "gui::DisplayColorDialog\0acceptDialog\0"
    "\0rejectDialog"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__DisplayColorDialog[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x0a /* Public */,
       3,    0,   25,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void gui::DisplayColorDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DisplayColorDialog *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->acceptDialog(); break;
        case 1: _t->rejectDialog(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject gui::DisplayColorDialog::staticMetaObject = { {
    QMetaObject::SuperData::link<QDialog::staticMetaObject>(),
    qt_meta_stringdata_gui__DisplayColorDialog.data,
    qt_meta_data_gui__DisplayColorDialog,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::DisplayColorDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::DisplayColorDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__DisplayColorDialog.stringdata0))
        return static_cast<void*>(this);
    return QDialog::qt_metacast(_clname);
}

int gui::DisplayColorDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
struct qt_meta_stringdata_gui__DisplayControlModel_t {
    QByteArrayData data[1];
    char stringdata0[25];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__DisplayControlModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__DisplayControlModel_t qt_meta_stringdata_gui__DisplayControlModel = {
    {
QT_MOC_LITERAL(0, 0, 24) // "gui::DisplayControlModel"

    },
    "gui::DisplayControlModel"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__DisplayControlModel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void gui::DisplayControlModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    (void)_o;
    (void)_id;
    (void)_c;
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject gui::DisplayControlModel::staticMetaObject = { {
    QMetaObject::SuperData::link<QStandardItemModel::staticMetaObject>(),
    qt_meta_stringdata_gui__DisplayControlModel.data,
    qt_meta_data_gui__DisplayControlModel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::DisplayControlModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::DisplayControlModel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__DisplayControlModel.stringdata0))
        return static_cast<void*>(this);
    return QStandardItemModel::qt_metacast(_clname);
}

int gui::DisplayControlModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QStandardItemModel::qt_metacall(_c, _id, _a);
    return _id;
}
struct qt_meta_stringdata_gui__DisplayControls_t {
    QByteArrayData data[20];
    char stringdata0[235];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__DisplayControls_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__DisplayControls_t qt_meta_stringdata_gui__DisplayControls = {
    {
QT_MOC_LITERAL(0, 0, 20), // "gui::DisplayControls"
QT_MOC_LITERAL(1, 21, 7), // "changed"
QT_MOC_LITERAL(2, 29, 0), // ""
QT_MOC_LITERAL(3, 30, 8), // "selected"
QT_MOC_LITERAL(4, 39, 8), // "Selected"
QT_MOC_LITERAL(5, 48, 11), // "blockLoaded"
QT_MOC_LITERAL(6, 60, 13), // "odb::dbBlock*"
QT_MOC_LITERAL(7, 74, 5), // "block"
QT_MOC_LITERAL(8, 80, 15), // "setCurrentBlock"
QT_MOC_LITERAL(9, 96, 11), // "itemChanged"
QT_MOC_LITERAL(10, 108, 14), // "QStandardItem*"
QT_MOC_LITERAL(11, 123, 4), // "item"
QT_MOC_LITERAL(12, 128, 19), // "displayItemSelected"
QT_MOC_LITERAL(13, 148, 14), // "QItemSelection"
QT_MOC_LITERAL(14, 163, 9), // "selection"
QT_MOC_LITERAL(15, 173, 21), // "displayItemDblClicked"
QT_MOC_LITERAL(16, 195, 11), // "QModelIndex"
QT_MOC_LITERAL(17, 207, 5), // "index"
QT_MOC_LITERAL(18, 213, 15), // "itemContextMenu"
QT_MOC_LITERAL(19, 229, 5) // "point"

    },
    "gui::DisplayControls\0changed\0\0selected\0"
    "Selected\0blockLoaded\0odb::dbBlock*\0"
    "block\0setCurrentBlock\0itemChanged\0"
    "QStandardItem*\0item\0displayItemSelected\0"
    "QItemSelection\0selection\0displayItemDblClicked\0"
    "QModelIndex\0index\0itemContextMenu\0"
    "point"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__DisplayControls[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x06 /* Public */,
       3,    1,   55,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   58,    2, 0x0a /* Public */,
       8,    1,   61,    2, 0x0a /* Public */,
       9,    1,   64,    2, 0x0a /* Public */,
      12,    1,   67,    2, 0x0a /* Public */,
      15,    1,   70,    2, 0x0a /* Public */,
      18,    1,   73,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 4,    3,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void, QMetaType::QPoint,   19,

       0        // eod
};

void gui::DisplayControls::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DisplayControls *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->changed(); break;
        case 1: _t->selected((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 2: _t->blockLoaded((*reinterpret_cast< odb::dbBlock*(*)>(_a[1]))); break;
        case 3: _t->setCurrentBlock((*reinterpret_cast< odb::dbBlock*(*)>(_a[1]))); break;
        case 4: _t->itemChanged((*reinterpret_cast< QStandardItem*(*)>(_a[1]))); break;
        case 5: _t->displayItemSelected((*reinterpret_cast< const QItemSelection(*)>(_a[1]))); break;
        case 6: _t->displayItemDblClicked((*reinterpret_cast< const QModelIndex(*)>(_a[1]))); break;
        case 7: _t->itemContextMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 5:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< QItemSelection >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DisplayControls::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DisplayControls::changed)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DisplayControls::*)(const Selected & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DisplayControls::selected)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject gui::DisplayControls::staticMetaObject = { {
    QMetaObject::SuperData::link<QDockWidget::staticMetaObject>(),
    qt_meta_stringdata_gui__DisplayControls.data,
    qt_meta_data_gui__DisplayControls,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::DisplayControls::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::DisplayControls::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__DisplayControls.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "Options"))
        return static_cast< Options*>(this);
    if (!strcmp(_clname, "sta::dbNetworkObserver"))
        return static_cast< sta::dbNetworkObserver*>(this);
    if (!strcmp(_clname, "odb::dbBlockCallBackObj"))
        return static_cast< odb::dbBlockCallBackObj*>(this);
    return QDockWidget::qt_metacast(_clname);
}

int gui::DisplayControls::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDockWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void gui::DisplayControls::changed()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void gui::DisplayControls::selected(const Selected & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
