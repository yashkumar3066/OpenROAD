/****************************************************************************
** Meta object code from reading C++ file 'mainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/gui/src/mainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/QList>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_gui__MainWindow_t {
    QByteArrayData data[103];
    char stringdata0[1252];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_gui__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_gui__MainWindow_t qt_meta_stringdata_gui__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 15), // "gui::MainWindow"
QT_MOC_LITERAL(1, 16, 11), // "blockLoaded"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 13), // "odb::dbBlock*"
QT_MOC_LITERAL(4, 43, 5), // "block"
QT_MOC_LITERAL(5, 49, 4), // "exit"
QT_MOC_LITERAL(6, 54, 4), // "hide"
QT_MOC_LITERAL(7, 59, 6), // "redraw"
QT_MOC_LITERAL(8, 66, 5), // "pause"
QT_MOC_LITERAL(9, 72, 7), // "timeout"
QT_MOC_LITERAL(10, 80, 16), // "selectionChanged"
QT_MOC_LITERAL(11, 97, 8), // "Selected"
QT_MOC_LITERAL(12, 106, 9), // "selection"
QT_MOC_LITERAL(13, 116, 16), // "highlightChanged"
QT_MOC_LITERAL(14, 133, 13), // "rulersChanged"
QT_MOC_LITERAL(15, 147, 19), // "displayUnitsChanged"
QT_MOC_LITERAL(16, 167, 14), // "dbu_per_micron"
QT_MOC_LITERAL(17, 182, 6), // "useDBU"
QT_MOC_LITERAL(18, 189, 12), // "saveSettings"
QT_MOC_LITERAL(19, 202, 11), // "setLocation"
QT_MOC_LITERAL(20, 214, 1), // "x"
QT_MOC_LITERAL(21, 216, 1), // "y"
QT_MOC_LITERAL(22, 218, 20), // "updateSelectedStatus"
QT_MOC_LITERAL(23, 239, 11), // "addSelected"
QT_MOC_LITERAL(24, 251, 12), // "SelectionSet"
QT_MOC_LITERAL(25, 264, 10), // "selections"
QT_MOC_LITERAL(26, 275, 11), // "setSelected"
QT_MOC_LITERAL(27, 287, 14), // "removeSelected"
QT_MOC_LITERAL(28, 302, 20), // "removeSelectedByType"
QT_MOC_LITERAL(29, 323, 11), // "std::string"
QT_MOC_LITERAL(30, 335, 4), // "type"
QT_MOC_LITERAL(31, 340, 17), // "show_connectivity"
QT_MOC_LITERAL(32, 358, 14), // "addHighlighted"
QT_MOC_LITERAL(33, 373, 10), // "highlights"
QT_MOC_LITERAL(34, 384, 15), // "highlight_group"
QT_MOC_LITERAL(35, 400, 17), // "removeHighlighted"
QT_MOC_LITERAL(36, 418, 8), // "addRuler"
QT_MOC_LITERAL(37, 427, 2), // "x0"
QT_MOC_LITERAL(38, 430, 2), // "y0"
QT_MOC_LITERAL(39, 433, 2), // "x1"
QT_MOC_LITERAL(40, 436, 2), // "y1"
QT_MOC_LITERAL(41, 439, 5), // "label"
QT_MOC_LITERAL(42, 445, 4), // "name"
QT_MOC_LITERAL(43, 450, 9), // "euclidian"
QT_MOC_LITERAL(44, 460, 11), // "deleteRuler"
QT_MOC_LITERAL(45, 472, 20), // "updateHighlightedSet"
QT_MOC_LITERAL(46, 493, 22), // "QList<const Selected*>"
QT_MOC_LITERAL(47, 516, 18), // "items_to_highlight"
QT_MOC_LITERAL(48, 535, 16), // "clearHighlighted"
QT_MOC_LITERAL(49, 552, 11), // "clearRulers"
QT_MOC_LITERAL(50, 564, 18), // "removeFromSelected"
QT_MOC_LITERAL(51, 583, 5), // "items"
QT_MOC_LITERAL(52, 589, 21), // "removeFromHighlighted"
QT_MOC_LITERAL(53, 611, 6), // "zoomTo"
QT_MOC_LITERAL(54, 618, 9), // "odb::Rect"
QT_MOC_LITERAL(55, 628, 8), // "rect_dbu"
QT_MOC_LITERAL(56, 637, 13), // "zoomInToItems"
QT_MOC_LITERAL(57, 651, 6), // "status"
QT_MOC_LITERAL(58, 658, 7), // "message"
QT_MOC_LITERAL(59, 666, 14), // "showFindDialog"
QT_MOC_LITERAL(60, 681, 14), // "showGotoDialog"
QT_MOC_LITERAL(61, 696, 8), // "showHelp"
QT_MOC_LITERAL(62, 705, 16), // "addToolbarButton"
QT_MOC_LITERAL(63, 722, 4), // "text"
QT_MOC_LITERAL(64, 727, 6), // "script"
QT_MOC_LITERAL(65, 734, 4), // "echo"
QT_MOC_LITERAL(66, 739, 19), // "removeToolbarButton"
QT_MOC_LITERAL(67, 759, 11), // "addMenuItem"
QT_MOC_LITERAL(68, 771, 4), // "path"
QT_MOC_LITERAL(69, 776, 8), // "shortcut"
QT_MOC_LITERAL(70, 785, 14), // "removeMenuItem"
QT_MOC_LITERAL(71, 800, 16), // "requestUserInput"
QT_MOC_LITERAL(72, 817, 5), // "title"
QT_MOC_LITERAL(73, 823, 8), // "question"
QT_MOC_LITERAL(74, 832, 14), // "anyObjectInSet"
QT_MOC_LITERAL(75, 847, 13), // "selection_set"
QT_MOC_LITERAL(76, 861, 17), // "odb::dbObjectType"
QT_MOC_LITERAL(77, 879, 8), // "obj_type"
QT_MOC_LITERAL(78, 888, 29), // "selectHighlightConnectedInsts"
QT_MOC_LITERAL(79, 918, 11), // "select_flag"
QT_MOC_LITERAL(80, 930, 28), // "selectHighlightConnectedNets"
QT_MOC_LITERAL(81, 959, 6), // "output"
QT_MOC_LITERAL(82, 966, 5), // "input"
QT_MOC_LITERAL(83, 972, 35), // "selectHighlightConnectedBuffe..."
QT_MOC_LITERAL(84, 1008, 10), // "timingCone"
QT_MOC_LITERAL(85, 1019, 12), // "Gui::odbTerm"
QT_MOC_LITERAL(86, 1032, 4), // "term"
QT_MOC_LITERAL(87, 1037, 5), // "fanin"
QT_MOC_LITERAL(88, 1043, 6), // "fanout"
QT_MOC_LITERAL(89, 1050, 18), // "timingPathsThrough"
QT_MOC_LITERAL(90, 1069, 22), // "std::set<Gui::odbTerm>"
QT_MOC_LITERAL(91, 1092, 5), // "terms"
QT_MOC_LITERAL(92, 1098, 15), // "registerHeatMap"
QT_MOC_LITERAL(93, 1114, 18), // "HeatMapDataSource*"
QT_MOC_LITERAL(94, 1133, 7), // "heatmap"
QT_MOC_LITERAL(95, 1141, 17), // "unregisterHeatMap"
QT_MOC_LITERAL(96, 1159, 9), // "setUseDBU"
QT_MOC_LITERAL(97, 1169, 7), // "use_dbu"
QT_MOC_LITERAL(98, 1177, 16), // "setClearLocation"
QT_MOC_LITERAL(99, 1194, 19), // "showApplicationFont"
QT_MOC_LITERAL(100, 1214, 17), // "showGlobalConnect"
QT_MOC_LITERAL(101, 1232, 10), // "openDesign"
QT_MOC_LITERAL(102, 1243, 8) // "setBlock"

    },
    "gui::MainWindow\0blockLoaded\0\0odb::dbBlock*\0"
    "block\0exit\0hide\0redraw\0pause\0timeout\0"
    "selectionChanged\0Selected\0selection\0"
    "highlightChanged\0rulersChanged\0"
    "displayUnitsChanged\0dbu_per_micron\0"
    "useDBU\0saveSettings\0setLocation\0x\0y\0"
    "updateSelectedStatus\0addSelected\0"
    "SelectionSet\0selections\0setSelected\0"
    "removeSelected\0removeSelectedByType\0"
    "std::string\0type\0show_connectivity\0"
    "addHighlighted\0highlights\0highlight_group\0"
    "removeHighlighted\0addRuler\0x0\0y0\0x1\0"
    "y1\0label\0name\0euclidian\0deleteRuler\0"
    "updateHighlightedSet\0QList<const Selected*>\0"
    "items_to_highlight\0clearHighlighted\0"
    "clearRulers\0removeFromSelected\0items\0"
    "removeFromHighlighted\0zoomTo\0odb::Rect\0"
    "rect_dbu\0zoomInToItems\0status\0message\0"
    "showFindDialog\0showGotoDialog\0showHelp\0"
    "addToolbarButton\0text\0script\0echo\0"
    "removeToolbarButton\0addMenuItem\0path\0"
    "shortcut\0removeMenuItem\0requestUserInput\0"
    "title\0question\0anyObjectInSet\0"
    "selection_set\0odb::dbObjectType\0"
    "obj_type\0selectHighlightConnectedInsts\0"
    "select_flag\0selectHighlightConnectedNets\0"
    "output\0input\0selectHighlightConnectedBufferTrees\0"
    "timingCone\0Gui::odbTerm\0term\0fanin\0"
    "fanout\0timingPathsThrough\0"
    "std::set<Gui::odbTerm>\0terms\0"
    "registerHeatMap\0HeatMapDataSource*\0"
    "heatmap\0unregisterHeatMap\0setUseDBU\0"
    "use_dbu\0setClearLocation\0showApplicationFont\0"
    "showGlobalConnect\0openDesign\0setBlock"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_gui__MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      64,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      10,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  334,    2, 0x06 /* Public */,
       5,    0,  337,    2, 0x06 /* Public */,
       6,    0,  338,    2, 0x06 /* Public */,
       7,    0,  339,    2, 0x06 /* Public */,
       8,    1,  340,    2, 0x06 /* Public */,
      10,    1,  343,    2, 0x06 /* Public */,
      10,    0,  346,    2, 0x26 /* Public | MethodCloned */,
      13,    0,  347,    2, 0x06 /* Public */,
      14,    0,  348,    2, 0x06 /* Public */,
      15,    2,  349,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      18,    0,  354,    2, 0x0a /* Public */,
      19,    2,  355,    2, 0x0a /* Public */,
      22,    1,  360,    2, 0x0a /* Public */,
      23,    1,  363,    2, 0x0a /* Public */,
      23,    1,  366,    2, 0x0a /* Public */,
      26,    1,  369,    2, 0x0a /* Public */,
      27,    1,  372,    2, 0x0a /* Public */,
      28,    1,  375,    2, 0x0a /* Public */,
      26,    2,  378,    2, 0x0a /* Public */,
      26,    1,  383,    2, 0x2a /* Public | MethodCloned */,
      32,    2,  386,    2, 0x0a /* Public */,
      32,    1,  391,    2, 0x2a /* Public | MethodCloned */,
      35,    1,  394,    2, 0x0a /* Public */,
      36,    7,  397,    2, 0x0a /* Public */,
      36,    6,  412,    2, 0x2a /* Public | MethodCloned */,
      36,    5,  425,    2, 0x2a /* Public | MethodCloned */,
      36,    4,  436,    2, 0x2a /* Public | MethodCloned */,
      44,    1,  445,    2, 0x0a /* Public */,
      45,    2,  448,    2, 0x0a /* Public */,
      45,    1,  453,    2, 0x2a /* Public | MethodCloned */,
      48,    1,  456,    2, 0x0a /* Public */,
      48,    0,  459,    2, 0x2a /* Public | MethodCloned */,
      49,    0,  460,    2, 0x0a /* Public */,
      50,    1,  461,    2, 0x0a /* Public */,
      52,    2,  464,    2, 0x0a /* Public */,
      52,    1,  469,    2, 0x2a /* Public | MethodCloned */,
      53,    1,  472,    2, 0x0a /* Public */,
      56,    1,  475,    2, 0x0a /* Public */,
      57,    1,  478,    2, 0x0a /* Public */,
      59,    0,  481,    2, 0x0a /* Public */,
      60,    0,  482,    2, 0x0a /* Public */,
      61,    0,  483,    2, 0x0a /* Public */,
      62,    4,  484,    2, 0x0a /* Public */,
      66,    1,  493,    2, 0x0a /* Public */,
      67,    6,  496,    2, 0x0a /* Public */,
      70,    1,  509,    2, 0x0a /* Public */,
      71,    2,  512,    2, 0x0a /* Public */,
      74,    2,  517,    2, 0x0a /* Public */,
      78,    2,  522,    2, 0x0a /* Public */,
      78,    1,  527,    2, 0x2a /* Public | MethodCloned */,
      80,    4,  530,    2, 0x0a /* Public */,
      80,    3,  539,    2, 0x2a /* Public | MethodCloned */,
      83,    2,  546,    2, 0x0a /* Public */,
      83,    1,  551,    2, 0x2a /* Public | MethodCloned */,
      84,    3,  554,    2, 0x0a /* Public */,
      89,    1,  561,    2, 0x0a /* Public */,
      92,    1,  564,    2, 0x0a /* Public */,
      95,    1,  567,    2, 0x0a /* Public */,
      96,    1,  570,    2, 0x08 /* Private */,
      98,    0,  573,    2, 0x08 /* Private */,
      99,    0,  574,    2, 0x08 /* Private */,
     100,    0,  575,    2, 0x08 /* Private */,
     101,    0,  576,    2, 0x08 /* Private */,
     102,    1,  577,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Bool,   16,   17,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   20,   21,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void, 0x80000000 | 24,   25,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 29,   30,
    QMetaType::Void, 0x80000000 | 11, QMetaType::Bool,   12,   31,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, 0x80000000 | 24, QMetaType::Int,   33,   34,
    QMetaType::Void, 0x80000000 | 24,   33,
    QMetaType::Void, 0x80000000 | 11,   12,
    0x80000000 | 29, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int, 0x80000000 | 29, 0x80000000 | 29, QMetaType::Bool,   37,   38,   39,   40,   41,   42,   43,
    0x80000000 | 29, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int, 0x80000000 | 29, 0x80000000 | 29,   37,   38,   39,   40,   41,   42,
    0x80000000 | 29, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int, 0x80000000 | 29,   37,   38,   39,   40,   41,
    0x80000000 | 29, QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int,   37,   38,   39,   40,
    QMetaType::Void, 0x80000000 | 29,   42,
    QMetaType::Void, 0x80000000 | 46, QMetaType::Int,   47,   34,
    QMetaType::Void, 0x80000000 | 46,   47,
    QMetaType::Void, QMetaType::Int,   34,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 46,   51,
    QMetaType::Void, 0x80000000 | 46, QMetaType::Int,   51,   34,
    QMetaType::Void, 0x80000000 | 46,   51,
    QMetaType::Void, 0x80000000 | 54,   55,
    QMetaType::Void, 0x80000000 | 46,   51,
    QMetaType::Void, 0x80000000 | 29,   58,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    0x80000000 | 29, 0x80000000 | 29, QMetaType::QString, QMetaType::QString, QMetaType::Bool,   42,   63,   64,   65,
    QMetaType::Void, 0x80000000 | 29,   42,
    0x80000000 | 29, 0x80000000 | 29, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::QString, QMetaType::Bool,   42,   68,   63,   64,   69,   65,
    QMetaType::Void, 0x80000000 | 29,   42,
    0x80000000 | 29, QMetaType::QString, QMetaType::QString,   72,   73,
    QMetaType::Bool, QMetaType::Bool, 0x80000000 | 76,   75,   77,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,   79,   34,
    QMetaType::Void, QMetaType::Bool,   79,
    QMetaType::Void, QMetaType::Bool, QMetaType::Bool, QMetaType::Bool, QMetaType::Int,   79,   81,   82,   34,
    QMetaType::Void, QMetaType::Bool, QMetaType::Bool, QMetaType::Bool,   79,   81,   82,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,   79,   34,
    QMetaType::Void, QMetaType::Bool,   79,
    QMetaType::Void, 0x80000000 | 85, QMetaType::Bool, QMetaType::Bool,   86,   87,   88,
    QMetaType::Void, 0x80000000 | 90,   91,
    QMetaType::Void, 0x80000000 | 93,   94,
    QMetaType::Void, 0x80000000 | 93,   94,
    QMetaType::Void, QMetaType::Bool,   97,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void gui::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->blockLoaded((*reinterpret_cast< odb::dbBlock*(*)>(_a[1]))); break;
        case 1: _t->exit(); break;
        case 2: _t->hide(); break;
        case 3: _t->redraw(); break;
        case 4: _t->pause((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->selectionChanged((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 6: _t->selectionChanged(); break;
        case 7: _t->highlightChanged(); break;
        case 8: _t->rulersChanged(); break;
        case 9: _t->displayUnitsChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 10: _t->saveSettings(); break;
        case 11: _t->setLocation((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 12: _t->updateSelectedStatus((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 13: _t->addSelected((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 14: _t->addSelected((*reinterpret_cast< const SelectionSet(*)>(_a[1]))); break;
        case 15: _t->setSelected((*reinterpret_cast< const SelectionSet(*)>(_a[1]))); break;
        case 16: _t->removeSelected((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 17: _t->removeSelectedByType((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 18: _t->setSelected((*reinterpret_cast< const Selected(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 19: _t->setSelected((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 20: _t->addHighlighted((*reinterpret_cast< const SelectionSet(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 21: _t->addHighlighted((*reinterpret_cast< const SelectionSet(*)>(_a[1]))); break;
        case 22: _t->removeHighlighted((*reinterpret_cast< const Selected(*)>(_a[1]))); break;
        case 23: { std::string _r = _t->addRuler((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< const std::string(*)>(_a[5])),(*reinterpret_cast< const std::string(*)>(_a[6])),(*reinterpret_cast< bool(*)>(_a[7])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 24: { std::string _r = _t->addRuler((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< const std::string(*)>(_a[5])),(*reinterpret_cast< const std::string(*)>(_a[6])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 25: { std::string _r = _t->addRuler((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])),(*reinterpret_cast< const std::string(*)>(_a[5])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 26: { std::string _r = _t->addRuler((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 27: _t->deleteRuler((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 28: _t->updateHighlightedSet((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 29: _t->updateHighlightedSet((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1]))); break;
        case 30: _t->clearHighlighted((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 31: _t->clearHighlighted(); break;
        case 32: _t->clearRulers(); break;
        case 33: _t->removeFromSelected((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1]))); break;
        case 34: _t->removeFromHighlighted((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 35: _t->removeFromHighlighted((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1]))); break;
        case 36: _t->zoomTo((*reinterpret_cast< const odb::Rect(*)>(_a[1]))); break;
        case 37: _t->zoomInToItems((*reinterpret_cast< const QList<const Selected*>(*)>(_a[1]))); break;
        case 38: _t->status((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 39: _t->showFindDialog(); break;
        case 40: _t->showGotoDialog(); break;
        case 41: _t->showHelp(); break;
        case 42: { std::string _r = _t->addToolbarButton((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3])),(*reinterpret_cast< bool(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 43: _t->removeToolbarButton((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 44: { std::string _r = _t->addMenuItem((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3])),(*reinterpret_cast< const QString(*)>(_a[4])),(*reinterpret_cast< const QString(*)>(_a[5])),(*reinterpret_cast< bool(*)>(_a[6])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 45: _t->removeMenuItem((*reinterpret_cast< const std::string(*)>(_a[1]))); break;
        case 46: { std::string _r = _t->requestUserInput((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< std::string*>(_a[0]) = std::move(_r); }  break;
        case 47: { bool _r = _t->anyObjectInSet((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< odb::dbObjectType(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 48: _t->selectHighlightConnectedInsts((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 49: _t->selectHighlightConnectedInsts((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 50: _t->selectHighlightConnectedNets((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 51: _t->selectHighlightConnectedNets((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 52: _t->selectHighlightConnectedBufferTrees((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 53: _t->selectHighlightConnectedBufferTrees((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 54: _t->timingCone((*reinterpret_cast< Gui::odbTerm(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< bool(*)>(_a[3]))); break;
        case 55: _t->timingPathsThrough((*reinterpret_cast< const std::set<Gui::odbTerm>(*)>(_a[1]))); break;
        case 56: _t->registerHeatMap((*reinterpret_cast< HeatMapDataSource*(*)>(_a[1]))); break;
        case 57: _t->unregisterHeatMap((*reinterpret_cast< HeatMapDataSource*(*)>(_a[1]))); break;
        case 58: _t->setUseDBU((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 59: _t->setClearLocation(); break;
        case 60: _t->showApplicationFont(); break;
        case 61: _t->showGlobalConnect(); break;
        case 62: _t->openDesign(); break;
        case 63: _t->setBlock((*reinterpret_cast< odb::dbBlock*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)(odb::dbBlock * );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::blockLoaded)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::exit)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::hide)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::redraw)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::pause)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(const Selected & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::selectionChanged)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::highlightChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rulersChanged)) {
                *result = 8;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(int , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::displayUnitsChanged)) {
                *result = 9;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject gui::MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_gui__MainWindow.data,
    qt_meta_data_gui__MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_gui__MainWindow.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "ord::OpenRoadObserver"))
        return static_cast< ord::OpenRoadObserver*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 64)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 64;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 64)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 64;
    }
    return _id;
}

// SIGNAL 0
void gui::MainWindow::blockLoaded(odb::dbBlock * _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void gui::MainWindow::exit()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void gui::MainWindow::hide()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void gui::MainWindow::redraw()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void gui::MainWindow::pause(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void gui::MainWindow::selectionChanged(const Selected & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 7
void gui::MainWindow::highlightChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 7, nullptr);
}

// SIGNAL 8
void gui::MainWindow::rulersChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 8, nullptr);
}

// SIGNAL 9
void gui::MainWindow::displayUnitsChanged(int _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
