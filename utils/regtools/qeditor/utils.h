/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2014 by Amaury Pouly
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/
#ifndef AUX_H
#define AUX_H

#include <QEvent>
#include <QPaintEvent>
#include <QLineEdit>
#include <QValidator>
#include <QToolButton>
#include <QMenu>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QTableWidget>
#include <QToolBar>
#include <QLabel>
#include <QHBoxLayout>
#include <QItemEditorCreatorBase>
#include <QStyledItemDelegate>
#include <QComboBox>
#include <QFileDialog>
#include <QScrollBar>
#include "settings.h"
#include "backend.h"

class SocBitRangeValidator : public QValidator
{
    Q_OBJECT
public:
    SocBitRangeValidator(QObject *parent = 0);

    virtual void fixup(QString& input) const;
    virtual State validate(QString& input, int& pos) const;
    /* validate and return the interpreted value */
    State parse(const QString& input, int& last_bit, int& first_bit) const;
};

class SocFieldValidator : public QValidator
{
    Q_OBJECT
public:
    SocFieldValidator(QObject *parent = 0);
    SocFieldValidator(const soc_reg_field_t& field, QObject *parent = 0);

    virtual void fixup(QString& input) const;
    virtual State validate(QString& input, int& pos) const;
    /* validate and return the interpreted value */
    State parse(const QString& input, soc_word_t& val) const;

protected:
    soc_reg_field_t m_field;
};

class RegLineEdit : public QWidget
{
    Q_OBJECT
public:
    enum EditMode
    {
        Write, Set, Clear, Toggle
    };

    RegLineEdit(QWidget *parent = 0);
    ~RegLineEdit();
    void SetReadOnly(bool ro);
    void EnableSCT(bool en);
    void SetMode(EditMode mode);
    EditMode GetMode();
    QLineEdit *GetLineEdit();
    void setText(const QString& text);
    QString text() const;

    Q_PROPERTY(QString text READ text WRITE setText USER true)

protected slots:
    void OnWriteAct();
    void OnSetAct();
    void OnClearAct();
    void OnToggleAct();
protected:
    void ShowMode(bool show);
    void DoAutoHide();

    QHBoxLayout *m_layout;
    QToolButton *m_button;
    QLineEdit *m_edit;
    EditMode m_mode;
    bool m_has_sct;
    bool m_readonly;
    QMenu *m_menu;
};

class SocFieldItemDelegate : public QStyledItemDelegate
{
public:
    SocFieldItemDelegate(QObject *parent = 0):QStyledItemDelegate(parent), m_bitcount(32) {}
    SocFieldItemDelegate(const soc_reg_field_t& field, QObject *parent = 0)
        :QStyledItemDelegate(parent), m_bitcount(field.last_bit - field.first_bit + 1) {}

    virtual QString displayText(const QVariant& value, const QLocale& locale) const;
protected:
    int m_bitcount;
};

class SocFieldEditor : public QLineEdit
{
    Q_OBJECT
    Q_PROPERTY(uint field READ field WRITE setField USER true)
public:
    SocFieldEditor(const soc_reg_field_t& field, QWidget *parent = 0);
    virtual ~SocFieldEditor();

    uint field() const;
    void setField(uint field);
    void SetRegField(const soc_reg_field_t& field);

protected:
    SocFieldValidator *m_validator;
    uint m_field;
    soc_reg_field_t m_reg_field;
};

class SocFieldEditorCreator : public QItemEditorCreatorBase
{
public:
    SocFieldEditorCreator() { m_field.first_bit = 0; m_field.last_bit = 31; }
    SocFieldEditorCreator(const soc_reg_field_t& field):m_field(field) {}

    virtual QWidget *createWidget(QWidget *parent) const;
    virtual QByteArray valuePropertyName() const;

protected:
    soc_reg_field_t m_field;
};

class SocFieldCachedValue
{
public:
    SocFieldCachedValue():m_value(0) {}
    SocFieldCachedValue(const soc_reg_field_t& field, uint value);
    virtual ~SocFieldCachedValue() {}
    const soc_reg_field_t& field() const { return m_field; }
    uint value() const { return m_value; }
    /* return empty string if there no match */
    QString value_name() const { return m_name; }
protected:
    soc_reg_field_t m_field;
    uint m_value;
    QString m_name;
};

Q_DECLARE_METATYPE(SocFieldCachedValue)

class SocFieldBitRange
{
public:
    SocFieldBitRange():m_first_bit(0),m_last_bit(0) {}
    SocFieldBitRange(const soc_reg_field_t& field)
        :m_first_bit(field.first_bit), m_last_bit(field.last_bit) {}
    unsigned GetFirstBit() const { return m_first_bit; }
    unsigned GetLastBit() const { return m_last_bit; }
protected:
    unsigned m_first_bit, m_last_bit;
};

Q_DECLARE_METATYPE(SocFieldBitRange)

class SocFieldCachedItemDelegate : public QStyledItemDelegate
{
public:
    enum DisplayMode
    {
        DisplayValueAndName, /* "value (name)" or "value" if no name */
        DisplayName, /* "name" or "value" if no name */
        DisplayValue, /* "value" */
    };

    SocFieldCachedItemDelegate(QObject *parent = 0);
    virtual QString displayText(const QVariant& value, const QLocale& locale) const;
    void SetMode(DisplayMode mode) { m_mode = mode; }
    DisplayMode GetMode() const { return m_mode; }

protected:
    DisplayMode m_mode;
};

class SocFieldCachedEditor : public SocFieldEditor
{
    Q_OBJECT
    Q_PROPERTY(SocFieldCachedValue value READ value WRITE setValue USER true)
public:
    SocFieldCachedEditor(QWidget *parent = 0);
    virtual ~SocFieldCachedEditor();

    SocFieldCachedValue value() const;
    void setValue(SocFieldCachedValue field);
protected:
    SocFieldCachedValue m_value;
};

class SocFieldCachedEditorCreator : public QItemEditorCreatorBase
{
public:
    SocFieldCachedEditorCreator() {}

    virtual QWidget *createWidget(QWidget *parent) const;
    virtual QByteArray valuePropertyName() const;

protected:
};

struct RegThemeGroup
{
    QBrush foreground;
    QBrush background;
    QFont font;
};

struct RegTheme
{
    RegTheme():valid(false) {}
    bool valid;
    RegThemeGroup normal;
    RegThemeGroup diff;
    RegThemeGroup error;
};

class RegFieldTableModel : public QAbstractTableModel
{
    Q_OBJECT
public:
    RegFieldTableModel(QObject *parent);
    virtual int rowCount(const QModelIndex & parent = QModelIndex()) const;
    virtual int columnCount(const QModelIndex & parent = QModelIndex()) const;
    virtual QVariant data(const QModelIndex & index, int role) const;
    virtual QVariant headerData(int section, Qt::Orientation orientation, int role) const;
    virtual Qt::ItemFlags flags (const QModelIndex & index) const;
    virtual bool setData(const QModelIndex& index, const QVariant& value, int role);

    void SetRegister(const soc_reg_t& reg);
    /* values can either be an invalid QVariant() (means no value/error), or a
     * QVariant containing a soc_word_t */
    void SetValues(const QVector< QVariant >& values);
    QVariant GetValue(int index);
    void SetTheme(const RegTheme& theme);
    void SetReadOnly(bool en);

signals:
    void OnValueModified(int index);

protected:
    void RecomputeTheme();

    enum
    {
        BitRangeColumn = 0,
        NameColumn,
        FirstValueColumn,
        DescColumnOffset = FirstValueColumn, /* offset from nr_values */
        ColumnCountOffset, /* offset from nr_values */
    };

    enum ColorStatus
    {
        None,
        Normal,
        Diff,
        Error
    };

    soc_reg_t m_reg;
    QVector< QVariant > m_value;
    QVector< ColorStatus > m_status;
    RegTheme m_theme;
    bool m_read_only;
};

class RegSexyDisplay2 : public QAbstractItemView
{
    Q_OBJECT
public:
    RegSexyDisplay2(QWidget *parent = 0);
    virtual QModelIndex indexAt(const QPoint& point) const;
    virtual void scrollTo(const QModelIndex& index, ScrollHint hint = EnsureVisible);
    virtual QRect visualRect(const QModelIndex& index ) const;
    virtual void setModel(QAbstractItemModel *model);

protected slots:
    virtual void dataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight);
    virtual void rowsInserted(const QModelIndex &parent, int start, int end);
    virtual void rowsAboutToBeRemoved(const QModelIndex &parent, int start, int end);
    virtual void scrollContentsBy(int dx, int dy);
    virtual void updateGeometries();

protected:
    int GetMarginSize() const; // margin in cells
    int GetSeparatorSize() const; // size of lines betweens cells
    int GetColumnWidth() const; // width of a 1-bit column (excluding separators)
    int GetHeaderHeight() const; // height of the header (excluding separators)
    int GetGapHeight() const; // height of gap between header and fields
    int GetMaxContentHeight() const; // maximum height of field columns
    int GetHeaderTextSep() const; // height between digits in header
    void RecomputeGeometry();

    virtual bool isIndexHidden(const QModelIndex& index) const;
    virtual QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers);
    virtual void setSelection(const QRect& rect, QItemSelectionModel::SelectionFlags flags);
    virtual int verticalOffset() const;
    virtual int horizontalOffset() const;
    virtual QRegion visualRegionForSelection(const QItemSelection& selection) const;
    virtual void paintEvent(QPaintEvent *event);
    virtual void resizeEvent(QResizeEvent* event);

    bool m_is_dirty;
    int m_minimum_width, m_minimum_height;
};

/**
 * The Qt designers chose to make QAbstractItemView a QAbstractScrollArea, so
 * that the table scrolls when it doesn't fit. This might be a problem when
 * one wants to put several tables on top of another, and the whole set into a
 * big scrollable area, because QAbstractScrollArea provides dummy values as
 * (minimum) size hints...So the big scroll area has no way of knowing the actual
 * size of the widget inside and it ends being a scrollable table inside another
 * scrollable, which is just super weird.
 * The Unscroll<T> class provides a workaround this behaviour: it expects T
 * to derive from QAbstractScrollArea and provides correct (minimum) size hints,
 * based on the value of the scroll bars.
 */
template<typename T>
class Unscroll : public T
{
public:
    Unscroll(QWidget *parent = 0):T(parent) {}
    virtual QSize sizeHint() const
    {
        QScrollBar *hsb = this->horizontalScrollBar();
        QScrollBar *vsb = this->verticalScrollBar();
        int w = hsb->maximum() - hsb->minimum() + hsb->pageStep();
        int h = vsb->maximum() - vsb->minimum() + vsb->pageStep();
        QMargins m = this->contentsMargins();
        return QSize(m.left() + w + m.right(), m.top() + h + m.bottom());
    }

    virtual QSize minimumSizeHint() const
    {
        return sizeHint();
    }
};

class GrowingTableView : public QTableView
{
    Q_OBJECT
public:
    GrowingTableView(QWidget *parent = 0);
    virtual void setModel(QAbstractItemModel *model);

protected slots:
    void DataChanged(const QModelIndex& tl, const QModelIndex& br);
};

class MyTextEditor : public QWidget
{
    Q_OBJECT
public:
    MyTextEditor(QWidget *parent = 0);
    void SetGrowingMode(bool en);
    void SetReadOnly(bool ro);
    void SetTextHtml(const QString& text);
    QString GetTextHtml();
    bool IsModified();
signals:
    void OnTextChanged();

protected slots:
    void OnInternalTextChanged();
    void OnTextBold(bool checked);
    void OnTextItalic(bool checked);
    void OnTextUnderline(bool checked);
    void OnCharFormatChanged(const QTextCharFormat& fmt);

protected:
    bool m_growing_mode;
    bool m_read_only;
    QToolBar *m_toolbar;
    QTextEdit *m_edit;
    QToolButton *m_bold_button;
    QToolButton *m_italic_button;
    QToolButton *m_underline_button;
};

class BackendSelector : public QWidget
{
    Q_OBJECT
public:
    BackendSelector(Backend *backend, QWidget *parent = 0);
    virtual ~BackendSelector();
    IoBackend *GetBackend();
    void SetNothingMessage(const QString& msg);

signals:
    void OnSelect(IoBackend *backend);

protected:
    void ChangeBackend(IoBackend *new_backend);

    enum
    {
        DataSelNothing,
        DataSelFile,
#ifdef HAVE_HWSTUB
        DataSelDevice,
#endif
    };

    Backend *m_backend;
    IoBackend *m_io_backend;
    QComboBox *m_data_selector;
    QLineEdit *m_data_sel_edit;
#ifdef HAVE_HWSTUB
    QComboBox *m_dev_selector;
    HWStubBackendHelper m_hwstub_helper;
#endif
    QLabel *m_nothing_text;

private slots:
#ifdef HAVE_HWSTUB
    void OnDevListChanged();
    void OnDevChanged(int index);
    void OnDevListChanged2(bool, struct libusb_device *);
    void ClearDevList();
#endif
    void OnDataSelChanged(int index);
};

class MessageWidget : public QFrame
{
    Q_OBJECT
public:
    enum MessageType
    {
        Positive,
        Information,
        Warning,
        Error
    };

    MessageWidget(QWidget *parent = 0);
    virtual ~MessageWidget();
    /* returns message ID */
    int SetMessage(MessageType type, const QString& msg);
    /* clear message if ID match, nop otherwise */
    void HideMessage(int id);

protected:
    void UpdateType();

    QLabel *m_icon;
    QLabel *m_text;
    QToolButton *m_close;
    MessageType m_type;
    int m_id;

private slots:
    void OnClose(bool clicked);
};

#endif /* AUX_H */
